// UM982 ROS2 driver node.
//
// Reads the Unicore UM982 dual-antenna RTK receiver over serial and publishes:
//   * sensor_msgs/NavSatFix       on  /gnss_inertial/navsatfix   (RTK position)
//   * geometry_msgs/QuaternionStamped on ~/heading               (dual-ant yaw)
//   * std_msgs/Float64            on ~/heading_deg               (true heading)
//
// The NavSatFix topic is the drop-in replacement for the position stream that
// the original APX-15 driver produced, so the mapping node needs no change.
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "um982_driver/mavlink_rtcm.hpp"
#include "um982_driver/nmea_parser.hpp"
#include "um982_driver/ntrip_client.hpp"
#include "um982_driver/serial_port.hpp"

using namespace std::chrono_literals;
using um982::NtripClient;
using um982::NtripConfig;

class Um982DriverNode : public rclcpp::Node
{
public:
  Um982DriverNode()
  : Node("um982_driver")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_ = declare_parameter<int>("baud", 230400);
    frame_id_ = declare_parameter<std::string>("frame_id", "gnss");
    publish_heading_ = declare_parameter<bool>("publish_heading", true);
    // On connect, tell the UM982 to STREAM its dual-antenna heading message.
    // The receiver computes heading from ANT1/ANT2 automatically, but it will
    // not output it unless asked — without this the ~/heading topic exists but
    // never publishes, and the map falls back to the drifting IMU yaw. Requires
    // BOTH antennas connected with a good baseline fix.
    configure_heading_ = declare_parameter<bool>("configure_heading", true);

    // RTCM correction source for RTK. "none" = standalone single-point fix
    // (metre-level). "ntrip" = pull corrections from an NTRIP caster over the
    // network (e.g. a state CORS network). "serial" = read a raw RTCM3 stream
    // from another serial device (e.g. a radio link fed by your own base).
    // "mavlink" = extract GPS_RTCM_DATA off a MAVLink link (e.g. a Pixhawk that
    // already receives your base corrections over its SiK radio). Whatever the
    // source, the bytes are injected back into this UM982's port, flipping the
    // GGA fix quality from 1 to 4 (RTK fixed / centimetre).
    rtcm_source_ = declare_parameter<std::string>("rtcm_source", "none");
    NtripConfig ntrip_cfg;
    ntrip_cfg.host = declare_parameter<std::string>("ntrip_host", "");
    ntrip_cfg.port = declare_parameter<int>("ntrip_port", 2101);
    ntrip_cfg.mountpoint = declare_parameter<std::string>("ntrip_mountpoint", "");
    ntrip_cfg.user = declare_parameter<std::string>("ntrip_user", "");
    ntrip_cfg.password = declare_parameter<std::string>("ntrip_password", "");
    ntrip_cfg.send_gga = declare_parameter<bool>("ntrip_send_gga", true);
    rtcm_serial_port_ = declare_parameter<std::string>("rtcm_serial_port", "/dev/rtcm");
    rtcm_serial_baud_ = declare_parameter<int>("rtcm_serial_baud", 57600);
    mavlink_serial_port_ = declare_parameter<std::string>("mavlink_serial_port", "/dev/pixhawk");
    mavlink_serial_baud_ = declare_parameter<int>("mavlink_serial_baud", 57600);

    navsat_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "/gnss_inertial/navsatfix", rclcpp::SensorDataQoS());
    heading_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
      "~/heading", rclcpp::SensorDataQoS());
    heading_deg_pub_ = create_publisher<std_msgs::msg::Float64>(
      "~/heading_deg", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "UM982 driver starting on %s @ %d baud", port_.c_str(), baud_);
    running_ = true;
    read_thread_ = std::thread(&Um982DriverNode::readLoop, this);

    startRtcmSource(ntrip_cfg);
  }

  ~Um982DriverNode() override
  {
    running_ = false;
    if (ntrip_)
    {
      ntrip_->stop();
    }
    if (rtcm_thread_.joinable())
    {
      rtcm_thread_.join();
    }
    if (read_thread_.joinable())
    {
      read_thread_.join();
    }
  }

private:
  void readLoop()
  {
    while (running_ && rclcpp::ok())
    {
      um982::SerialPort serial;
      try
      {
        serial.open(port_, baud_);
        RCLCPP_INFO(get_logger(), "Connected to UM982 serial port.");
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(get_logger(), "%s. Retrying in 1s.", e.what());
        std::this_thread::sleep_for(1s);
        continue;
      }

      if (configure_heading_ && publish_heading_)
      {
        configureReceiverHeading();
      }

      std::string line;
      while (running_ && rclcpp::ok() && serial.isOpen())
      {
        if (!serial.readLine(line))
        {
          continue;  // timeout, keep going
        }
        if (line.empty())
        {
          continue;
        }
        handleSentence(line);
      }
    }
  }

  void handleSentence(const std::string & line)
  {
    const std::string type = um982::messageType(line);
    if (type == "GGA")
    {
      um982::GgaFix fix = um982::parseGga(line);
      if (fix.valid)
      {
        publishNavSatFix(fix);
        // Keep the latest positioned GGA so the NTRIP client can report the
        // rover's location to a VRS / network-RTK caster.
        {
          std::lock_guard<std::mutex> lk(gga_mtx_);
          last_gga_ = line;
        }
      }
    }
    else if (publish_heading_ && (type == "HDT" || type == "UNIHEADINGA"))
    {
      um982::Heading h = um982::parseHeading(line);
      if (h.valid)
      {
        publishHeading(h);
      }
    }
  }

  void publishNavSatFix(const um982::GgaFix & fix)
  {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    // Map the GGA fix quality onto NavSatStatus while also preserving the raw
    // quality integer (RTK fixed = 4, RTK float = 5) so the mapping node can
    // tell RTK-fixed frames apart if desired.
    switch (fix.quality)
    {
      case um982::FixQuality::Invalid:
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
      case um982::FixQuality::RtkFixed:
      case um982::FixQuality::RtkFloat:
      case um982::FixQuality::DgpsFix:
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        break;
      default:
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        break;
    }
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                         sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
                         sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO |
                         sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;

    msg.latitude = fix.latitude_deg;
    msg.longitude = fix.longitude_deg;
    msg.altitude = fix.altitude_m;

    // Rough covariance from HDOP (metres^2). Real accuracy comes from the RTK
    // solution; this is only advisory for downstream consumers.
    double sigma = (fix.hdop > 0.0 ? fix.hdop : 1.0) * 0.02;  // ~2cm * hdop
    double var = sigma * sigma;
    msg.position_covariance[0] = var;
    msg.position_covariance[4] = var;
    msg.position_covariance[8] = var * 4.0;  // vertical typically worse
    msg.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    navsat_pub_->publish(msg);
  }

  void publishHeading(const um982::Heading & h)
  {
    std_msgs::msg::Float64 deg;
    deg.data = h.heading_deg;
    heading_deg_pub_->publish(deg);

    // Encode true heading (yaw) as a quaternion about +Z.
    double yaw = h.heading_deg * M_PI / 180.0;
    geometry_msgs::msg::QuaternionStamped q;
    q.header.stamp = now();
    q.header.frame_id = frame_id_;
    q.quaternion.w = std::cos(yaw / 2.0);
    q.quaternion.x = 0.0;
    q.quaternion.y = 0.0;
    q.quaternion.z = std::sin(yaw / 2.0);
    heading_pub_->publish(q);
  }

  // --- RTCM correction handling (RTK) -------------------------------------

  void startRtcmSource(const NtripConfig & ntrip_cfg)
  {
    if (rtcm_source_ == "ntrip")
    {
      if (ntrip_cfg.host.empty() || ntrip_cfg.mountpoint.empty())
      {
        RCLCPP_WARN(get_logger(),
          "rtcm_source=ntrip but ntrip_host/ntrip_mountpoint are empty - "
          "no corrections will be pulled. Staying on single-point fix.");
        return;
      }
      auto info = [this](const std::string & m) { RCLCPP_INFO(get_logger(), "%s", m.c_str()); };
      auto warn = [this](const std::string & m) { RCLCPP_WARN(get_logger(), "%s", m.c_str()); };
      ntrip_ = std::make_unique<NtripClient>(
        ntrip_cfg,
        [this](const unsigned char * d, size_t l) { injectRtcm(d, l); },
        [this]() { return latestGga(); },
        info, warn);
      ntrip_->start();
      RCLCPP_INFO(get_logger(),
        "RTCM source: NTRIP %s:%d/%s (corrections -> %s)",
        ntrip_cfg.host.c_str(), ntrip_cfg.port, ntrip_cfg.mountpoint.c_str(),
        port_.c_str());
    }
    else if (rtcm_source_ == "serial")
    {
      rtcm_thread_ = std::thread(&Um982DriverNode::rtcmSerialLoop, this);
      RCLCPP_INFO(get_logger(),
        "RTCM source: serial bridge %s @ %d baud (corrections -> %s)",
        rtcm_serial_port_.c_str(), rtcm_serial_baud_, port_.c_str());
    }
    else if (rtcm_source_ == "mavlink")
    {
      rtcm_thread_ = std::thread(&Um982DriverNode::rtcmMavlinkLoop, this);
      RCLCPP_INFO(get_logger(),
        "RTCM source: MAVLink GPS_RTCM_DATA on %s @ %d baud (corrections -> %s)",
        mavlink_serial_port_.c_str(), mavlink_serial_baud_, port_.c_str());
    }
    else
    {
      RCLCPP_INFO(get_logger(),
        "RTCM source: none - running standalone single-point fix "
        "(set rtcm_source to 'ntrip', 'serial' or 'mavlink' for RTK).");
    }
  }

  // Reads a raw RTCM3 stream from a second serial device (e.g. a radio link fed
  // by your own base station) and injects it into the UM982.
  void rtcmSerialLoop()
  {
    while (running_ && rclcpp::ok())
    {
      um982::SerialPort src;
      try
      {
        src.open(rtcm_serial_port_, rtcm_serial_baud_, false);
        RCLCPP_INFO(get_logger(), "RTCM serial bridge connected on %s.",
                    rtcm_serial_port_.c_str());
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(get_logger(), "%s. Retrying in 1s.", e.what());
        std::this_thread::sleep_for(1s);
        continue;
      }

      unsigned char buf[2048];
      while (running_ && rclcpp::ok() && src.isOpen())
      {
        ssize_t n = src.readRaw(buf, sizeof(buf));
        if (n > 0)
        {
          injectRtcm(buf, static_cast<size_t>(n));
        }
        else if (n < 0)
        {
          break;  // error — reopen
        }
        // n == 0 is a read timeout; keep waiting for corrections.
      }
    }
  }

  // Reads a MAVLink stream from a flight controller (e.g. a Pixhawk fed base
  // corrections over its SiK radio), extracts the GPS_RTCM_DATA messages and
  // injects the reassembled RTCM3 into the UM982.
  void rtcmMavlinkLoop()
  {
    um982::MavlinkRtcm extractor(
      [this](const unsigned char * d, size_t l) { injectRtcm(d, l); });

    while (running_ && rclcpp::ok())
    {
      um982::SerialPort src;
      try
      {
        src.open(mavlink_serial_port_, mavlink_serial_baud_, false);
        RCLCPP_INFO(get_logger(), "MAVLink link connected on %s.",
                    mavlink_serial_port_.c_str());
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(get_logger(), "%s. Retrying in 1s.", e.what());
        std::this_thread::sleep_for(1s);
        continue;
      }

      unsigned char buf[2048];
      while (running_ && rclcpp::ok() && src.isOpen())
      {
        ssize_t n = src.readRaw(buf, sizeof(buf));
        if (n > 0)
        {
          extractor.feed(buf, static_cast<size_t>(n));
        }
        else if (n < 0)
        {
          break;  // error — reopen
        }
      }
    }
  }

  // Write RTCM bytes into the UM982's serial port. Opens a dedicated writable
  // handle on the same device lazily and reopens it if a write ever fails.
  void injectRtcm(const unsigned char * data, size_t len)
  {
    std::lock_guard<std::mutex> lk(writer_mtx_);
    if (!writer_.isOpen())
    {
      try
      {
        writer_.open(port_, baud_, true);
        RCLCPP_INFO(get_logger(), "RTCM injection channel open on %s.", port_.c_str());
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(get_logger(), "RTCM injection: %s", e.what());
        return;
      }
    }
    if (!writer_.writeRaw(data, len))
    {
      RCLCPP_WARN(get_logger(), "RTCM injection write failed - will reopen.");
      writer_.close();
      return;
    }
    rtcm_bytes_ += len;
    if (rtcm_bytes_ - rtcm_bytes_logged_ >= 50000)
    {
      rtcm_bytes_logged_ = rtcm_bytes_;
      RCLCPP_INFO(get_logger(), "RTCM corrections flowing (%zu KB injected).",
                  rtcm_bytes_ / 1024);
    }
  }

  // Ask the UM982 to stream its dual-antenna heading. Reuses the same writable
  // handle as RTCM injection. Sent on every (re)connect at runtime — we do NOT
  // SAVECONFIG, to avoid flash wear and to leave the receiver's saved config
  // untouched. Both the NMEA (GPHDT) and Unicore (UNIHEADINGA) heading messages
  // are enabled; this driver parses either, so whichever the firmware supports
  // will flow. Harmless if only one antenna is connected (no heading solution =
  // the message carries no valid heading and nothing is published).
  void configureReceiverHeading()
  {
    std::lock_guard<std::mutex> lk(writer_mtx_);
    if (!writer_.isOpen())
    {
      try
      {
        writer_.open(port_, baud_, true);
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(get_logger(), "Heading config: cannot open %s for write: %s",
                    port_.c_str(), e.what());
        return;
      }
    }
    static const char * const kCmds[] = {
      "LOG GPHDT ONTIME 0.1\r\n",
      "LOG UNIHEADINGA ONTIME 0.1\r\n",
    };
    for (const char * cmd : kCmds)
    {
      writer_.writeStr(cmd);
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(get_logger(),
                "Requested UM982 heading output (GPHDT + UNIHEADINGA @10Hz). "
                "Needs both antennas with a good baseline to produce a value.");
  }

  std::string latestGga()
  {
    std::lock_guard<std::mutex> lk(gga_mtx_);
    return last_gga_;
  }

  std::string port_;
  int baud_ = 230400;
  std::string frame_id_;
  bool publish_heading_ = true;
  bool configure_heading_ = true;

  std::string rtcm_source_ = "none";
  std::string rtcm_serial_port_;
  int rtcm_serial_baud_ = 57600;
  std::string mavlink_serial_port_;
  int mavlink_serial_baud_ = 57600;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_deg_pub_;

  std::unique_ptr<NtripClient> ntrip_;
  um982::SerialPort writer_;
  std::mutex writer_mtx_;
  size_t rtcm_bytes_ = 0;
  size_t rtcm_bytes_logged_ = 0;

  std::mutex gga_mtx_;
  std::string last_gga_;

  std::atomic<bool> running_{false};
  std::thread read_thread_;
  std::thread rtcm_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Um982DriverNode>());
  rclcpp::shutdown();
  return 0;
}
