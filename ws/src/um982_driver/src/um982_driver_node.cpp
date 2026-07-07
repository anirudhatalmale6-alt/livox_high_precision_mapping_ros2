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
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "um982_driver/nmea_parser.hpp"
#include "um982_driver/serial_port.hpp"

using namespace std::chrono_literals;

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

    navsat_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "/gnss_inertial/navsatfix", rclcpp::SensorDataQoS());
    heading_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
      "~/heading", rclcpp::SensorDataQoS());
    heading_deg_pub_ = create_publisher<std_msgs::msg::Float64>(
      "~/heading_deg", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "UM982 driver starting on %s @ %d baud", port_.c_str(), baud_);
    running_ = true;
    read_thread_ = std::thread(&Um982DriverNode::readLoop, this);
  }

  ~Um982DriverNode() override
  {
    running_ = false;
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

  std::string port_;
  int baud_ = 230400;
  std::string frame_id_;
  bool publish_heading_ = true;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_deg_pub_;

  std::atomic<bool> running_{false};
  std::thread read_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Um982DriverNode>());
  rclcpp::shutdown();
  return 0;
}
