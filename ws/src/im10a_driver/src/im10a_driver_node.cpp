// IM10A ROS2 driver node.
//
// Reads the Hiwonder IM10A 10-axis IMU over serial (WitMotion 0x55 protocol)
// and publishes a standard sensor_msgs/Imu on "/imu/data".
//
// This is a from-scratch, dependency-free replacement for the vendor ROS2
// driver. Publishing on "/imu/data" makes it a drop-in: the existing
// imu_gnss_adapter already subscribes there and republishes to
// "/gnss_inertial/imu", so the rest of the mapping pipeline is unchanged.
#include <atomic>
#include <array>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "im10a_driver/serial_port.hpp"
#include "im10a_driver/witmotion_parser.hpp"

using namespace std::chrono_literals;

class Im10aDriverNode : public rclcpp::Node
{
public:
  Im10aDriverNode()
  : Node("im10a_driver")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_ = declare_parameter<int>("baud", 115200);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu");
    publish_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");

    // Fixed covariances. The IM10A does not report per-axis variance, so we
    // publish conservative constants; downstream fusion can override.
    orientation_stddev_ = declare_parameter<double>("orientation_stddev", 0.02);       // rad
    angular_vel_stddev_ = declare_parameter<double>("angular_velocity_stddev", 0.01);  // rad/s
    linear_acc_stddev_ = declare_parameter<double>("linear_acceleration_stddev", 0.1); // m/s^2

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      publish_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "IM10A driver starting on %s @ %d baud -> %s",
                port_.c_str(), baud_, publish_topic_.c_str());
    running_ = true;
    read_thread_ = std::thread(&Im10aDriverNode::readLoop, this);
  }

  ~Im10aDriverNode() override
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
      im10a::SerialPort serial;
      try
      {
        serial.open(port_, baud_);
        RCLCPP_INFO(get_logger(), "Connected to IM10A serial port.");
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(get_logger(), "%s. Retrying in 1s.", e.what());
        std::this_thread::sleep_for(1s);
        continue;
      }

      im10a::WitmotionParser parser;
      std::array<uint8_t, 256> chunk;
      bool logged_first = false;

      while (running_ && rclcpp::ok() && serial.isOpen())
      {
        ssize_t n = serial.readBytes(chunk.data(), chunk.size());
        if (n <= 0)
        {
          continue;  // timeout or transient error
        }
        for (ssize_t i = 0; i < n; ++i)
        {
          im10a::ImuSample sample;
          if (parser.pushByte(chunk[i], sample))
          {
            publishImu(sample);
            if (!logged_first)
            {
              RCLCPP_INFO(get_logger(),
                          "IM10A streaming (orientation source: %s).",
                          parser.hasQuaternion() ? "quaternion 0x59" : "Euler 0x53");
              logged_first = true;
            }
          }
        }
      }
    }
  }

  void publishImu(const im10a::ImuSample & s)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    msg.orientation.w = s.qw;
    msg.orientation.x = s.qx;
    msg.orientation.y = s.qy;
    msg.orientation.z = s.qz;

    msg.angular_velocity.x = s.gx;
    msg.angular_velocity.y = s.gy;
    msg.angular_velocity.z = s.gz;

    msg.linear_acceleration.x = s.ax;
    msg.linear_acceleration.y = s.ay;
    msg.linear_acceleration.z = s.az;

    const double ovar = orientation_stddev_ * orientation_stddev_;
    const double gvar = angular_vel_stddev_ * angular_vel_stddev_;
    const double avar = linear_acc_stddev_ * linear_acc_stddev_;
    // Diagonal covariance; a leading -1 would mark the field unavailable.
    msg.orientation_covariance = {ovar, 0, 0, 0, ovar, 0, 0, 0, ovar};
    msg.angular_velocity_covariance = {gvar, 0, 0, 0, gvar, 0, 0, 0, gvar};
    msg.linear_acceleration_covariance = {avar, 0, 0, 0, avar, 0, 0, 0, avar};
    if (!s.has_orientation)
    {
      msg.orientation_covariance[0] = -1.0;  // orientation not available
    }

    imu_pub_->publish(msg);
  }

  std::string port_;
  int baud_ = 115200;
  std::string frame_id_;
  std::string publish_topic_;
  double orientation_stddev_ = 0.02;
  double angular_vel_stddev_ = 0.01;
  double linear_acc_stddev_ = 0.1;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::atomic<bool> running_{false};
  std::thread read_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Im10aDriverNode>());
  rclcpp::shutdown();
  return 0;
}
