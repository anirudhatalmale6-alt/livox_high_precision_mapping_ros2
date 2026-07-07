// imu_gnss_adapter — bridges the Hiwonder IM10A IMU into the mapping pipeline.
//
// The IM10A ships with an official Hiwonder ROS2 driver that publishes a
// standard sensor_msgs/Imu (default topic "/imu/data"). Rather than reimplement
// the vendor's serial protocol, this adapter subscribes to that topic and
// republishes it as "/gnss_inertial/imu" — the exact contract the mapping node
// expects (previously produced by the APX-15 driver).
//
// It also optionally replaces the IMU's yaw with the UM982's dual-antenna GNSS
// heading, which is drift-free and absolute (unlike a MEMS magnetometer yaw).
// This is the single biggest accuracy win available from this sensor combo.
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class ImuGnssAdapter : public rclcpp::Node
{
public:
  ImuGnssAdapter()
  : Node("imu_gnss_adapter")
  {
    input_topic_ = declare_parameter<std::string>("input_imu_topic", "/imu/data");
    output_frame_ = declare_parameter<std::string>("output_frame_id", "imu");
    use_gnss_heading_ = declare_parameter<bool>("use_gnss_heading", false);
    heading_topic_ = declare_parameter<std::string>(
      "gnss_heading_topic", "/um982_driver/heading");
    // Mounting yaw offset (deg) between the GNSS antenna baseline and the IMU
    // x-axis, added to the GNSS heading before it replaces the IMU yaw.
    heading_offset_deg_ = declare_parameter<double>("heading_offset_deg", 0.0);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/gnss_inertial/imu", rclcpp::SensorDataQoS());

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuGnssAdapter::imuCbk, this, std::placeholders::_1));

    if (use_gnss_heading_)
    {
      heading_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
        heading_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ImuGnssAdapter::headingCbk, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "GNSS heading fusion ENABLED from %s (offset %.2f deg).",
                  heading_topic_.c_str(), heading_offset_deg_);
    }

    RCLCPP_INFO(get_logger(), "Adapting %s -> /gnss_inertial/imu", input_topic_.c_str());
  }

private:
  void headingCbk(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    // Recover yaw from the heading quaternion (rotation about +Z).
    tf2::Quaternion q(msg->quaternion.x, msg->quaternion.y,
                      msg->quaternion.z, msg->quaternion.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    gnss_yaw_ = yaw + heading_offset_deg_ * M_PI / 180.0;
    have_gnss_yaw_ = true;
  }

  void imuCbk(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu out = *msg;
    out.header.frame_id = output_frame_;

    if (use_gnss_heading_ && have_gnss_yaw_)
    {
      // Keep the IMU's roll/pitch (well observed by accel/gyro) and substitute
      // the absolute GNSS yaw.
      tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                        msg->orientation.z, msg->orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      tf2::Quaternion fused;
      fused.setRPY(roll, pitch, gnss_yaw_);
      fused.normalize();
      out.orientation.x = fused.x();
      out.orientation.y = fused.y();
      out.orientation.z = fused.z();
      out.orientation.w = fused.w();
    }

    imu_pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_frame_;
  std::string heading_topic_;
  bool use_gnss_heading_ = false;
  double heading_offset_deg_ = 0.0;

  bool have_gnss_yaw_ = false;
  double gnss_yaw_ = 0.0;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuGnssAdapter>());
  rclcpp::shutdown();
  return 0;
}
