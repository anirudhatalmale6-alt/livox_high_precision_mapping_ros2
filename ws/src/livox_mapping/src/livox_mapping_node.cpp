// livox_mapping_node — ROS2 Humble port of the original ROS1 livox_mapping_case.
//
// It fuses:
//   * /livox/lidar            (sensor_msgs/PointCloud2) from livox_ros_driver2
//   * /gnss_inertial/imu      (sensor_msgs/Imu)   attitude from the IM10A IMU
//   * /gnss_inertial/navsatfix(sensor_msgs/NavSatFix) RTK position from the UM982
//
// For every LiDAR frame it interpolates the pose (SLERP on attitude, linear on
// position), motion-compensates ("deskews") each point to the frame start, and
// accumulates a coloured, georeferenced point cloud that is published on
// /pub_pointcloud2 and optionally written to a .pcd on shutdown.
//
// The georeferencing maths (Mercator projection, per-point distortion
// correction, the rtk2lidar extrinsic transform) are ported 1:1 from the
// original so the mapping output is unchanged; only the ROS layer and the
// sensor sources differ.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>

using PointType = pcl::PointXYZI;

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;

inline double toTime(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

// SLERP between two orientation quaternions (stored in Odometry) at fraction t.
bool averageQuaternion(const nav_msgs::msg::Odometry & start,
                       const nav_msgs::msg::Odometry & end,
                       Eigen::Quaterniond & result, double t)
{
  if (t > 1.0 || t < 0.0)
  {
    return false;
  }
  double s[4] = {start.pose.pose.orientation.w, start.pose.pose.orientation.x,
                 start.pose.pose.orientation.y, start.pose.pose.orientation.z};
  double e[4] = {end.pose.pose.orientation.w, end.pose.pose.orientation.x,
                 end.pose.pose.orientation.y, end.pose.pose.orientation.z};
  double cosa = s[0] * e[0] + s[1] * e[1] + s[2] * e[2] + s[3] * e[3];
  if (cosa < 0.0)
  {
    for (double & v : e) { v = -v; }
    cosa = -cosa;
  }
  double k0, k1;
  if (cosa > 0.9995)
  {
    k0 = 1.0 - t;
    k1 = t;
  }
  else
  {
    double sina = std::sqrt(1.0 - cosa * cosa);
    double a = std::atan2(sina, cosa);
    k0 = std::sin((1.0 - t) * a) / sina;
    k1 = std::sin(t * a) / sina;
  }
  result.w() = s[0] * k0 + e[0] * k1;
  result.x() = s[1] * k0 + e[1] * k1;
  result.y() = s[2] * k0 + e[2] * k1;
  result.z() = s[3] * k0 + e[3] * k1;
  return true;
}

// Gauss-Kruger / Mercator projection used to map lat/lon to a local metric
// plane. Ported verbatim from the original project.
bool mercatorProj(double B0, double L0, double B, double L, double & X, double & Y)
{
  static double _A = 6378137, _B = 6356752.3142, _B0 = B0, _L0 = L0;
  static double e = std::sqrt(1 - (_B / _A) * (_B / _A));
  static double e_ = std::sqrt((_A / _B) * (_A / _B) - 1);
  static double NB0 = ((_A * _A) / _B) / std::sqrt(1 + e_ * e_ * std::cos(_B0) * std::cos(_B0));
  static double K = NB0 * std::cos(_B0);

  if (L < -M_PI || L > M_PI || B < -M_PI_2 || B > M_PI_2)
  {
    return false;
  }
  Y = K * (L - _L0);
  X = K * std::log(std::tan(M_PI_4 + B / 2) *
                   std::pow((1 - e * std::sin(B)) / (1 + e * std::sin(B)), e / 2));
  return true;
}

// Colourise a point by reflectivity (blue->green->red ramp), ported verbatim.
void rgbTrans(const PointType & pi, pcl::PointXYZRGB & po)
{
  po.x = pi.x;
  po.y = pi.y;
  po.z = pi.z;
  int r = static_cast<int>(pi.intensity);
  if (r < 30)
  {
    po.r = 0;  po.g = (r * 255 / 30) & 0xff;  po.b = 0xff;
  }
  else if (r < 90)
  {
    po.r = 0;  po.g = 0xff;  po.b = (((90 - r) * 255) / 60) & 0xff;
  }
  else if (r < 150)
  {
    po.r = ((r - 90) * 255 / 60) & 0xff;  po.g = 0xff;  po.b = 0;
  }
  else
  {
    po.r = 0xff;  po.g = (((255 - r) * 255) / (255 - 150)) & 0xff;  po.b = 0;
  }
}
}  // namespace

class LivoxMappingNode : public rclcpp::Node
{
public:
  LivoxMappingNode()
  : Node("livox_mapping")
  {
    lidar_delta_time_ = declare_parameter<double>("lidar_delta_time", 0.01);
    map_file_path_ = declare_parameter<std::string>("map_file_path", "");
    save_pcd_ = declare_parameter<bool>("save_pcd", true);
    // Periodically flush the map to disk while running, so a file always exists
    // even if the node is killed hard (e.g. Ctrl-C during heavy load, where the
    // process may be SIGKILLed before the on-exit save can run). Every run
    // writes to ONE timestamped file, rewritten in place. 0 disables.
    autosave_sec_ = declare_parameter<double>("autosave_sec", 15.0);
    last_save_ = std::chrono::steady_clock::now();

    // rtk2lidar extrinsic (row-major 4x4): transform from the IMU/GNSS body
    // frame to the LiDAR frame.
    //
    // Default is IDENTITY, which is correct for the Livox Avia's BUILT-IN IMU:
    // that IMU shares the LiDAR body and Livox aligns its axes with the point-
    // cloud frame, so no rotation is needed. (The small few-cm translation
    // offset between the Avia IMU and the optical centre is negligible at
    // walking scale; add it here if you ever need survey-exact lever-arm.)
    //
    // If you instead use a SEPARATE, externally mounted IMU (e.g. the IM10A)
    // rotated 180 deg in yaw relative to the LiDAR, override rtk2lidar with:
    //   [-1, 0, 0, 0,  0, -1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1]
    // Symptom of a wrong extrinsic: the whole map comes out rotated/flipped.
    std::vector<double> default_ext = {
       1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1};
    auto ext = declare_parameter<std::vector<double>>("rtk2lidar", default_ext);
    if (ext.size() != 16)
    {
      RCLCPP_WARN(get_logger(), "rtk2lidar must have 16 elements; using default.");
      ext = default_ext;
    }
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        rtk2lidar_(i, j) = ext[i * 4 + j];
      }
    }

    accumulated_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // The UM982 and IM10A drivers publish sensor streams with BEST_EFFORT
    // reliability (rclcpp::SensorDataQoS). A subscription must use a compatible
    // (BEST_EFFORT) reliability or ROS2 silently drops every message. We keep a
    // deep KeepLast history so the time-alignment state machine still has plenty
    // of samples buffered. The LiDAR driver publishes RELIABLE, so its
    // subscription stays RELIABLE to match.
    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(20000)).best_effort();
    auto gnss_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort();
    sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", rclcpp::QoS(1000),
      std::bind(&LivoxMappingNode::lidarCbk, this, std::placeholders::_1));
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/gnss_inertial/imu", imu_qos,
      std::bind(&LivoxMappingNode::imuCbk, this, std::placeholders::_1));
    sub_rtk_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gnss_inertial/navsatfix", gnss_qos,
      std::bind(&LivoxMappingNode::rtkCbk, this, std::placeholders::_1));

    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("pub_pointcloud2", 1);
    pub_odometry_ = create_publisher<nav_msgs::msg::Odometry>("pub_odometry", 1);

    // Drive the time-alignment state machine off a timer (the ROS1 version ran
    // it in a busy while-loop in main()).
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&LivoxMappingNode::processPending, this));

    // Diagnostics heartbeat: until a map is actually building, say plainly what
    // is (not) arriving, so a silent "no file" is impossible to misread. The
    // mapper needs ALL THREE streams (LiDAR + IMU + GNSS) to fuse a single point.
    diag_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&LivoxMappingNode::diagnostics, this));

    RCLCPP_INFO(get_logger(), "livox_mapping node ready (lidar_delta_time=%.4f).",
                lidar_delta_time_);
  }

  // Build a filename stamped with the local date/time, e.g.
  // livox_map_2026-07-17_10-29-48.pcd, so each run is saved separately and
  // never overwrites the previous one.
  std::string timestampedFilename()
  {
    std::time_t t = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    char stamp[24];
    std::strftime(stamp, sizeof(stamp), "%Y-%m-%d_%H-%M-%S", &tm_buf);
    return std::string("livox_map_") + stamp + ".pcd";
  }

  // One output file per run, chosen the first time we save (start-of-run stamp)
  // and rewritten in place by every periodic and final save.
  const std::string & sessionFile()
  {
    if (session_file_.empty())
    {
      std::string name = timestampedFilename();
      session_file_ = map_file_path_.empty() ? name : map_file_path_ + "/" + name;
    }
    return session_file_;
  }

  // Write the accumulated cloud to `file`. Writes to a temporary then renames,
  // so the target is only ever replaced by a COMPLETE file — a hard kill mid-
  // write cannot leave a truncated .pcd (the previous complete one survives).
  void saveMap()
  {
    if (!save_pcd_ || accumulated_->empty())
    {
      return;
    }
    const std::string & file = sessionFile();
    const std::string tmp = file + ".tmp";
    pcl::PCDWriter writer;
    if (writer.writeBinary(tmp, *accumulated_) != 0)
    {
      RCLCPP_WARN(get_logger(), "Map save failed to write %s", tmp.c_str());
      return;
    }
    if (std::rename(tmp.c_str(), file.c_str()) != 0)
    {
      RCLCPP_WARN(get_logger(), "Map save failed to finalise %s", file.c_str());
      return;
    }
    writeGeoRef(file);
    RCLCPP_INFO(get_logger(), "Saved %zu points to %s",
                accumulated_->size(), file.c_str());
  }

  // Write a small human-readable sidecar (<file>.geo.txt) recording where the
  // local (0,0,0) sits on the globe. The .pcd itself stores points in a local
  // metric frame (metres from the first RTK fix) - that is the georeferenced
  // map, true to scale and orientation, but PCD carries no lat/lon field, so
  // this file is what lets you place the whole cloud back on the earth.
  void writeGeoRef(const std::string & pcd_file)
  {
    if (!global_ref_set_)
    {
      return;
    }
    const std::string geo = pcd_file + ".geo.txt";
    std::ofstream f(geo);
    if (!f)
    {
      return;
    }
    f << std::fixed << std::setprecision(8)
      << "Georeference for " << pcd_file << "\n"
      << "\n"
      << "The point cloud is in a LOCAL metric frame (units: metres).\n"
      << "Its origin (0,0,0) is the first RTK position of this run:\n"
      << "  latitude  = " << lla0_[1] << " deg\n"
      << "  longitude = " << lla0_[0] << " deg\n"
      << std::setprecision(3)
      << "  altitude  = " << lla0_[2] << " m\n"
      << "\n"
      << "Axes (original livox_high_precision_mapping convention):\n"
      << "  x = local northing (m)\n"
      << "  y = local easting  (m)\n"
      << "  z = vertical (m), positive downward (anchor_alt - point_alt)\n"
      << "\n"
      << "Scale and orientation are already true (RTK position + dual-antenna\n"
      << "heading), so this single anchor places the entire cloud on the globe:\n"
      << "project each point about the origin above with a local-tangent-plane\n"
      << "(ENU) or Mercator transform to recover real-world lat/lon or UTM.\n";
    f.close();
  }

  // Called from the processing path after each frame is appended. Flushes to
  // disk at most once per autosave_sec_, on the same thread, so it is safe
  // without locking (the cloud is only touched by this thread).
  void maybeAutosave()
  {
    if (autosave_sec_ <= 0.0)
    {
      return;
    }
    auto nowt = std::chrono::steady_clock::now();
    double since = std::chrono::duration<double>(nowt - last_save_).count();
    if (since >= autosave_sec_)
    {
      saveMap();
      last_save_ = nowt;
    }
  }

  // Periodic heartbeat while the node runs. Until the map is actually building,
  // it spells out which of the three required streams is missing, so a run that
  // produces no file never looks like a silent failure. Once points are flowing
  // it announces success once and then goes quiet.
  void diagnostics()
  {
    if (!accumulated_->empty())
    {
      if (!announced_flowing_)
      {
        RCLCPP_INFO(get_logger(),
                    "Mapping is building: %zu points so far (LiDAR + IMU + GNSS all flowing).",
                    accumulated_->size());
        announced_flowing_ = true;
      }
      return;
    }

    std::string missing;
    if (lidar_datas_.empty()) { missing += " LiDAR(/livox/lidar)"; }
    if (imu_datas_.empty())   { missing += " IMU(/gnss_inertial/imu)"; }
    if (rtk_datas_.empty())   { missing += " GNSS(/gnss_inertial/navsatfix)"; }

    if (!missing.empty())
    {
      RCLCPP_WARN(get_logger(),
                  "No map yet - not receiving:%s. All three are required; nothing "
                  "will be saved until they arrive. Is the Livox driver running "
                  "(and RTK/heading up)?",
                  missing.c_str());
    }
    else
    {
      RCLCPP_WARN(get_logger(),
                  "Receiving LiDAR(%zu) IMU(%zu) GNSS(%zu) but no points fused yet "
                  "- check their timestamps overlap (1PPS/time sync).",
                  lidar_datas_.size(), imu_datas_.size(), rtk_datas_.size());
    }
  }

  // Final save on shutdown. Unlike the periodic saveMap(), this says out loud
  // when there is nothing to write, so an empty run is never a silent no-file.
  void finalSave()
  {
    if (accumulated_->empty())
    {
      RCLCPP_WARN(get_logger(),
                  "Shutting down with an EMPTY map - no .pcd written because no "
                  "LiDAR+IMU+GNSS data was fused this run. Was the Livox LiDAR "
                  "driver running alongside this launch?");
      return;
    }
    saveMap();
  }

private:
  void lidarCbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!lidar_datas_.empty() &&
        toTime(lidar_datas_.back()->header.stamp) > toTime(msg->header.stamp))
    {
      RCLCPP_WARN(get_logger(), "lidar time went backwards; dropping frame.");
      return;
    }
    lidar_datas_.push_back(msg);
  }

  void imuCbk(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Store only what the mapping maths needs: the orientation quaternion.
    nav_msgs::msg::Odometry o;
    o.header = msg->header;
    o.pose.pose.orientation = msg->orientation;
    imu_datas_.push_back(o);
  }

  void rtkCbk(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    rtk_datas_.push_back(*msg);
  }

  // Time-alignment state machine: advance through LiDAR frames, and for each one
  // find the bracketing IMU and RTK samples. Ported from the original main loop.
  void processPending()
  {
    while (num_lidar_ < lidar_datas_.size())
    {
      bool imu_flag = false;
      bool rtk_flag = false;

      if (num_imu_ < imu_datas_.size())
      {
        if (toTime(imu_datas_[num_imu_ - 1].header.stamp) <=
            toTime(lidar_datas_[num_lidar_]->header.stamp))
        {
          if (toTime(imu_datas_[num_imu_].header.stamp) >=
              toTime(lidar_datas_[num_lidar_]->header.stamp))
          {
            imu_flag = true;
          }
          else
          {
            num_imu_++;
            continue;
          }
        }
        else
        {
          num_lidar_++;
          continue;
        }
      }
      else
      {
        break;  // wait for more imu data
      }

      if (num_rtk_ < rtk_datas_.size())
      {
        if (toTime(rtk_datas_[num_rtk_ - 1].header.stamp) <=
            toTime(lidar_datas_[num_lidar_]->header.stamp))
        {
          if (toTime(rtk_datas_[num_rtk_].header.stamp) >=
              toTime(lidar_datas_[num_lidar_]->header.stamp))
          {
            rtk_flag = true;
          }
          else
          {
            num_rtk_++;
            continue;
          }
        }
        else
        {
          num_lidar_++;
          continue;
        }
      }
      else
      {
        break;  // wait for more rtk data
      }

      if (imu_flag && rtk_flag)
      {
        if (init_flag_)
        {
          process(num_lidar_last_, num_imu_last_ - 1, num_rtk_last_);
        }
        else
        {
          init_flag_ = true;
        }
        num_lidar_last_ = num_lidar_;
        num_imu_last_ = num_imu_;
        num_rtk_last_ = num_rtk_;
        num_lidar_++;
      }
    }
  }

  void process(size_t num_lidar, size_t num_imu, size_t num_rtk)
  {
    pcl::PointCloud<PointType> laserCloudIn;
    pcl::fromROSMsg(*lidar_datas_[num_lidar], laserCloudIn);
    if (laserCloudIn.points.empty())
    {
      return;
    }

    double lidar_t = toTime(lidar_datas_[num_lidar]->header.stamp);
    double imu_back_t = toTime(imu_datas_[num_imu + 1].header.stamp);
    double rtk_back_t = toTime(rtk_datas_[num_rtk].header.stamp);
    double dt = lidar_delta_time_ / laserCloudIn.points.size();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colour(new pcl::PointCloud<pcl::PointXYZRGB>());

    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();

    for (size_t i = 0; i < laserCloudIn.points.size(); ++i)
    {
      if (lidar_t > imu_back_t && num_imu + 2 < imu_datas_.size())
      {
        num_imu++;
        imu_back_t = toTime(imu_datas_[num_imu + 1].header.stamp);
      }
      if (lidar_t > rtk_back_t && num_rtk + 1 < rtk_datas_.size())
      {
        num_rtk++;
        rtk_back_t = toTime(rtk_datas_[num_rtk].header.stamp);
      }

      double t_imu0 = toTime(imu_datas_[num_imu].header.stamp);
      double t_imu1 = toTime(imu_datas_[num_imu + 1].header.stamp);
      double temp_t = lidar_t - t_imu0;
      double span = t_imu1 - t_imu0;
      if (span <= 0.0)
      {
        continue;
      }
      Eigen::Quaterniond q;
      if (!averageQuaternion(imu_datas_[num_imu], imu_datas_[num_imu + 1], q, temp_t / span))
      {
        continue;
      }
      Eigen::Matrix3d rot = q.normalized().toRotationMatrix();

      double t_rtk0 = toTime(rtk_datas_[num_rtk - 1].header.stamp);
      double t_rtk1 = toTime(rtk_datas_[num_rtk].header.stamp);
      double rt = (t_rtk1 - t_rtk0);
      double t1 = lidar_t - t_rtk0;
      double frac = (rt != 0.0) ? (t1 / rt) : 0.0;
      double lla[3], p[3];
      lla[0] = rtk_datas_[num_rtk - 1].longitude * (1 - frac) +
               rtk_datas_[num_rtk].longitude * frac;
      lla[1] = rtk_datas_[num_rtk - 1].latitude * (1 - frac) +
               rtk_datas_[num_rtk].latitude * frac;
      lla[2] = rtk_datas_[num_rtk - 1].altitude * (1 - frac) +
               rtk_datas_[num_rtk].altitude * frac;

      // Global one-time reference: the very first processed point anchors the
      // whole map. This mirrors the original code's function-static LLA0/p0/T1,
      // and is what places every subsequent frame into a single consistent
      // world frame (rather than collapsing each frame onto its own origin).
      if (!global_ref_set_)
      {
        lla0_[0] = lla[0];
        lla0_[1] = lla[1];
        lla0_[2] = lla[2];
      }

      if (!mercatorProj(lla0_[1] * kDeg2Rad, lla0_[0] * kDeg2Rad,
                        lla[1] * kDeg2Rad, lla[0] * kDeg2Rad, p[0], p[1]))
      {
        continue;
      }
      p[2] = lla[2];

      if (!global_ref_set_)
      {
        p0_[0] = p[0];
        p0_[1] = p[1];
        p0_[2] = p[2];
      }
      p[0] = p[0] - p0_[0];
      p[1] = p[1] - p0_[1];
      p[2] = p0_[2] - p[2];

      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T.block<3, 3>(0, 0) = rot;
      T.block<3, 1>(0, 3) = Eigen::Vector3d(p[0], p[1], p[2]);

      if (!global_ref_set_)
      {
        T1_ = T;
        global_ref_set_ = true;
        RCLCPP_INFO(get_logger(),
                    "Map anchor (local 0,0,0) set at first RTK fix: "
                    "lat %.8f  lon %.8f  alt %.3f m. Written to the .geo.txt "
                    "sidecar so the cloud can be georeferenced.",
                    lla0_[1], lla0_[0], lla0_[2]);
      }

      trans = rtk2lidar_ * T1_.inverse() * T * rtk2lidar_.inverse();

      Eigen::Vector4d or_point(laserCloudIn.points[i].x, laserCloudIn.points[i].y,
                               laserCloudIn.points[i].z, 1.0);
      Eigen::Vector4d after = trans * or_point;

      PointType tp;
      tp.x = after[0];
      tp.y = after[1];
      tp.z = after[2];
      tp.intensity = laserCloudIn.points[i].intensity;
      pcl::PointXYZRGB cp;
      rgbTrans(tp, cp);
      colour->push_back(cp);

      lidar_t += dt;
    }

    // Publish odometry + TF of the last computed pose.
    Eigen::Matrix3d R = trans.block<3, 3>(0, 0);
    Eigen::Quaterniond QQ(R);
    nav_msgs::msg::Odometry odo;
    odo.header.frame_id = "camera_init";
    odo.header.stamp = now();
    odo.child_frame_id = "livox";
    odo.pose.pose.orientation.w = QQ.w();
    odo.pose.pose.orientation.x = QQ.x();
    odo.pose.pose.orientation.y = QQ.y();
    odo.pose.pose.orientation.z = QQ.z();
    odo.pose.pose.position.x = trans(0, 3);
    odo.pose.pose.position.y = trans(1, 3);
    odo.pose.pose.position.z = trans(2, 3);
    pub_odometry_->publish(odo);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "camera_init";
    tf.child_frame_id = "aft_mapped";
    tf.transform.translation.x = trans(0, 3);
    tf.transform.translation.y = trans(1, 3);
    tf.transform.translation.z = trans(2, 3);
    tf.transform.rotation.x = QQ.x();
    tf.transform.rotation.y = QQ.y();
    tf.transform.rotation.z = QQ.z();
    tf.transform.rotation.w = QQ.w();
    tf_broadcaster_->sendTransform(tf);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*colour, output);
    output.header.frame_id = "camera_init";
    output.header.stamp = now();
    pub_cloud_->publish(output);

    *accumulated_ += *colour;

    maybeAutosave();
  }

  // parameters
  double lidar_delta_time_ = 0.01;
  std::string map_file_path_;
  bool save_pcd_ = true;
  double autosave_sec_ = 15.0;
  Eigen::Matrix4d rtk2lidar_ = Eigen::Matrix4d::Identity();

  // Autosave bookkeeping (single output file per run, periodic in-place flush).
  std::string session_file_;
  std::chrono::steady_clock::time_point last_save_;

  // buffers
  std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> lidar_datas_;
  std::vector<nav_msgs::msg::Odometry> imu_datas_;
  std::vector<sensor_msgs::msg::NavSatFix> rtk_datas_;

  // alignment indices (start at 1 to mirror the original bracketing logic)
  size_t num_lidar_ = 0, num_imu_ = 1, num_rtk_ = 1;
  size_t num_lidar_last_ = 0, num_imu_last_ = 1, num_rtk_last_ = 1;
  bool init_flag_ = false;

  // Global map anchor (first processed point); persists across all frames.
  bool global_ref_set_ = false;
  Eigen::Matrix4d T1_ = Eigen::Matrix4d::Identity();
  double p0_[3] = {0, 0, 0};
  double lla0_[3] = {0, 0, 0};

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_;
  bool announced_flowing_ = false;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_rtk_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivoxMappingNode>();
  rclcpp::spin(node);
  node->finalSave();
  rclcpp::shutdown();
  return 0;
}
