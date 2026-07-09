// ROS-free parser for the Hiwonder IM10A IMU serial protocol.
//
// The IM10A streams the widely-used WitMotion "0x55" binary format: a sequence
// of fixed 11-byte frames
//
//     [0x55][type][d0 d1 d2 d3 d4 d5 d6 d7][checksum]
//
// where checksum = (sum of the first 10 bytes) & 0xFF and each pair of data
// bytes is a little-endian signed int16. The frame types we care about for a
// sensor_msgs/Imu are:
//
//     0x51  acceleration      ax ay az  (+ chip temperature)
//     0x52  angular velocity  gx gy gz
//     0x53  Euler angles      roll pitch yaw   (fallback orientation)
//     0x59  orientation quaternion  q0 q1 q2 q3   (preferred orientation)
//
// This header is deliberately free of ROS / Eigen / PCL so it can be unit-tested
// with a plain g++ invocation against real captured device bytes.
#ifndef IM10A_DRIVER__WITMOTION_PARSER_HPP_
#define IM10A_DRIVER__WITMOTION_PARSER_HPP_

#include <cstdint>
#include <vector>

namespace im10a
{

// WitMotion frame type identifiers (second byte of every frame).
enum FrameType : uint8_t
{
  kTime = 0x50,
  kAccel = 0x51,
  kGyro = 0x52,
  kAngle = 0x53,
  kMagnetic = 0x54,
  kPressure = 0x56,
  kQuaternion = 0x59,
};

// Full-precision scale factors (raw int16 -> SI units).
//   accel range  +/-16 g       -> m/s^2   (g = 9.80665)
//   gyro  range  +/-2000 deg/s  -> rad/s
//   angle range  +/-180 deg     -> rad
//   quaternion                  -> unit scalar
constexpr double kGravity = 9.80665;
constexpr double kAccelScale = 16.0 * kGravity / 32768.0;
constexpr double kGyroScale = 2000.0 / 32768.0 * (3.14159265358979323846 / 180.0);
constexpr double kAngleScale = 180.0 / 32768.0 * (3.14159265358979323846 / 180.0);
constexpr double kQuatScale = 1.0 / 32768.0;

// A snapshot of the latest IMU state. Emitted once per acceleration frame with
// the most recent gyro and orientation attached.
struct ImuSample
{
  bool has_accel = false;
  bool has_gyro = false;
  bool has_orientation = false;

  double ax = 0.0, ay = 0.0, az = 0.0;        // linear acceleration [m/s^2]
  double gx = 0.0, gy = 0.0, gz = 0.0;        // angular velocity   [rad/s]
  double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;  // orientation quaternion

  double roll = 0.0, pitch = 0.0, yaw = 0.0;  // Euler angles [rad] (0x53)
  double temperature = 0.0;                   // chip temperature [deg C]
};

// Little-endian signed 16-bit from two bytes (low, high).
inline int16_t le16(uint8_t lo, uint8_t hi)
{
  return static_cast<int16_t>(static_cast<uint16_t>(lo) |
                              (static_cast<uint16_t>(hi) << 8));
}

// Verify the trailing checksum byte of an 11-byte frame.
bool validChecksum(const uint8_t frame[11]);

// Decode one already-validated 11-byte frame into the running state. Returns the
// frame type. Only accel/gyro/angle/quaternion mutate the Imu-relevant fields.
uint8_t decodeFrame(const uint8_t frame[11], ImuSample & state);

// Streaming, self-synchronising parser. Feed raw bytes one at a time; whenever a
// complete acceleration frame has been assembled (checksum-valid), it fills
// `out` with the current sample and returns true. Bad frames are skipped and the
// stream re-aligns on the next 0x55 header without losing sync.
class WitmotionParser
{
public:
  bool pushByte(uint8_t b, ImuSample & out);

  // Whether a quaternion (0x59) frame has ever been seen. When false, the
  // orientation in emitted samples is derived from the 0x53 Euler angles.
  bool hasQuaternion() const { return has_quaternion_; }

private:
  static constexpr size_t kFrameLen = 11;
  static constexpr size_t kMaxBuffer = 64;  // runaway guard

  std::vector<uint8_t> buf_;
  ImuSample state_;
  bool has_quaternion_ = false;
};

// Build an orientation quaternion from roll/pitch/yaw (rad), ZYX aerospace
// order, matching the WitMotion angle convention. Used only when the device is
// not configured to output the 0x59 quaternion frame.
void quaternionFromEuler(double roll, double pitch, double yaw,
                         double & qw, double & qx, double & qy, double & qz);

}  // namespace im10a

#endif  // IM10A_DRIVER__WITMOTION_PARSER_HPP_
