#include "im10a_driver/witmotion_parser.hpp"

#include <cmath>

namespace im10a
{

bool validChecksum(const uint8_t frame[11])
{
  uint8_t sum = 0;
  for (int i = 0; i < 10; ++i)
  {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  return sum == frame[10];
}

uint8_t decodeFrame(const uint8_t frame[11], ImuSample & state)
{
  const uint8_t type = frame[1];
  const uint8_t * d = frame + 2;  // 8 data bytes

  switch (type)
  {
    case kAccel:
      state.ax = le16(d[0], d[1]) * kAccelScale;
      state.ay = le16(d[2], d[3]) * kAccelScale;
      state.az = le16(d[4], d[5]) * kAccelScale;
      state.temperature = le16(d[6], d[7]) / 100.0;
      state.has_accel = true;
      break;

    case kGyro:
      state.gx = le16(d[0], d[1]) * kGyroScale;
      state.gy = le16(d[2], d[3]) * kGyroScale;
      state.gz = le16(d[4], d[5]) * kGyroScale;
      state.has_gyro = true;
      break;

    case kAngle:
      state.roll = le16(d[0], d[1]) * kAngleScale;
      state.pitch = le16(d[2], d[3]) * kAngleScale;
      state.yaw = le16(d[4], d[5]) * kAngleScale;
      break;

    case kQuaternion:
      state.qw = le16(d[0], d[1]) * kQuatScale;
      state.qx = le16(d[2], d[3]) * kQuatScale;
      state.qy = le16(d[4], d[5]) * kQuatScale;
      state.qz = le16(d[6], d[7]) * kQuatScale;
      state.has_orientation = true;
      break;

    default:
      break;  // time / magnetic / pressure — parsed for sync, unused here
  }
  return type;
}

void quaternionFromEuler(double roll, double pitch, double yaw,
                         double & qw, double & qx, double & qy, double & qz)
{
  const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);

  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}

bool WitmotionParser::pushByte(uint8_t b, ImuSample & out)
{
  buf_.push_back(b);

  // Drop leading bytes until the buffer starts with a frame header.
  while (!buf_.empty() && buf_.front() != 0x55)
  {
    buf_.erase(buf_.begin());
  }
  if (buf_.size() > kMaxBuffer)
  {
    buf_.clear();  // desynced garbage — start over
    return false;
  }
  if (buf_.size() < kFrameLen)
  {
    return false;  // need more bytes for a full frame
  }

  uint8_t frame[kFrameLen];
  for (std::size_t i = 0; i < kFrameLen; ++i)
  {
    frame[i] = buf_[i];
  }

  if (!validChecksum(frame))
  {
    // Not a real frame boundary — drop the false header and re-sync.
    buf_.erase(buf_.begin());
    return false;
  }

  buf_.erase(buf_.begin(), buf_.begin() + kFrameLen);
  const uint8_t type = decodeFrame(frame, state_);
  if (type == kQuaternion)
  {
    has_quaternion_ = true;
  }

  // Emit a sample once per acceleration frame, carrying the latest gyro and
  // orientation. If the device never sends a quaternion, derive one from the
  // most recent Euler angles.
  if (type == kAccel)
  {
    out = state_;
    if (!has_quaternion_)
    {
      quaternionFromEuler(state_.roll, state_.pitch, state_.yaw,
                          out.qw, out.qx, out.qy, out.qz);
      out.has_orientation = true;
    }
    return true;
  }
  return false;
}

}  // namespace im10a
