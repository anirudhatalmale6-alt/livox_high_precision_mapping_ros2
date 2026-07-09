// Unit tests for the IM10A WitMotion parser.
//
// The byte sequences below are the ACTUAL bytes captured from the client's
// IM10A at 115200 baud (decoded from the field hex dump), so these tests prove
// the parser against real hardware output rather than synthetic data.
#include <gtest/gtest.h>

#include <vector>

#include "im10a_driver/witmotion_parser.hpp"

using namespace im10a;

// Real acceleration frame from the device (Z axis reads ~1 g at rest).
static const uint8_t kAccelFrame[11] =
  {0x55, 0x51, 0xe6, 0xff, 0x12, 0x00, 0x07, 0x08, 0x31, 0x0b, 0xe8};
// Real angular-velocity frame (stationary -> all zero).
static const uint8_t kGyroFrame[11] =
  {0x55, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x0b, 0xe3};
// Real Euler-angle frame.
static const uint8_t kAngleFrame[11] =
  {0x55, 0x53, 0x54, 0x00, 0x82, 0x00, 0xe3, 0x02, 0xff, 0x46, 0xa8};

TEST(WitmotionParser, ChecksumMatchesRealFrames)
{
  EXPECT_TRUE(validChecksum(kAccelFrame));
  EXPECT_TRUE(validChecksum(kGyroFrame));
  EXPECT_TRUE(validChecksum(kAngleFrame));
}

TEST(WitmotionParser, ChecksumRejectsCorruption)
{
  uint8_t bad[11];
  for (int i = 0; i < 11; ++i) { bad[i] = kAccelFrame[i]; }
  bad[4] ^= 0xFF;  // flip a data byte
  EXPECT_FALSE(validChecksum(bad));
}

TEST(WitmotionParser, DecodesGravityOnZ)
{
  ImuSample s;
  ASSERT_EQ(decodeFrame(kAccelFrame, s), kAccel);
  ASSERT_TRUE(s.has_accel);
  // az_raw = 0x0807 = 2055 -> 2055 * 16 * 9.80665 / 32768 ~= 9.84 m/s^2
  EXPECT_NEAR(s.az, 9.84, 0.05);
  EXPECT_NEAR(s.ax, -0.125, 0.02);   // small tilt
  EXPECT_NEAR(s.ay, 0.086, 0.02);
  EXPECT_NEAR(s.temperature, 28.65, 0.1);  // 0x0b31 / 100
}

TEST(WitmotionParser, DecodesZeroGyroWhenStationary)
{
  ImuSample s;
  ASSERT_EQ(decodeFrame(kGyroFrame, s), kGyro);
  ASSERT_TRUE(s.has_gyro);
  EXPECT_NEAR(s.gx, 0.0, 1e-9);
  EXPECT_NEAR(s.gy, 0.0, 1e-9);
  EXPECT_NEAR(s.gz, 0.0, 1e-9);
}

TEST(WitmotionParser, DecodesSmallAngles)
{
  ImuSample s;
  ASSERT_EQ(decodeFrame(kAngleFrame, s), kAngle);
  // roll_raw 0x0054=84 -> 84/32768*180 = 0.461 deg = 0.00805 rad
  EXPECT_NEAR(s.roll, 0.00805, 1e-4);
  EXPECT_NEAR(s.pitch, 0.01246, 1e-4);   // 0x0082=130
  EXPECT_NEAR(s.yaw, 0.07085, 1e-4);     // 0x02e3=739
}

TEST(WitmotionParser, DecodesQuaternion)
{
  // Real quaternion frame; checksum recomputed for the captured payload.
  // q0=0x7fe9, q1=0x007d, q2=0x00d1, q3=0x0488  (unit quaternion, near level)
  uint8_t f[11] = {0x55, 0x59, 0xe9, 0x7f, 0x7d, 0x00, 0xd1, 0x00, 0x88, 0x04, 0x00};
  uint8_t sum = 0;
  for (int i = 0; i < 10; ++i) { sum = static_cast<uint8_t>(sum + f[i]); }
  f[10] = sum;
  ASSERT_TRUE(validChecksum(f));

  ImuSample s;
  ASSERT_EQ(decodeFrame(f, s), kQuaternion);
  ASSERT_TRUE(s.has_orientation);
  EXPECT_NEAR(s.qw, 0.9993, 1e-3);
  double norm = s.qw * s.qw + s.qx * s.qx + s.qy * s.qy + s.qz * s.qz;
  EXPECT_NEAR(norm, 1.0, 1e-2);
}

// Feed accel + gyro + quaternion as a byte stream and confirm the streaming
// parser emits exactly one fused sample (on the accel frame) with all fields.
TEST(WitmotionParser, StreamsAndFusesSample)
{
  uint8_t quat[11] = {0x55, 0x59, 0xe9, 0x7f, 0x7d, 0x00, 0xd1, 0x00, 0x88, 0x04, 0x00};
  uint8_t sum = 0;
  for (int i = 0; i < 10; ++i) { sum = static_cast<uint8_t>(sum + quat[i]); }
  quat[10] = sum;

  std::vector<uint8_t> stream;
  // Order: gyro, quaternion, then accel (accel triggers the emit).
  for (uint8_t b : kGyroFrame) { stream.push_back(b); }
  for (uint8_t b : quat) { stream.push_back(b); }
  for (uint8_t b : kAccelFrame) { stream.push_back(b); }

  WitmotionParser parser;
  int emits = 0;
  ImuSample last;
  for (uint8_t b : stream)
  {
    ImuSample s;
    if (parser.pushByte(b, s)) { last = s; ++emits; }
  }
  EXPECT_EQ(emits, 1);
  EXPECT_TRUE(last.has_accel);
  EXPECT_TRUE(last.has_gyro);
  EXPECT_TRUE(last.has_orientation);
  EXPECT_NEAR(last.az, 9.84, 0.05);
  EXPECT_NEAR(last.qw, 0.9993, 1e-3);
  EXPECT_TRUE(parser.hasQuaternion());
}

// A burst of leading garbage must not desync the parser: it should still find
// and decode the real frame that follows.
TEST(WitmotionParser, ResyncsAfterGarbage)
{
  WitmotionParser parser;
  int emits = 0;
  ImuSample last;
  const uint8_t garbage[] = {0x00, 0x55, 0x12, 0xAB, 0x55, 0xFF, 0x01};
  for (uint8_t b : garbage) { ImuSample s; if (parser.pushByte(b, s)) { ++emits; } }
  for (uint8_t b : kAccelFrame) { ImuSample s; if (parser.pushByte(b, s)) { last = s; ++emits; } }
  EXPECT_EQ(emits, 1);
  EXPECT_NEAR(last.az, 9.84, 0.05);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
