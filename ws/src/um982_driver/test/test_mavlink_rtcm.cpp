// Unit tests for the MAVLink GPS_RTCM_DATA extractor.
//
// The MAVLink checksum is first pinned to its independent known-answer
// (CRC-16/MCRF4XX over "123456789" == 0x6F91), then used to build well-formed
// GPS_RTCM_DATA frames whose payloads must round-trip out of the extractor.
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "um982_driver/mavlink_rtcm.hpp"

using um982::MavlinkRtcm;
using Bytes = std::vector<unsigned char>;

namespace
{
uint16_t crcOver(const Bytes & b, uint8_t extra)
{
  uint16_t c = 0xFFFF;
  for (auto x : b) { MavlinkRtcm::crcAccumulate(x, c); }
  MavlinkRtcm::crcAccumulate(extra, c);
  return c;
}

// Build a MAVLink v2 frame carrying GPS_RTCM_DATA (msg id 233).
Bytes v2frame(uint8_t seq, const Bytes & payload)
{
  Bytes core;
  core.push_back(static_cast<unsigned char>(payload.size()));  // len
  core.push_back(0);  // incompat
  core.push_back(0);  // compat
  core.push_back(seq);
  core.push_back(1);  // sysid
  core.push_back(1);  // compid
  core.push_back(233);
  core.push_back(0);
  core.push_back(0);  // msgid (little-endian, 233)
  for (auto x : payload) { core.push_back(x); }
  const uint16_t crc = crcOver(core, 35);
  Bytes f;
  f.push_back(0xFD);
  for (auto x : core) { f.push_back(x); }
  f.push_back(crc & 0xFF);
  f.push_back((crc >> 8) & 0xFF);
  return f;
}

Bytes v1frame(uint8_t seq, const Bytes & payload)
{
  Bytes core;
  core.push_back(static_cast<unsigned char>(payload.size()));
  core.push_back(seq);
  core.push_back(1);
  core.push_back(1);
  core.push_back(233);  // msgid (1 byte)
  for (auto x : payload) { core.push_back(x); }
  const uint16_t crc = crcOver(core, 35);
  Bytes f;
  f.push_back(0xFE);
  for (auto x : core) { f.push_back(x); }
  f.push_back(crc & 0xFF);
  f.push_back((crc >> 8) & 0xFF);
  return f;
}

Bytes rtcmPayload(uint8_t flags, const Bytes & data)
{
  Bytes p;
  p.push_back(flags);
  p.push_back(static_cast<unsigned char>(data.size()));
  for (auto x : data) { p.push_back(x); }
  return p;
}
}  // namespace

TEST(MavlinkRtcm, CrcKnownAnswer)
{
  uint16_t c = 0xFFFF;
  for (const char * p = "123456789"; *p != '\0'; ++p)
  {
    MavlinkRtcm::crcAccumulate(static_cast<uint8_t>(*p), c);
  }
  EXPECT_EQ(c, 0x6F91);
}

TEST(MavlinkRtcm, V2SingleChunk)
{
  Bytes out;
  MavlinkRtcm ex([&](const unsigned char * d, size_t n) { out.assign(d, d + n); });
  const Bytes rtcm = {0xD3, 0x00, 0x13, 0xAA, 0xBB, 0xCC, 0x11, 0x22};
  const Bytes f = v2frame(7, rtcmPayload(0x00, rtcm));
  ex.feed(f.data(), f.size());
  EXPECT_EQ(out, rtcm);
}

TEST(MavlinkRtcm, V2FragmentedReassembly)
{
  Bytes out;
  MavlinkRtcm ex([&](const unsigned char * d, size_t n) { out.assign(d, d + n); });
  Bytes d0(180), d1(50);
  for (size_t i = 0; i < d0.size(); ++i) { d0[i] = static_cast<unsigned char>(i & 0xFF); }
  for (size_t i = 0; i < d1.size(); ++i) { d1[i] = static_cast<unsigned char>(0x80 + i); }
  const uint8_t seq = 5;
  const uint8_t f0 = 0x01 | (0 << 1) | (seq << 3);
  const uint8_t f1 = 0x01 | (1 << 1) | (seq << 3);
  const Bytes fr0 = v2frame(10, rtcmPayload(f0, d0));
  const Bytes fr1 = v2frame(11, rtcmPayload(f1, d1));

  ex.feed(fr0.data(), fr0.size());
  EXPECT_TRUE(out.empty());  // first fragment does not close the set

  ex.feed(fr1.data(), fr1.size());
  Bytes expected = d0;
  expected.insert(expected.end(), d1.begin(), d1.end());
  EXPECT_EQ(out, expected);
}

TEST(MavlinkRtcm, V1SingleChunk)
{
  Bytes out;
  MavlinkRtcm ex([&](const unsigned char * d, size_t n) { out.assign(d, d + n); });
  const Bytes rtcm = {0xD3, 0x01, 0x02, 0x03, 0x04};
  const Bytes f = v1frame(3, rtcmPayload(0x00, rtcm));
  ex.feed(f.data(), f.size());
  EXPECT_EQ(out, rtcm);
}

TEST(MavlinkRtcm, CorruptFrameRejected)
{
  Bytes out;
  MavlinkRtcm ex([&](const unsigned char * d, size_t n) { out.assign(d, d + n); });
  const Bytes rtcm = {0xD3, 0x09, 0x08, 0x07};
  Bytes f = v2frame(1, rtcmPayload(0x00, rtcm));
  f[12] ^= 0xFF;  // flip a payload byte → CRC mismatch
  ex.feed(f.data(), f.size());
  EXPECT_TRUE(out.empty());
}

TEST(MavlinkRtcm, ResyncAfterGarbage)
{
  Bytes out;
  MavlinkRtcm ex([&](const unsigned char * d, size_t n) { out.assign(d, d + n); });
  const Bytes junk = {0x00, 0xFD, 0x01, 0x02, 0x11, 0x22, 0x33, 0xFE, 0xAB};
  ex.feed(junk.data(), junk.size());
  const Bytes rtcm = {0xD3, 0x55, 0x66};
  const Bytes f = v2frame(2, rtcmPayload(0x00, rtcm));
  ex.feed(f.data(), f.size());
  EXPECT_EQ(out, rtcm);
}

TEST(MavlinkRtcm, ByteAtATimeDelivery)
{
  // Same frame but fed one byte at a time — the parser must still assemble it.
  Bytes out;
  MavlinkRtcm ex([&](const unsigned char * d, size_t n) { out.assign(d, d + n); });
  const Bytes rtcm = {0xD3, 0xEE, 0xFF, 0x01, 0x02, 0x03};
  const Bytes f = v2frame(9, rtcmPayload(0x00, rtcm));
  for (unsigned char b : f)
  {
    ex.feed(&b, 1);
  }
  EXPECT_EQ(out, rtcm);
}
