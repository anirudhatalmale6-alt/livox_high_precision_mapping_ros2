// Extracts an RTCM3 correction stream from a MAVLink byte stream by decoding
// GPS_RTCM_DATA (msg id 233) messages and reassembling their fragments.
//
// This is the natural fit when your RTK base already feeds a Pixhawk/ArduPilot
// flight controller: the ground station injects the base corrections into the
// autopilot as GPS_RTCM_DATA messages over the (SiK-radio) MAVLink link, and the
// autopilot forwards those broadcast messages on to any companion link. A Pi
// tapping that MAVLink stream (a serial link to the flight controller, or its
// own telemetry radio) can pull the exact same corrections back out and feed
// them to the UM982 — no separate correction radio for the mapping receiver.
//
// Supports MAVLink v1 (0xFE) and v2 (0xFD, incl. payload truncation + optional
// signature). Every candidate frame's CRC is validated (MAVLink X25 /
// CRC-16-MCRF4XX with the message CRC_EXTRA) so only genuine GPS_RTCM_DATA
// frames are accepted — this also makes the parser self-resynchronising after
// garbage or a mid-frame start. Header-only, libc only.
#ifndef UM982_DRIVER__MAVLINK_RTCM_HPP_
#define UM982_DRIVER__MAVLINK_RTCM_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

namespace um982
{

class MavlinkRtcm
{
public:
  using RtcmSink = std::function<void(const unsigned char *, size_t)>;

  explicit MavlinkRtcm(RtcmSink sink) : sink_(std::move(sink)) {}

  // Feed raw bytes read from the MAVLink serial link. Complete, reassembled
  // RTCM3 chunks are delivered to the sink as they are decoded.
  void feed(const unsigned char * data, size_t len)
  {
    stream_.insert(stream_.end(), data, data + len);
    scan();
  }

  // MAVLink checksum accumulator (CRC-16/MCRF4XX, aka the X25 variant MAVLink
  // uses). Exposed static so it can be unit-tested against the known answer for
  // the string "123456789" (0x6F91).
  static void crcAccumulate(uint8_t b, uint16_t & crc)
  {
    uint8_t tmp = b ^ static_cast<uint8_t>(crc & 0xFF);
    tmp ^= static_cast<uint8_t>(tmp << 4);
    crc = static_cast<uint16_t>(
      (crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^
      (static_cast<uint16_t>(tmp) << 3) ^ (tmp >> 4));
  }

private:
  static constexpr uint8_t STX_V1 = 0xFE;
  static constexpr uint8_t STX_V2 = 0xFD;
  static constexpr uint32_t GPS_RTCM_DATA_ID = 233;
  static constexpr uint8_t GPS_RTCM_DATA_CRC_EXTRA = 35;
  static constexpr size_t MAX_FRAME = 300;  // GPS_RTCM_DATA fits well under this

  enum class Res { Good, Bad, NeedMore };

  // Walk the buffer looking for valid GPS_RTCM_DATA frames. A frame is only
  // consumed if it is msg 233 with a matching CRC; anything else advances the
  // read position by one byte so we resynchronise on the next start byte.
  void scan()
  {
    size_t i = 0;
    size_t retain_from = 0;      // erase everything before this when done
    bool pending = false;        // saw an incomplete candidate we must keep

    while (i < stream_.size())
    {
      const uint8_t c = stream_[i];
      if (c != STX_V1 && c != STX_V2)
      {
        ++i;
        continue;
      }
      size_t frame_len = 0;
      const Res r = tryParse(i, frame_len);
      if (r == Res::Good)
      {
        i += frame_len;
        retain_from = i;         // fully resolved up to here
        pending = false;
      }
      else if (r == Res::NeedMore)
      {
        if (!pending)
        {
          pending = true;
          retain_from = i;       // keep from the first unresolved start byte
        }
        ++i;                     // keep scanning later start bytes meanwhile
      }
      else  // Res::Bad
      {
        ++i;
        if (!pending)
        {
          retain_from = i;       // junk byte, safe to drop
        }
      }
    }

    if (retain_from > stream_.size())
    {
      retain_from = stream_.size();
    }
    stream_.erase(stream_.begin(), stream_.begin() + retain_from);

    // Hard cap so a stuck false start byte can never grow the buffer forever.
    if (stream_.size() > 4 * MAX_FRAME)
    {
      stream_.erase(stream_.begin(),
                    stream_.begin() + (stream_.size() - 4 * MAX_FRAME));
    }
  }

  // Try to parse a frame beginning at start (a start byte). On Res::Good the
  // frame is GPS_RTCM_DATA with a valid CRC and its payload has been dispatched;
  // frame_len returns its total length including the start byte.
  Res tryParse(size_t start, size_t & frame_len)
  {
    const bool v2 = (stream_[start] == STX_V2);
    const size_t avail = stream_.size() - start;
    const size_t min_hdr = v2 ? 3 : 2;  // need up to the length (+incompat) byte
    if (avail < min_hdr)
    {
      return Res::NeedMore;
    }

    const size_t payload_len = stream_[start + 1];
    const size_t hdr = v2 ? 10 : 6;     // header bytes incl. the start byte
    size_t sig = 0;
    if (v2)
    {
      const uint8_t incompat = stream_[start + 2];
      if ((incompat & ~0x01) != 0)
      {
        return Res::Bad;                // unknown incompat flag → not a real v2 frame
      }
      sig = (incompat & 0x01) ? 13 : 0;
    }
    frame_len = hdr + payload_len + 2 + sig;
    if (frame_len > MAX_FRAME)
    {
      return Res::Bad;
    }
    if (avail < frame_len)
    {
      return Res::NeedMore;
    }

    uint32_t msgid;
    if (v2)
    {
      msgid = static_cast<uint32_t>(stream_[start + 7]) |
              (static_cast<uint32_t>(stream_[start + 8]) << 8) |
              (static_cast<uint32_t>(stream_[start + 9]) << 16);
    }
    else
    {
      msgid = stream_[start + 5];
    }
    if (msgid != GPS_RTCM_DATA_ID)
    {
      return Res::Bad;
    }

    // CRC over [len .. end of payload] + CRC_EXTRA.
    uint16_t crc = 0xFFFF;
    for (size_t k = start + 1; k < start + hdr + payload_len; ++k)
    {
      crcAccumulate(stream_[k], crc);
    }
    crcAccumulate(GPS_RTCM_DATA_CRC_EXTRA, crc);
    const size_t crc_at = start + hdr + payload_len;
    const uint16_t rx_crc = static_cast<uint16_t>(stream_[crc_at]) |
                            (static_cast<uint16_t>(stream_[crc_at + 1]) << 8);
    if (crc != rx_crc)
    {
      return Res::Bad;
    }

    // GPS_RTCM_DATA payload: flags(u8), len(u8), data[180]. MAVLink v2 may have
    // truncated trailing zero bytes, so reconstruct defensively.
    const unsigned char * p = stream_.data() + start + hdr;
    const uint8_t flags = (payload_len >= 1) ? p[0] : 0;
    uint8_t data_len = (payload_len >= 2) ? p[1] : 0;
    if (data_len > 180) { data_len = 180; }

    unsigned char data[180];
    for (size_t k = 0; k < 180; ++k)
    {
      const size_t src = 2 + k;
      data[k] = (src < payload_len) ? p[src] : 0;
    }
    handleRtcm(flags, data_len, data);
    return Res::Good;
  }

  void handleRtcm(uint8_t flags, uint8_t len, const unsigned char * data)
  {
    const bool fragmented = (flags & 0x01) != 0;
    const uint8_t frag_id = (flags >> 1) & 0x03;
    const uint8_t seq_id = (flags >> 3) & 0x1F;

    if (!fragmented)
    {
      if (len > 0) { sink_(data, len); }
      have_asm_ = false;
      asm_.clear();
      return;
    }

    if (frag_id == 0)
    {
      asm_.assign(data, data + len);
      asm_seq_ = seq_id;
      expect_frag_ = 1;
      have_asm_ = true;
    }
    else
    {
      if (!have_asm_ || seq_id != asm_seq_ || frag_id != expect_frag_)
      {
        have_asm_ = false;       // lost/out-of-order fragment — drop the set
        asm_.clear();
        return;
      }
      asm_.insert(asm_.end(), data, data + len);
      ++expect_frag_;
    }

    // Last fragment is the short one; a full 4th fragment also closes the set.
    if (len < 180 || frag_id == 3)
    {
      if (!asm_.empty()) { sink_(asm_.data(), asm_.size()); }
      have_asm_ = false;
      asm_.clear();
    }
  }

  RtcmSink sink_;
  std::vector<unsigned char> stream_;

  std::vector<unsigned char> asm_;
  bool have_asm_ = false;
  uint8_t asm_seq_ = 0;
  uint8_t expect_frag_ = 0;
};

}  // namespace um982

#endif  // UM982_DRIVER__MAVLINK_RTCM_HPP_
