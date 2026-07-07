// NMEA / Unicore sentence parsing for the Unicore UM982 dual-antenna RTK receiver.
//
// This header is intentionally free of any ROS dependency so the parsing logic
// can be unit-tested with a plain g++ build (see tests/test_nmea_parser.cpp).
//
// The UM982 speaks standard NMEA-0183 extended with Unicore proprietary
// sentences. For high precision georeferenced mapping we need:
//   * absolute position  -> GGA  (latitude, longitude, altitude, fix quality)
//   * true heading        -> HDT  (from the dual-antenna solution) and/or the
//                                  Unicore #UNIHEADINGA log.
#ifndef UM982_DRIVER__NMEA_PARSER_HPP_
#define UM982_DRIVER__NMEA_PARSER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace um982
{

// GGA fix-quality field (NMEA position fix indicator).
enum class FixQuality : int
{
  Invalid = 0,
  GpsFix = 1,
  DgpsFix = 2,
  PpsFix = 3,
  RtkFixed = 4,
  RtkFloat = 5,
  Estimated = 6,
  Manual = 7,
  Simulation = 8,
  Unknown = -1
};

struct GgaFix
{
  bool valid = false;          // true if the sentence parsed and had a fix
  double utc_seconds = 0.0;    // seconds of day from the UTC field (hhmmss.ss)
  double latitude_deg = 0.0;   // signed decimal degrees (N +, S -)
  double longitude_deg = 0.0;  // signed decimal degrees (E +, W -)
  double altitude_m = 0.0;     // orthometric height (MSL) + geoid separation
  FixQuality quality = FixQuality::Invalid;
  int num_satellites = 0;
  double hdop = 0.0;
};

struct Heading
{
  bool valid = false;
  double heading_deg = 0.0;    // true heading, degrees clockwise from north
  double pitch_deg = 0.0;      // pitch (only present in #UNIHEADINGA), degrees
  bool has_pitch = false;
};

// Verify the "*HH" NMEA checksum of a full sentence (with or without a leading
// '$' / '#' and with or without a trailing CR/LF). Returns true when the XOR of
// the payload bytes matches the two hex digits after '*'. Sentences without a
// '*' checksum are treated as invalid.
bool validateChecksum(const std::string & sentence);

// Split a raw sentence into comma separated fields. The leading '$'/'#' and the
// trailing "*HH" checksum are stripped. The first element is the message id
// (e.g. "GPGGA", "GNGGA", "GNHDT").
std::vector<std::string> splitFields(const std::string & sentence);

// Returns the message id of a sentence, upper-cased and without the talker-id
// specifics stripped (e.g. "GNGGA" -> "GGA", "GPHDT" -> "HDT"). For Unicore
// logs like "#UNIHEADINGA" it returns "UNIHEADINGA".
std::string messageType(const std::string & sentence);

// Parse a GGA sentence. Returns a GgaFix with valid=false if the sentence is
// not a GGA, fails the checksum, or has no position fix.
GgaFix parseGga(const std::string & sentence);

// Parse a heading sentence. Accepts NMEA "HDT" (heading true) and the Unicore
// "#UNIHEADINGA" log. Returns valid=false otherwise.
Heading parseHeading(const std::string & sentence);

// Convert an NMEA ddmm.mmmm coordinate plus hemisphere ('N'/'S'/'E'/'W') to
// signed decimal degrees.
double nmeaToDecimalDegrees(const std::string & ddmm, const std::string & hemi);

}  // namespace um982

#endif  // UM982_DRIVER__NMEA_PARSER_HPP_
