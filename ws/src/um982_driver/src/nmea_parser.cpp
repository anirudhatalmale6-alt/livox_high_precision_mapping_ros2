#include "um982_driver/nmea_parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace um982
{

namespace
{
// Return the payload of a sentence (between the leading '$'/'#' and the '*'
// checksum) together with the two checksum hex chars. found_star is set false
// when there is no '*'.
std::string stripFraming(const std::string & sentence, bool & found_star,
                         std::string & checksum_hex)
{
  std::string s = sentence;
  // Trim trailing CR/LF/whitespace.
  while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' '))
  {
    s.pop_back();
  }
  size_t start = 0;
  if (!s.empty() && (s.front() == '$' || s.front() == '#'))
  {
    start = 1;
  }
  size_t star = s.find('*', start);
  found_star = (star != std::string::npos);
  checksum_hex.clear();
  if (found_star)
  {
    checksum_hex = s.substr(star + 1);
    return s.substr(start, star - start);
  }
  return s.substr(start);
}

double toDouble(const std::string & f)
{
  if (f.empty())
  {
    return 0.0;
  }
  return std::strtod(f.c_str(), nullptr);
}

int toInt(const std::string & f)
{
  if (f.empty())
  {
    return 0;
  }
  return static_cast<int>(std::strtol(f.c_str(), nullptr, 10));
}
}  // namespace

bool validateChecksum(const std::string & sentence)
{
  bool found_star = false;
  std::string checksum_hex;
  std::string payload = stripFraming(sentence, found_star, checksum_hex);
  if (!found_star || checksum_hex.size() < 2)
  {
    return false;
  }
  unsigned int expected =
    static_cast<unsigned int>(std::strtol(checksum_hex.substr(0, 2).c_str(), nullptr, 16));
  unsigned char actual = 0;
  for (char c : payload)
  {
    actual ^= static_cast<unsigned char>(c);
  }
  return actual == static_cast<unsigned char>(expected);
}

std::vector<std::string> splitFields(const std::string & sentence)
{
  bool found_star = false;
  std::string checksum_hex;
  std::string payload = stripFraming(sentence, found_star, checksum_hex);

  std::vector<std::string> fields;
  std::string field;
  std::istringstream ss(payload);
  // Unicore logs separate the header from the body with ';' - normalise it to a
  // comma so a single tokenizer handles both NMEA and Unicore forms.
  for (char & c : payload)
  {
    if (c == ';')
    {
      c = ',';
    }
  }
  ss.clear();
  ss.str(payload);
  while (std::getline(ss, field, ','))
  {
    fields.push_back(field);
  }
  return fields;
}

std::string messageType(const std::string & sentence)
{
  auto fields = splitFields(sentence);
  if (fields.empty())
  {
    return "";
  }
  std::string id = fields[0];
  std::transform(id.begin(), id.end(), id.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  // Standard NMEA ids are 5 chars: 2-char talker + 3-char type. Collapse the
  // talker id so "GNGGA"/"GPGGA" both map to "GGA".
  if (id.size() == 5 &&
      (id.rfind("GP", 0) == 0 || id.rfind("GN", 0) == 0 || id.rfind("GL", 0) == 0 ||
       id.rfind("GA", 0) == 0 || id.rfind("GB", 0) == 0 || id.rfind("BD", 0) == 0))
  {
    return id.substr(2);
  }
  return id;
}

double nmeaToDecimalDegrees(const std::string & ddmm, const std::string & hemi)
{
  if (ddmm.empty())
  {
    return 0.0;
  }
  double raw = toDouble(ddmm);
  // The integer part of raw/100 is degrees; the remainder is minutes.
  double degrees = std::floor(raw / 100.0);
  double minutes = raw - degrees * 100.0;
  double decimal = degrees + minutes / 60.0;
  if (hemi == "S" || hemi == "W" || hemi == "s" || hemi == "w")
  {
    decimal = -decimal;
  }
  return decimal;
}

GgaFix parseGga(const std::string & sentence)
{
  GgaFix fix;
  if (messageType(sentence) != "GGA")
  {
    return fix;
  }
  if (!validateChecksum(sentence))
  {
    return fix;
  }
  auto f = splitFields(sentence);
  // GGA: id,utc,lat,N/S,lon,E/W,quality,numSat,hdop,alt,M,geoidSep,M,...
  if (f.size() < 10)
  {
    return fix;
  }
  int quality = toInt(f[6]);
  if (quality <= 0)
  {
    // No fix.
    fix.quality = FixQuality::Invalid;
    return fix;
  }
  fix.utc_seconds = toDouble(f[1]);
  fix.latitude_deg = nmeaToDecimalDegrees(f[2], f[3]);
  fix.longitude_deg = nmeaToDecimalDegrees(f[4], f[5]);
  fix.num_satellites = toInt(f[7]);
  fix.hdop = toDouble(f[8]);
  double altitude = toDouble(f[9]);
  double geoid_sep = (f.size() > 11) ? toDouble(f[11]) : 0.0;
  // Ellipsoidal-ish height used by the mapping projection = MSL height + geoid
  // separation. The mapping node only cares about consistent relative altitude.
  fix.altitude_m = altitude + geoid_sep;
  fix.quality = static_cast<FixQuality>(quality);
  fix.valid = true;
  return fix;
}

Heading parseHeading(const std::string & sentence)
{
  Heading h;
  std::string type = messageType(sentence);
  if (!validateChecksum(sentence))
  {
    return h;
  }
  auto f = splitFields(sentence);
  if (type == "HDT")
  {
    // HDT: id,heading,T
    if (f.size() >= 2 && !f[1].empty())
    {
      h.heading_deg = toDouble(f[1]);
      h.valid = true;
    }
    return h;
  }
  if (type == "UNIHEADINGA")
  {
    // #UNIHEADINGA header fields...;sol_status,pos_type,baseline_length,
    //   heading,pitch,...  The body starts after the ';' (normalised to ',').
    // Locate the body by finding the "heading" position relative to the ';'
    // split. The header has 10 comma fields before ';'. After normalisation the
    // body begins at index 10. Body layout:
    //   [10] sol_status  [11] pos_type  [12] length  [13] heading  [14] pitch
    if (f.size() >= 15)
    {
      h.heading_deg = toDouble(f[13]);
      h.pitch_deg = toDouble(f[14]);
      h.has_pitch = true;
      h.valid = true;
    }
    return h;
  }
  return h;
}

}  // namespace um982
