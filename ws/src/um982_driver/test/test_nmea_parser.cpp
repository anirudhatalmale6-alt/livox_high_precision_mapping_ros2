// Unit tests for the UM982 NMEA / Unicore parser.
#include <gtest/gtest.h>

#include "um982_driver/nmea_parser.hpp"

using namespace um982;

TEST(NmeaParser, ChecksumValid)
{
  // Well known valid GGA sentence.
  EXPECT_TRUE(validateChecksum(
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"));
}

TEST(NmeaParser, ChecksumInvalid)
{
  EXPECT_FALSE(validateChecksum(
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48"));
}

TEST(NmeaParser, MessageTypeCollapsesTalkerId)
{
  EXPECT_EQ(messageType("$GNGGA,,,,,,0,,,,,,,,*78"), "GGA");
  EXPECT_EQ(messageType("$GPHDT,274.07,T*03"), "HDT");
}

TEST(NmeaParser, DecimalDegrees)
{
  // 4807.038 N -> 48 + 7.038/60 = 48.1173
  EXPECT_NEAR(nmeaToDecimalDegrees("4807.038", "N"), 48.1173, 1e-4);
  EXPECT_NEAR(nmeaToDecimalDegrees("01131.000", "E"), 11.5166667, 1e-4);
  EXPECT_NEAR(nmeaToDecimalDegrees("4807.038", "S"), -48.1173, 1e-4);
}

TEST(NmeaParser, ParseGga)
{
  GgaFix fix = parseGga(
    "$GPGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,*42");
  ASSERT_TRUE(fix.valid);
  EXPECT_EQ(fix.quality, FixQuality::RtkFixed);
  EXPECT_NEAR(fix.latitude_deg, 48.1173, 1e-4);
  EXPECT_NEAR(fix.longitude_deg, 11.51667, 1e-4);
  EXPECT_NEAR(fix.altitude_m, 545.4 + 46.9, 1e-3);
  EXPECT_EQ(fix.num_satellites, 8);
}

TEST(NmeaParser, ParseGgaNoFix)
{
  GgaFix fix = parseGga("$GPGGA,123519,,,,,0,00,,,M,,M,,*6B");
  EXPECT_FALSE(fix.valid);
}

TEST(NmeaParser, ParseHdt)
{
  Heading h = parseHeading("$GPHDT,274.07,T*03");
  ASSERT_TRUE(h.valid);
  EXPECT_NEAR(h.heading_deg, 274.07, 1e-2);
}

TEST(NmeaParser, CivilToUnixSeconds)
{
  // 1970-01-01T00:00:00Z is 0; 2000-01-01 and a known recent instant.
  EXPECT_NEAR(civilToUnixSeconds(1970, 1, 1, 0.0), 0.0, 1e-6);
  EXPECT_NEAR(civilToUnixSeconds(2000, 1, 1, 0.0), 946684800.0, 1e-6);
  // 2026-07-23 10:00:53 UTC (verified against `date -u`).
  EXPECT_NEAR(civilToUnixSeconds(2026, 7, 23, 10 * 3600 + 53), 1784800853.0, 1e-6);
  // Year boundary rollover.
  EXPECT_NEAR(civilToUnixSeconds(2027, 1, 1, 1.0), 1798761601.0, 1e-6);
}

TEST(NmeaParser, ParseRmc)
{
  RmcTime t = parseRmc(
    "$GPRMC,100053.00,A,4052.0190,N,07337.9730,W,0.03,0.0,230726,,,A*7C");
  ASSERT_TRUE(t.valid);
  EXPECT_EQ(t.year, 2026);
  EXPECT_EQ(t.month, 7);
  EXPECT_EQ(t.day, 23);
  EXPECT_NEAR(t.utc_seconds, 10 * 3600 + 53, 1e-6);       // seconds of day
  EXPECT_NEAR(t.unix_seconds, 1784800853.0, 1e-3);
}

TEST(NmeaParser, ParseRmcRejectsVoidAndNonRmc)
{
  // Status 'V' (void) must not yield a time.
  EXPECT_FALSE(parseRmc("$GNRMC,100053.00,V,,,,,,,230726,,,N*66").valid);
  // A GGA sentence is not an RMC.
  EXPECT_FALSE(parseRmc(
    "$GPGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,*42").valid);
}
