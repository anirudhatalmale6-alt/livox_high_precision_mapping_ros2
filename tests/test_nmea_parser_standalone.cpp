#include "um982_driver/nmea_parser.hpp"
#include <cstdio>
#include <cmath>
using namespace um982;
int failures = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } else { printf("ok: %s\n", #cond);} } while(0)
#define NEAR(a,b,t) (std::fabs((a)-(b)) < (t))
int main(){
  CHECK(validateChecksum("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"));
  CHECK(!validateChecksum("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48"));
  CHECK(messageType("$GNGGA,,,,,,0,,,,,,,,*78") == "GGA");
  CHECK(messageType("$GPHDT,274.07,T*03") == "HDT");
  CHECK(NEAR(nmeaToDecimalDegrees("4807.038","N"), 48.1173, 1e-4));
  CHECK(NEAR(nmeaToDecimalDegrees("01131.000","E"), 11.5166667, 1e-4));
  CHECK(NEAR(nmeaToDecimalDegrees("4807.038","S"), -48.1173, 1e-4));
  GgaFix fix = parseGga("$GPGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,*42");
  CHECK(fix.valid);
  CHECK(fix.quality == FixQuality::RtkFixed);
  CHECK(NEAR(fix.latitude_deg, 48.1173, 1e-4));
  CHECK(NEAR(fix.longitude_deg, 11.51667, 1e-4));
  CHECK(NEAR(fix.altitude_m, 545.4+46.9, 1e-3));
  CHECK(fix.num_satellites == 8);
  GgaFix nofix = parseGga("$GPGGA,123519,,,,,0,00,,,M,,M,,*6B");
  CHECK(!nofix.valid);
  Heading h = parseHeading("$GPHDT,274.07,T*03");
  CHECK(h.valid);
  CHECK(NEAR(h.heading_deg, 274.07, 1e-2));
  printf("\n%s (%d failures)\n", failures==0?"ALL PASS":"FAILURES", failures);
  return failures;
}
