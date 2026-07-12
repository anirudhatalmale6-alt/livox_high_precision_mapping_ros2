#!/usr/bin/env bash
#
# setup_sensor_names.sh — create stable /dev/gps and /dev/imu names.
#
# The UM982 (GPS) and IM10A (IMU) both use the same USB-serial chip
# (CH340, 1a86:7523), so /dev/ttyUSB0 / ttyUSB1 can swap on every boot and
# cannot be told apart by USB id. This script figures out which port is which
# by *listening* to each one (the GPS talks NMEA text, the IMU talks binary),
# then writes a udev rule pinning each physical USB port to the right name.
#
# Because it keys off the physical USB port path, it works on any machine
# (mini PC, Raspberry Pi, ...) — just run it once on the new machine, with both
# sensors plugged into the ports you intend to keep using.
#
# Run it with NOTHING else using the sensors (stop any running mapping/launch
# first). It needs sudo to write the rule.
#
#   bash scripts/setup_sensor_names.sh
#
set -u

RULES=/etc/udev/rules.d/99-livox-sensors.rules
BAUD=115200

echo "Looking for CH340 serial devices (UM982 GPS + IM10A IMU)..."
PORTS=$(ls /dev/ttyUSB* 2>/dev/null || true)
if [ -z "$PORTS" ]; then
  echo "No /dev/ttyUSB* found. Plug both sensors in and try again."
  echo "(If they used to appear and don't now: 'sudo apt remove brltty -y', then replug.)"
  exit 1
fi

TMP=$(mktemp)
FOUND_GPS=0
FOUND_IMU=0

for p in $PORTS; do
  # Same chip on both, so identify by the DATA on the wire.
  stty -F "$p" "$BAUD" raw -echo 2>/dev/null || true
  DATA=$(timeout 3 cat "$p" 2>/dev/null | strings | grep -m1 -E 'GNGGA|GNRMC|GPGGA|GNVTG|GPRMC' || true)
  IDPATH=$(udevadm info -q property -n "$p" 2>/dev/null | sed -n 's/^ID_PATH=//p')

  if [ -z "$IDPATH" ]; then
    echo "  $p: could not read USB port path (skipping)"
    continue
  fi

  if [ -n "$DATA" ]; then
    ROLE=gps
    FOUND_GPS=1
    echo "  $p -> /dev/gps   (heard NMEA: ${DATA})"
  else
    ROLE=imu
    FOUND_IMU=1
    echo "  $p -> /dev/imu   (no NMEA — binary IMU stream)"
  fi

  printf 'SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="1a86", ENV{ID_MODEL_ID}=="7523", ENV{ID_PATH}=="%s", SYMLINK+="%s", MODE="0666"\n' \
    "$IDPATH" "$ROLE" >> "$TMP"
done

if [ "$FOUND_GPS" -eq 0 ] || [ "$FOUND_IMU" -eq 0 ]; then
  echo
  echo "WARNING: expected to find BOTH a GPS and an IMU, but did not."
  echo "  GPS found: $FOUND_GPS   IMU found: $FOUND_IMU"
  echo "Check both sensors are plugged in and powered, then re-run."
  echo "(Not writing the rule to avoid a half-configured setup.)"
  rm -f "$TMP"
  exit 1
fi

echo
echo "Writing $RULES (needs sudo):"
cat "$TMP"
sudo cp "$TMP" "$RULES"
rm -f "$TMP"

sudo udevadm control --reload-rules
sudo udevadm trigger
sleep 1

echo
echo "Result:"
ls -l /dev/gps /dev/imu 2>&1 || true
echo
echo "If /dev/gps and /dev/imu both point at a ttyUSB above, you're set."
echo "You can now launch the mapping as normal."
