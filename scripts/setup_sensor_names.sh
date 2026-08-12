#!/usr/bin/env bash
#
# setup_sensor_names.sh — create stable /dev/gps (and /dev/imu if present).
#
# The UM982 (GPS) and the optional IM10A (IMU) both use the same USB-serial
# chip (CH340, 1a86:7523), so /dev/ttyUSB0 / ttyUSB1 can swap on every boot and
# cannot be told apart by USB id. This script figures out which port is which
# by *listening* to each one (the GPS talks NMEA text, the IMU talks binary),
# then writes a udev rule pinning each physical USB port to the right name.
#
# AN EXTERNAL IMU IS OPTIONAL. The normal setup uses the Livox Avia's own
# built-in IMU, so the GPS is usually the ONLY serial sensor. A single port is
# therefore the expected case, not an error - this script used to refuse to
# write anything unless it found both, which left the unit with no /dev/gps at
# all and the driver retrying forever.
#
# Because it keys off the physical USB port path, it works on any machine
# (mini PC, Raspberry Pi, Orange Pi, ...) — just run it once on the new machine,
# with the sensors plugged into the ports you intend to keep using.
#
# Run it with NOTHING else using the sensors (stop the mapper first:
# `sudo systemctl stop mapper-field`). It needs sudo to write the rule.
#
#   bash scripts/setup_sensor_names.sh
#
set -u

RULES=/etc/udev/rules.d/99-livox-sensors.rules
BAUD=115200

echo "Looking for CH340 serial devices (UM982 GPS, and an external IMU if you use one)..."
PORTS=$(ls /dev/ttyUSB* 2>/dev/null || true)
if [ -z "$PORTS" ]; then
  echo "No /dev/ttyUSB* found. Plug the GPS in and try again."
  echo "(If it used to appear and doesn't now: 'sudo apt remove brltty -y', then replug.)"
  exit 1
fi

# --- pass 1: listen to every port and record what we heard --------------------
# Decide only after looking at ALL of them. Judging each port in isolation is
# what made a silent GPS get labelled as an IMU.
NAMES=""; PATHS=""; NMEA=""
NPORTS=0
for p in $PORTS; do
  stty -F "$p" "$BAUD" raw -echo 2>/dev/null || true
  DATA=$(timeout 3 cat "$p" 2>/dev/null | strings | grep -m1 -E 'GNGGA|GNRMC|GPGGA|GNVTG|GPRMC' || true)
  IDPATH=$(udevadm info -q property -n "$p" 2>/dev/null | sed -n 's/^ID_PATH=//p')
  if [ -z "$IDPATH" ]; then
    echo "  $p: could not read USB port path (skipping)"
    continue
  fi
  NPORTS=$((NPORTS + 1))
  NAMES="$NAMES $p"; PATHS="$PATHS $IDPATH"
  if [ -n "$DATA" ]; then
    NMEA="$NMEA yes"
    echo "  $p: NMEA heard  ->  this is the GPS   ($DATA)"
  else
    NMEA="$NMEA no"
    echo "  $p: no NMEA in 3 s"
  fi
done

[ "$NPORTS" -gt 0 ] || { echo "No usable serial ports found."; exit 1; }

# --- pass 2: assign roles -----------------------------------------------------
set -- $NAMES;  P_LIST="$*"
set -- $PATHS;  D_LIST="$*"
set -- $NMEA;   N_LIST="$*"

GPS_PATH=""; IMU_PATH=""; GPS_NAME=""; IMU_NAME=""
i=1
for n in $N_LIST; do
  pn=$(echo "$P_LIST" | cut -d' ' -f$i)
  dp=$(echo "$D_LIST" | cut -d' ' -f$i)
  if [ "$n" = "yes" ] && [ -z "$GPS_PATH" ]; then
    GPS_PATH="$dp"; GPS_NAME="$pn"
  elif [ -z "$IMU_PATH" ]; then
    IMU_PATH="$dp"; IMU_NAME="$pn"
  fi
  i=$((i + 1))
done

# A silent single port is the GPS, not an IMU. With the Avia's built-in IMU
# there is no second serial sensor, and the UM982 stays quiet until it is
# configured or has sky view - so silence proves nothing.
if [ -z "$GPS_PATH" ] && [ "$NPORTS" -eq 1 ]; then
  echo
  echo "Only one serial device is connected and it did not send NMEA."
  echo "With the Avia's built-in IMU that device can only be the GPS."
  echo "(The UM982 stays silent until it is configured or gets sky view.)"
  printf "Assign %s as /dev/gps? [Y/n] " "$IMU_NAME"
  read -r ans
  case "${ans:-Y}" in
    n|N) echo "Nothing written."; exit 1 ;;
    *)   GPS_PATH="$IMU_PATH"; GPS_NAME="$IMU_NAME"; IMU_PATH=""; IMU_NAME="" ;;
  esac
fi

if [ -z "$GPS_PATH" ]; then
  echo
  echo "Could not identify the GPS: $NPORTS ports, none sending NMEA."
  echo "Stop the mapper first so nothing else is holding the port:"
  echo "    sudo systemctl stop mapper-field"
  echo "then re-run this. (Not writing a rule I would only be guessing at.)"
  exit 1
fi

TMP=$(mktemp)
rule() {
  printf 'SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="1a86", ENV{ID_MODEL_ID}=="7523", ENV{ID_PATH}=="%s", SYMLINK+="%s", MODE="0666"\n' \
    "$1" "$2" >> "$TMP"
}
rule "$GPS_PATH" gps
echo
echo "  $GPS_NAME  ->  /dev/gps"
if [ -n "$IMU_PATH" ]; then
  rule "$IMU_PATH" imu
  echo "  $IMU_NAME  ->  /dev/imu"
else
  echo "  (no external IMU — using the Livox Avia's built-in IMU, which is the"
  echo "   normal setup and needs no serial port)"
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
ls -l /dev/gps 2>&1 || true
[ -n "$IMU_PATH" ] && { ls -l /dev/imu 2>&1 || true; }
echo
if [ -e /dev/gps ]; then
  echo "/dev/gps is set up. Start the unit again:"
  echo "    sudo systemctl restart mapper-field"
else
  echo "/dev/gps did not appear. Unplug the GPS and plug it back in, then check"
  echo "with:  ls -l /dev/gps"
fi
