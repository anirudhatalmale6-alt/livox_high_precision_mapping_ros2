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
# With a single serial sensor the rule matches the device's own USB id, so
# /dev/gps survives being moved to another socket. Only when there are two
# identical chips to tell apart does it fall back to pinning the socket - and it
# says so when it does, because that form breaks if a plug is moved.
#
# It reads the real USB id from the device rather than assuming a CH340: a
# replacement cable may carry a different chip, and a hardcoded id would then
# match nothing.
#
# Run it with NOTHING else using the sensors (stop the mapper first:
# `sudo systemctl stop mapper-field`). It needs sudo to write the rule.
#
#   bash scripts/setup_sensor_names.sh
#
set -u

RULES=/etc/udev/rules.d/99-livox-sensors.rules
BAUD=115200

# Show any rule already in place. When /dev/gps has gone missing, the existing
# rule is the evidence for why - almost always a socket it is no longer in.
if [ -f "$RULES" ]; then
  echo "Existing rule ($RULES):"
  sed 's/^/  /' "$RULES"
  echo
fi

echo "Looking for USB serial devices (UM982 GPS, and an external IMU if you use one)..."
PORTS=$(ls /dev/ttyUSB* 2>/dev/null || true)
if [ -z "$PORTS" ]; then
  echo "No /dev/ttyUSB* found. Plug the GPS in and try again."
  echo "(If it used to appear and doesn't now: 'sudo apt remove brltty -y', then replug.)"
  exit 1
fi

# --- pass 1: listen to every port and record what we heard --------------------
# Decide only after looking at ALL of them. Judging each port in isolation is
# what made a silent GPS get labelled as an IMU.
NAMES=""; PATHS=""; NMEA=""; USBIDS=""
NPORTS=0
for p in $PORTS; do
  stty -F "$p" "$BAUD" raw -echo 2>/dev/null || true
  DATA=$(timeout 3 cat "$p" 2>/dev/null | strings | grep -m1 -E 'GNGGA|GNRMC|GPGGA|GNVTG|GPRMC' || true)
  PROPS=$(udevadm info -q property -n "$p" 2>/dev/null)
  IDPATH=$(printf '%s\n' "$PROPS" | sed -n 's/^ID_PATH=//p')
  # Read the chip's real USB id rather than assuming a CH340. A replacement
  # cable can easily carry a CP2102 or FT232 instead, and a rule hardcoded to
  # 1a86:7523 would then match nothing at all.
  VID=$(printf '%s\n' "$PROPS" | sed -n 's/^ID_VENDOR_ID=//p')
  PID=$(printf '%s\n' "$PROPS" | sed -n 's/^ID_MODEL_ID=//p')
  if [ -z "$IDPATH" ]; then
    echo "  $p: could not read USB port path (skipping)"
    continue
  fi
  NPORTS=$((NPORTS + 1))
  NAMES="$NAMES $p"; PATHS="$PATHS $IDPATH"
  USBIDS="$USBIDS ${VID:-none}:${PID:-none}"
  echo "  $p: USB id ${VID:-unknown}:${PID:-unknown}"
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
set -- $USBIDS; U_LIST="$*"

GPS_PATH=""; IMU_PATH=""; GPS_NAME=""; IMU_NAME=""
GPS_USB=""; IMU_USB=""
i=1
for n in $N_LIST; do
  pn=$(echo "$P_LIST" | cut -d' ' -f$i)
  dp=$(echo "$D_LIST" | cut -d' ' -f$i)
  up=$(echo "$U_LIST" | cut -d' ' -f$i)
  if [ "$n" = "yes" ] && [ -z "$GPS_PATH" ]; then
    GPS_PATH="$dp"; GPS_NAME="$pn"; GPS_USB="$up"
  elif [ -z "$IMU_PATH" ]; then
    IMU_PATH="$dp"; IMU_NAME="$pn"; IMU_USB="$up"
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
    *)   GPS_PATH="$IMU_PATH"; GPS_NAME="$IMU_NAME"; GPS_USB="$IMU_USB"
         IMU_PATH=""; IMU_NAME=""; IMU_USB="" ;;
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

# Two ways to pin a name, and which one is used matters a great deal:
#
#   by USB id      - the name follows the device, so it survives being moved to
#                    a different socket. Only safe when there is ONE serial
#                    sensor, since two identical chips would both match.
#   by USB socket  - the only way to tell two identical chips apart, but the
#                    name vanishes the moment the plug is moved.
#
# The normal rig uses the Avia's built-in IMU, so the GPS is the only serial
# device and gets the robust form. This unit lost /dev/gps for exactly this
# reason: a socket-pinned rule stopped matching once the wiring was redone, the
# driver had nothing to open, and the dashboard went to "Not connected".
rule_by_usb_id() {      # $1=vid:pid  $2=symlink name
  printf 'SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="%s", ENV{ID_MODEL_ID}=="%s", SYMLINK+="%s", MODE="0666"\n' \
    "${1%%:*}" "${1##*:}" "$2" >> "$TMP"
}
rule_by_socket() {      # $1=vid:pid  $2=ID_PATH  $3=symlink name
  # A device that reports no USB id must be matched on the socket alone.
  # Writing ID_VENDOR_ID=="none" would produce a rule that matches nothing and
  # fails silently - the exact failure this script exists to end.
  if [ "$1" = "none:none" ]; then
    printf 'SUBSYSTEM=="tty", ENV{ID_PATH}=="%s", SYMLINK+="%s", MODE="0666"\n' \
      "$2" "$3" >> "$TMP"
    return
  fi
  printf 'SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="%s", ENV{ID_MODEL_ID}=="%s", ENV{ID_PATH}=="%s", SYMLINK+="%s", MODE="0666"\n' \
    "${1%%:*}" "${1##*:}" "$2" "$3" >> "$TMP"
}

echo
if [ -n "$IMU_PATH" ]; then
  # Two identical chips - they can only be distinguished by which socket.
  rule_by_socket "$GPS_USB" "$GPS_PATH" gps
  rule_by_socket "$IMU_USB" "$IMU_PATH" imu
  echo "  $GPS_NAME  ->  /dev/gps   (pinned to its USB socket)"
  echo "  $IMU_NAME  ->  /dev/imu   (pinned to its USB socket)"
  echo
  echo "  NOTE: two identical serial chips can only be told apart by which"
  echo "  socket they are in, so moving either plug means re-running this."
elif [ "$GPS_USB" = "none:none" ]; then
  rule_by_socket "$GPS_USB" "$GPS_PATH" gps
  echo "  $GPS_NAME  ->  /dev/gps   (pinned to its USB socket)"
  echo "  (this adapter reports no USB id, so the socket is all there is to"
  echo "   match on - moving the plug means re-running this)"
else
  rule_by_usb_id "$GPS_USB" gps
  echo "  $GPS_NAME  ->  /dev/gps   (USB id $GPS_USB, any socket)"
  echo "  (no external IMU - the Avia's built-in IMU needs no serial port, so"
  echo "   /dev/gps can follow the device instead of the socket)"
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
