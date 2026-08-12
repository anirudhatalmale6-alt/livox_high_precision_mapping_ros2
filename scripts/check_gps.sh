#!/usr/bin/env bash
# =============================================================================
# check_gps.sh — find out whether the GPS is talking, and at what baud rate.
#
#   bash scripts/check_gps.sh            # auto-pick the port
#   bash scripts/check_gps.sh /dev/ttyUSB0
#
# Two things make a hand-rolled "cat /dev/ttyUSB0" lie to you:
#
#   1. The mapper service holds the port open and reads every byte, so your cat
#      sits there getting nothing while the GPS is talking perfectly well. This
#      script stops the service first and starts it again afterwards.
#   2. cat only ever listens at whatever baud rate the port was last set to.
#      A UM982 at 9600 looks completely silent to a listener at 115200.
#
# So: exclusive access, and every plausible baud rate.
# =============================================================================
set -u

GREEN='\033[1;32m'; YELLOW='\033[1;33m'; RED='\033[1;31m'; BLUE='\033[1;34m'; NC='\033[0m'
ok()   { echo -e "${GREEN}  ✓ $*${NC}"; }
warn() { echo -e "${YELLOW}  ! $*${NC}"; }
bad()  { echo -e "${RED}  ✗ $*${NC}"; }
step() { echo -e "\n${BLUE}==== $* ====${NC}"; }

BAUDS="115200 9600 38400 57600 230400 460800"

# ---- pick a port -------------------------------------------------------------
PORT="${1:-}"
if [ -z "$PORT" ]; then
  if [ -e /dev/gps ]; then
    PORT=/dev/gps
  else
    PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
  fi
fi
if [ -z "$PORT" ] || [ ! -e "$PORT" ]; then
  bad "No serial port found. Is the GPS plugged in?"
  echo "    Check with:  ls -l /dev/ttyUSB*  and  lsusb"
  exit 1
fi

step "Port"
echo "  using: $PORT"
if [ -e /dev/gps ]; then
  ok "/dev/gps exists -> $(readlink -f /dev/gps)"
else
  warn "/dev/gps does not exist yet (run: bash scripts/setup_sensor_names.sh)"
fi

# ---- get exclusive access ----------------------------------------------------
step "Making sure nothing else is holding the port"
WAS_ACTIVE=0
if systemctl is-active --quiet mapper-field 2>/dev/null; then
  WAS_ACTIVE=1
  echo "  mapper-field is running and reading this port - stopping it for the test"
  sudo systemctl stop mapper-field
  sleep 2
fi
# -n so this can never sit waiting for a password; if sudo is not available the
# check is simply skipped rather than reported as "something is holding it".
HOLDER=$(sudo -n fuser "$PORT" 2>/dev/null || true)
if [ -n "$HOLDER" ]; then
  warn "another program still has the port open (PID:$HOLDER)"
  echo "      Stop it, or the readings below will be wrong."
else
  ok "port is free"
fi

restore() {
  if [ "$WAS_ACTIVE" = "1" ]; then
    echo
    echo "Starting the mapper back up..."
    sudo systemctl start mapper-field
  fi
}
trap restore EXIT

# ---- listen at each baud rate ------------------------------------------------
step "Listening for data (3 s at each speed)"
FOUND_BAUD=""
FOUND_NMEA=""
ANY_BYTES=""
for B in $BAUDS; do
  stty -F "$PORT" "$B" raw -echo 2>/dev/null || { warn "$B: port rejected this speed"; continue; }
  RAW=$(timeout 3 cat "$PORT" 2>/dev/null | head -c 2000)
  if [ -z "$RAW" ]; then
    echo "  $B: silent"
    continue
  fi
  ANY_BYTES="$B"
  TEXT=$(printf '%s' "$RAW" | strings | head -6)
  if printf '%s' "$TEXT" | grep -qE 'GNGGA|GPGGA|GNRMC|GPRMC|GNVTG|UNIHEADING'; then
    FOUND_BAUD="$B"; FOUND_NMEA="$TEXT"
    ok "$B: GPS sentences found"
    printf '%s\n' "$TEXT" | sed 's/^/      /'
    break
  else
    echo "  $B: data, but not GPS text (probably the wrong speed):"
    printf '%s\n' "$TEXT" | head -3 | sed 's/^/      /'
  fi
done

# ---- verdict -----------------------------------------------------------------
step "Result"
if [ -n "$FOUND_BAUD" ]; then
  ok "The GPS is alive and talking at $FOUND_BAUD baud."
  FIX=$(printf '%s' "$FOUND_NMEA" | grep -m1 -E 'GNGGA|GPGGA' || true)
  if [ -n "$FIX" ]; then
    # GGA field 6 is fix quality: 0 = no fix, 1 = single point, 4 = RTK fixed,
    # 5 = RTK float. Position stays empty until this is non-zero.
    Q=$(printf '%s' "$FIX" | cut -d, -f7)
    SATS=$(printf '%s' "$FIX" | cut -d, -f8)
    case "${Q:-0}" in
      0|"") warn "Fix quality 0 - no position yet. The antenna needs a clear view of"
            echo "      the sky; a cold start can take 5-15 minutes." ;;
      1)    ok "Fix quality 1 - basic GPS position (${SATS:-?} satellites). RTK not applied yet." ;;
      2)    ok "Fix quality 2 - DGPS (${SATS:-?} satellites)." ;;
      4)    ok "Fix quality 4 - RTK FIXED, centimetre accuracy (${SATS:-?} satellites). Perfect." ;;
      5)    ok "Fix quality 5 - RTK float (${SATS:-?} satellites), converging toward 4." ;;
      *)    echo "      Fix quality $Q, ${SATS:-?} satellites." ;;
    esac
  fi
  if [ "$FOUND_BAUD" != "115200" ]; then
    echo
    warn "NOTE: the driver is set to 115200 but your GPS is at $FOUND_BAUD."
    echo "      Send me this and I'll change the setting."
  fi
elif [ -n "$ANY_BYTES" ]; then
  warn "Something is on the port at $ANY_BYTES baud, but it is not sending GPS text."
  echo "      That device may not be the GPS. Send me the output above."
else
  bad "Completely silent at every speed - the GPS is not sending anything."
  echo
  echo "  That points at power or wiring rather than software. Worth checking:"
  echo "    - is the UM982 powered (any LED on the board)?"
  echo "    - is the USB-serial adapter wired to the UM982's TX pin,"
  echo "      and are the grounds connected together?"
  echo "    - try the other USB socket / a different cable"
  echo
  echo "  For reference, this is the adapter the Pi can see:"
  ADAPTERS=$(lsusb 2>/dev/null | grep -iE "1a86|CH340|CP210|FTDI" || true)
  if [ -n "$ADAPTERS" ]; then
    printf '%s\n' "$ADAPTERS" | sed 's/^/      /'
  else
    echo "      (no USB-serial adapter visible at all - check the cable)"
  fi
fi
