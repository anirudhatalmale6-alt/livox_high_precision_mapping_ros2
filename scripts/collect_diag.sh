#!/usr/bin/env bash
# =============================================================================
# collect_diag.sh — one command that gathers everything needed to work out why
# the unit is not coming up, and writes it to ~/mapper_diag.txt.
#
# Send me that file (or paste its contents). It contains NO passwords — the
# NTRIP password and username are masked out.
#
# Usage:
#   bash scripts/collect_diag.sh
# =============================================================================
OUT="$HOME/mapper_diag.txt"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$REPO_DIR/ws"

# Ask for sudo NOW, while the terminal is still visible. Everything below is
# redirected into the report file, so a password prompt down there would be
# invisible and the script would just look like it had hung.
echo "This reads the service log and config, so it needs your password once."
sudo -v 2>/dev/null || echo "(no sudo — the report will just skip those parts)"

# Every probe is wrapped so a missing file or command can never stop the report.
sec() { printf '\n===== %s =====\n' "$*"; }
run() { printf '\n$ %s\n' "$*"; eval "$@" 2>&1 | head -n 120 || true; }

{
echo "Livox mapper diagnostics"
echo "generated: $(date)"
echo "user     : $(id -un)   host: $(hostname)"
echo "repo     : $REPO_DIR"

sec "1. Machine"
# Which board this is matters: GPIO libraries are Raspberry-Pi specific and
# quietly do nothing (or throw) on an Orange Pi or other SBC.
BOARD=""
[ -r /proc/device-tree/model ] && BOARD=$(tr -d '\0' < /proc/device-tree/model)
[ -n "$BOARD" ] || BOARD=$(grep -m1 -E '^Hardware' /proc/cpuinfo 2>/dev/null | cut -d: -f2-)
echo "board model:${BOARD:- unknown (not an SBC, or no device-tree)}"
run "uname -a"
run "cat /etc/os-release | head -3"
run "free -h"
run "df -h / /media 2>/dev/null"

sec "2. ROS2 + workspace build"
if [ -d /opt/ros/humble ]; then echo "/opt/ros/humble: PRESENT"; else echo "/opt/ros/humble: MISSING <-- ROS2 is not installed"; fi
run "git -C '$REPO_DIR' log --oneline -3"
run "ls '$WS_DIR/src'"
echo
echo "Built packages in the install tree:"
if [ -d "$WS_DIR/install" ]; then
  ls "$WS_DIR/install" 2>/dev/null
else
  echo "  NO install/ FOLDER <-- the workspace was never built"
fi
echo
for p in mapper_web livox_hp_mapping_bringup um982_driver im10a_driver \
         imu_gnss_adapter livox_mapping livox_mapping_interfaces livox_ros2_driver; do
  if [ -d "$WS_DIR/install/$p" ]; then echo "  OK      $p"; else echo "  MISSING $p"; fi
done
echo
if [ -f "$WS_DIR/install/mapper_web/share/mapper_web/web/index.html" ]; then
  echo "dashboard page installed: YES"
else
  echo "dashboard page installed: NO  <-- this is why the web page is blank/absent"
fi
run "ls -la '$WS_DIR/log/latest_build' 2>/dev/null | head -20"

sec "3. Field config (/etc/mapper/field.env) — secrets masked"
if sudo test -f /etc/mapper/field.env 2>/dev/null; then
  sudo sed -e 's/^\(NTRIP_PASS=\).*/\1********/' \
           -e 's/^\(NTRIP_USER=\).*/\1********/' /etc/mapper/field.env 2>/dev/null
else
  echo "/etc/mapper/field.env: MISSING <-- the boot service cannot start without it"
fi

sec "4. Boot service"
if [ -f /etc/systemd/system/mapper-field.service ]; then
  echo "unit file: installed"
else
  echo "unit file: MISSING <-- setup_field_unit.sh did not finish"
fi
run "systemctl is-enabled mapper-field"
run "systemctl is-active mapper-field"
run "systemctl status mapper-field --no-pager -l"

sec "5. Service log (most recent 150 lines) — the actual error is usually here"
run "journalctl -u mapper-field -n 150 --no-pager"

sec "6. Network + is the dashboard answering?"
run "hostname -I"
run "ss -ltnp 2>/dev/null | grep -E ':8080|State' "
echo
echo "Local fetch of the dashboard:"
code=$(curl -s -m 5 -o /dev/null -w '%{http_code}' http://127.0.0.1:8080/ 2>/dev/null)
if [ "$code" = "200" ]; then
  echo "  http://127.0.0.1:8080/ -> 200 OK (the web server IS running on the Pi)"
else
  echo "  http://127.0.0.1:8080/ -> '${code:-no answer}' (nothing serving on port 8080)"
fi

sec "7. Sensors"
run "ls -l /dev/gps /dev/imu 2>&1"
run "ls -l /dev/ttyUSB* 2>&1"
run "cat /etc/udev/rules.d/99-livox-sensors.rules 2>&1"
run "lsusb"
echo
# The GPS and IMU use the same USB chip, so the only way to tell what is on a
# port is to listen. The GPS talks NMEA text ($GNGGA...); the IMU talks binary.
echo "What is actually talking on each serial port (3 s listen each):"
for p in /dev/ttyUSB*; do
  [ -e "$p" ] || continue
  stty -F "$p" 115200 raw -echo 2>/dev/null
  SAMPLE=$(timeout 3 cat "$p" 2>/dev/null | strings | head -5)
  if echo "$SAMPLE" | grep -qE 'GNGGA|GPGGA|GNRMC|GPRMC|GNVTG'; then
    echo "  $p = GPS (NMEA seen):"
  elif [ -n "$SAMPLE" ]; then
    echo "  $p = data, but no NMEA (likely the IMU, or GPS with no antenna/fix):"
  else
    echo "  $p = SILENT - nothing coming out of this port at 115200"
  fi
  echo "$SAMPLE" | sed 's/^/      /' | head -5
done

echo
echo "brltty (steals CH340 serial ports if installed):"
dpkg -l brltty 2>/dev/null | grep -q '^ii' && echo "  INSTALLED <-- remove it: sudo apt remove brltty -y" || echo "  not installed (good)"

sec "8. GPIO support"
# Importing gpiozero proves nothing - it imports fine on boards it cannot
# drive. Actually try to make a pin object, which is what fails on an Orange Pi.
python3 - <<'PYEOF' 2>&1 | head -6
try:
    import gpiozero
    print('gpiozero imports OK, version', getattr(gpiozero, '__version__', '?'))
except Exception as e:
    print('gpiozero NOT installed:', e)
    raise SystemExit
try:
    d = gpiozero.LED(16); d.close()
    print('GPIO pins usable: YES')
except Exception as e:
    print('GPIO pins usable: NO -', type(e).__name__, e)
    print('  (button + status LED will be off; the web dashboard is unaffected)')
PYEOF

sec "9. LiDAR reachability (Avia is on ethernet)"
run "ip -4 addr show | grep -E 'inet |^[0-9]'"

echo
echo "===== END OF REPORT ====="
} > "$OUT" 2>&1

echo
echo "Diagnostics written to: $OUT"
echo
echo "Send me that file, or paste it into the chat. No passwords are in it."
echo
echo "Quick summary:"
grep -E '<--|dashboard page installed|MISSING |http://127.0.0.1:8080/ ->' "$OUT" | head -20
