#!/usr/bin/env bash
# =============================================================================
# Field-unit setup — turns a freshly-built Pi into a "power on and go" mapper.
#
# setup_minipc.sh installs ROS2 and BUILDS everything. This script does the rest:
# the bits that make the unit actually run by itself with no terminal —
#
#   1. GPIO support for the pushbutton + status LED
#   2. stable /dev/gps and /dev/imu names (both sensors use the same USB chip)
#   3. /etc/mapper/field.env  (workspace path + your NTRIP login, root-only)
#   4. mapper-field.service   (LiDAR + RTK + dashboard, started at boot)
#   5. passwordless shutdown/restart for the dashboard buttons
#
# Safe to run more than once — it keeps anything you have already configured.
#
# Usage:
#   bash scripts/setup_field_unit.sh
# =============================================================================
set -u

GREEN='\033[1;32m'; YELLOW='\033[1;33m'; RED='\033[1;31m'; BLUE='\033[1;34m'; NC='\033[0m'
step()  { echo -e "\n${BLUE}==== $* ====${NC}"; }
ok()    { echo -e "${GREEN}  ✓ $*${NC}"; }
warn()  { echo -e "${YELLOW}  ! $*${NC}"; }
die()   { echo -e "${RED}  ✗ $*${NC}"; exit 1; }

if [ "$(id -u)" = "0" ]; then
  die "Run as your normal user (not root / not sudo). It asks for sudo when needed."
fi

# ---- locate the repo + workspace --------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$REPO_DIR/ws"
PKG_DIR="$WS_DIR/src/mapper_web"
RUN_USER="$(id -un)"

step "Checking the workspace"
[ -d "$PKG_DIR" ] || die "mapper_web not found at $PKG_DIR — run this from inside the repo."
echo "  repo      : $REPO_DIR"
echo "  workspace : $WS_DIR"

# The dashboard is served out of the colcon INSTALL tree, not the source tree,
# so a repo that was never built (or built from the wrong folder) shows nothing.
if [ ! -f "$WS_DIR/install/mapper_web/share/mapper_web/web/index.html" ]; then
  warn "mapper_web is not in the install tree yet — building it now"
  ( set +u; cd "$WS_DIR" && source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --packages-select mapper_web ) \
    || die "colcon build of mapper_web failed — send me the output above"
fi
[ -f "$WS_DIR/install/mapper_web/share/mapper_web/web/index.html" ] \
  || die "the dashboard page still isn't installed — send me the build output"
ok "Dashboard files are installed"

# ---- 1. GPIO (button + LED) --------------------------------------------------
step "Installing GPIO support (pushbutton + status LED)"
sudo apt-get install -y python3-gpiozero python3-lgpio >/dev/null 2>&1 \
  || warn "gpiozero install had issues — the web page still works, only the physical button/LED won't"
ok "GPIO support done"

# ---- 2. stable sensor names --------------------------------------------------
step "Stable sensor names (/dev/gps and /dev/imu)"
if [ -e /dev/gps ] && [ -e /dev/imu ]; then
  ok "/dev/gps and /dev/imu already exist"
else
  warn "The GPS and IMU use the SAME USB chip, so they need to be told apart once."
  echo "  Both sensors must be plugged in, powered, and nothing else using them."
  read -r -p "  Do that now? [Y/n] " ans
  case "${ans:-Y}" in
    n|N) warn "Skipped — run 'bash scripts/setup_sensor_names.sh' later" ;;
    *)   bash "$SCRIPT_DIR/setup_sensor_names.sh" \
           || warn "sensor naming did not complete — re-run scripts/setup_sensor_names.sh with both sensors connected" ;;
  esac
fi

# ---- 3. /etc/mapper/field.env ------------------------------------------------
step "Field configuration (/etc/mapper/field.env)"
sudo mkdir -p /etc/mapper
if sudo test -f /etc/mapper/field.env; then
  ok "field.env already exists — keeping your settings (delete it to re-enter them)"
  # The workspace path may differ on a new Pi, so always refresh just that line.
  sudo sed -i "s|^MAPPER_WS=.*|MAPPER_WS=$WS_DIR|" /etc/mapper/field.env
  echo "  MAPPER_WS set to $WS_DIR"
  # Add any settings introduced after this file was first written, so an
  # existing unit picks up new options without losing what is already there.
  for kv in "BUTTON_GPIO=26" "LED_RED=16" "LED_GREEN=20" "LED_BLUE=21"; do
    key="${kv%%=*}"
    if ! sudo grep -q "^${key}=" /etc/mapper/field.env; then
      echo "$kv" | sudo tee -a /etc/mapper/field.env >/dev/null
      echo "  added $key (default — edit it if your board needs CHIP:LINE)"
    fi
  done
else
  echo "  Your RTK correction (NTRIP) login is stored here — root-only, never in git."
  echo "  Press ENTER to skip any of these; you can edit the file later with:"
  echo "    sudo nano /etc/mapper/field.env"
  read -r -p "  NTRIP host       : " NH
  read -r -p "  NTRIP port [2101]: " NP
  read -r -p "  NTRIP mountpoint : " NM
  read -r -p "  NTRIP username   : " NU
  read -r -s -p "  NTRIP password   : " NPW; echo
  TMP_ENV="$(mktemp)"
  cat > "$TMP_ENV" <<EOF
# Field unit configuration, read by mapper-field.service at boot.
# Root-only (chmod 600) — holds your NTRIP login, so it is NOT in git.
MAPPER_WS=$WS_DIR
NTRIP_HOST=${NH:-your.ntrip.host}
NTRIP_PORT=${NP:-2101}
NTRIP_MOUNT=${NM:-YOUR_MOUNTPOINT}
NTRIP_USER=${NU:-your_username}
NTRIP_PASS=${NPW:-your_password}

# Pushbutton + RGB LED pins. A plain number is a Raspberry Pi BCM pin;
# CHIP:LINE (e.g. 0:100) is a kernel GPIO line and works on any board.
# Find yours with:  bash scripts/list_gpio.sh watch
BUTTON_GPIO=26
LED_RED=16
LED_GREEN=20
LED_BLUE=21
EOF
  sudo cp "$TMP_ENV" /etc/mapper/field.env
  rm -f "$TMP_ENV"
  ok "Wrote /etc/mapper/field.env"
fi
sudo chown root:root /etc/mapper/field.env 2>/dev/null || true
sudo chmod 600 /etc/mapper/field.env
ok "Locked down (root-only)"

# ---- 4. boot service ---------------------------------------------------------
step "Installing the boot service (mapper-field)"
sudo cp "$PKG_DIR/mapper-field.service" /etc/systemd/system/ \
  || die "could not install mapper-field.service"
sudo systemctl daemon-reload
sudo systemctl enable mapper-field >/dev/null 2>&1 \
  || die "could not enable mapper-field"
ok "Service installed and enabled at boot"

# ---- 5. passwordless shutdown / restart --------------------------------------
step "Allowing SHUTDOWN / RESTART without a password prompt"
sh "$PKG_DIR/scripts/allow-poweroff.sh" "$RUN_USER" >/dev/null 2>&1 \
  && ok "Done" || warn "could not add the sudo rule (the boot service runs as root, so this is only needed if you launch by hand)"

# ---- start it ----------------------------------------------------------------
step "Starting the field unit"
sudo systemctl restart mapper-field
sleep 8
if systemctl is-active --quiet mapper-field; then
  ok "mapper-field is running"
else
  warn "mapper-field is not running yet. Run this and send me the file it makes:"
  warn "    bash scripts/collect_diag.sh"
fi

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
step "ALL DONE"
cat <<EOF

$(echo -e "${GREEN}The field unit is set up.${NC}")

Open the dashboard from any device on the same network:

    http://${IP:-<pi-ip>}:8080

From now on you just power the Pi on — wait ~30 s and open that page. No
terminals, no SSH.

Useful checks:
    systemctl status mapper-field          # is it running?
    journalctl -u mapper-field -f          # live log
    sudo systemctl restart mapper-field    # restart it

To load the latest version of everything later:
    bash scripts/update_field_unit.sh

If anything is not working, this writes one report file for me:
    bash scripts/collect_diag.sh

If you change the NTRIP login later:
    sudo nano /etc/mapper/field.env
    sudo systemctl restart mapper-field

If anything above printed an error, copy the text and send it to me.
EOF
