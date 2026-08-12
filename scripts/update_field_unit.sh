#!/usr/bin/env bash
# =============================================================================
# update_field_unit.sh — pull the latest code, rebuild everything, restart.
#
# This is THE one command to run whenever I push a change, or whenever a build
# looks half-finished:
#
#   cd ~/livox_high_precision_mapping_ros2
#   bash scripts/update_field_unit.sh
#
# It stops the unit, updates the repo, rebuilds the whole workspace, refreshes
# the boot service file, and starts the unit again. Nothing you configured
# (NTRIP login, sensor names) is touched.
#
# On a Raspberry Pi it deliberately builds SLOWLY (one package, two compiler
# jobs at a time). A Pi has enough cores to start four heavy C++ compiles at
# once but not enough RAM to finish them — the kernel then kills a compiler
# mid-build, and colcon reports a failure buried in hundreds of lines. The
# result is a workspace that looks built but is missing packages.
# =============================================================================
set -u

GREEN='\033[1;32m'; YELLOW='\033[1;33m'; RED='\033[1;31m'; BLUE='\033[1;34m'; NC='\033[0m'
step() { echo -e "\n${BLUE}==== $* ====${NC}"; }
ok()   { echo -e "${GREEN}  ✓ $*${NC}"; }
warn() { echo -e "${YELLOW}  ! $*${NC}"; }
die()  { echo -e "${RED}  ✗ $*${NC}"; exit 1; }

if [ "$(id -u)" = "0" ]; then
  die "Run as your normal user (not sudo). It asks for sudo when it needs it."
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$REPO_DIR/ws"
PKG_DIR="$WS_DIR/src/mapper_web"

[ -d "$WS_DIR/src" ] || die "workspace not found at $WS_DIR — run this from inside the repo"
[ -d /opt/ros/humble ] || die "ROS2 Humble is not installed — run scripts/setup_minipc.sh first"

# ---- stop the unit so it is not restarting into a half-written install tree --
step "Stopping the field unit while we update"
sudo systemctl stop mapper-field 2>/dev/null || true
ok "Stopped (it comes back at the end)"

# ---- update the code ---------------------------------------------------------
step "Getting the latest code"
git -C "$REPO_DIR" pull --ff-only \
  || die "git pull failed. If you edited files on the Pi, run: git -C $REPO_DIR stash"
git -C "$REPO_DIR" log --oneline -1
ok "Repo up to date"

# The Livox driver lives in its own repo inside src/ — update it too if present.
if [ -d "$WS_DIR/src/livox_ros2_driver/.git" ]; then
  git -C "$WS_DIR/src/livox_ros2_driver" pull --ff-only >/dev/null 2>&1 \
    && ok "Livox driver up to date" || warn "could not update livox_ros2_driver (non-fatal)"
fi

# ---- rebuild -----------------------------------------------------------------
step "Rebuilding the workspace"
# Total RAM in MB. Under ~6 GB (i.e. any Pi) build one package at a time.
RAM_MB=$(awk '/MemTotal/ {print int($2/1024)}' /proc/meminfo 2>/dev/null || echo 8192)
if [ "${RAM_MB:-8192}" -lt 6000 ]; then
  warn "${RAM_MB} MB RAM — building one package at a time so the compiler is not killed"
  warn "This takes a while (20-40 min on a Pi). Leave it running."
  BUILD_ARGS="--parallel-workers 1"
  export MAKEFLAGS="-j2"
else
  BUILD_ARGS=""
fi

BUILD_LOG="$HOME/mapper_build.log"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
( cd "$WS_DIR" && colcon build --symlink-install $BUILD_ARGS ) 2>&1 | tee "$BUILD_LOG"
BUILD_RC=${PIPESTATUS[0]}

if [ "$BUILD_RC" != "0" ]; then
  warn "The build reported errors. Full output saved to $BUILD_LOG"
  if grep -qiE 'signal 9|Killed|virtual memory exhausted|cannot allocate memory' "$BUILD_LOG"; then
    warn "It ran out of memory. Add swap and re-run this script:"
    echo "    sudo fallocate -l 4G /swapfile && sudo chmod 600 /swapfile"
    echo "    sudo mkswap /swapfile && sudo swapon /swapfile"
  fi
fi

# ---- did the pieces we actually need land? -----------------------------------
step "Checking what got built"
MISSING=""
for p in mapper_web livox_hp_mapping_bringup um982_driver im10a_driver \
         imu_gnss_adapter livox_mapping livox_mapping_interfaces; do
  if [ -d "$WS_DIR/install/$p" ]; then
    ok "$p"
  else
    echo -e "${RED}  ✗ $p${NC}"; MISSING="$MISSING $p"
  fi
done
if [ -f "$WS_DIR/install/mapper_web/share/mapper_web/web/index.html" ]; then
  ok "dashboard page installed"
else
  die "the dashboard page is still not installed — send me $BUILD_LOG"
fi
[ -z "$MISSING" ] || warn "not built:$MISSING — the dashboard will still run, but send me $BUILD_LOG"

# ---- refresh the service file + workspace path --------------------------------
step "Refreshing the boot service"
if [ -f "$PKG_DIR/mapper-field.service" ]; then
  sudo cp "$PKG_DIR/mapper-field.service" /etc/systemd/system/ || warn "could not update the service file"
  sudo systemctl daemon-reload
  ok "Service file refreshed"
fi
if sudo test -f /etc/mapper/field.env; then
  sudo sed -i "s|^MAPPER_WS=.*|MAPPER_WS=$WS_DIR|" /etc/mapper/field.env
  ok "Workspace path in field.env points at $WS_DIR"
else
  warn "/etc/mapper/field.env is missing — run: bash scripts/setup_field_unit.sh"
fi

# ---- start it again ----------------------------------------------------------
step "Starting the field unit"
sudo systemctl start mapper-field
sleep 10
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
CODE=$(curl -s -m 5 -o /dev/null -w '%{http_code}' http://127.0.0.1:8080/ 2>/dev/null)
if [ "$CODE" = "200" ]; then
  ok "Dashboard is up — open  http://${IP:-<pi-ip>}:8080"
else
  warn "The dashboard did not answer yet. Give it another 30 s and reload the page."
  warn "If it still does not come up, run:  bash scripts/collect_diag.sh"
  echo
  systemctl status mapper-field --no-pager -l 2>&1 | head -20
fi
