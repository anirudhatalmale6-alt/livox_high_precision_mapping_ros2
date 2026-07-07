#!/usr/bin/env bash
# =============================================================================
# One-shot mini-PC setup for the Livox High Precision Mapping (ROS2) project.
#
# It installs ROS2 Humble + every dependency, downloads this project, builds it,
# and (best-effort) sets up the Livox LiDAR ROS2 driver. Designed to be safe to
# run more than once.
#
#   Ubuntu 22.04 LTS only.  Do NOT run as root — run as your normal user; it
#   will ask for your password when it needs sudo.
#
# Usage:
#   1) Download this file (or the whole repo).
#   2) Run:   bash setup_minipc.sh
#   3) When it finishes, follow the "NEXT STEPS" it prints.
# =============================================================================
set -u

# ---- pretty output ----------------------------------------------------------
GREEN='\033[1;32m'; YELLOW='\033[1;33m'; RED='\033[1;31m'; BLUE='\033[1;34m'; NC='\033[0m'
step()  { echo -e "\n${BLUE}==== $* ====${NC}"; }
ok()    { echo -e "${GREEN}  ✓ $*${NC}"; }
warn()  { echo -e "${YELLOW}  ! $*${NC}"; }
die()   { echo -e "${RED}  ✗ $*${NC}"; exit 1; }

REPO_URL="https://github.com/anirudhatalmale6-alt/livox_high_precision_mapping_ros2.git"
REPO_DIR="$HOME/livox_high_precision_mapping_ros2"
WS_DIR="$REPO_DIR/ws"

# ---- sanity checks ----------------------------------------------------------
step "Checking the system"
if [ "$(id -u)" = "0" ]; then
  die "Please run as your normal user (not root / not sudo). It will prompt for sudo when needed."
fi
if ! command -v lsb_release >/dev/null 2>&1; then sudo apt-get update -y && sudo apt-get install -y lsb-release; fi
UBU_VER="$(lsb_release -rs 2>/dev/null || echo unknown)"
UBU_CODENAME="$(lsb_release -cs 2>/dev/null || echo unknown)"
echo "  Ubuntu version : $UBU_VER ($UBU_CODENAME)"
if [ "$UBU_VER" != "22.04" ]; then
  warn "This project targets Ubuntu 22.04 (ROS2 Humble). Detected $UBU_VER."
  warn "It may still work, but 22.04 is strongly recommended."
  read -r -p "  Continue anyway? [y/N] " ans
  case "$ans" in y|Y) ;; *) die "Aborted. Please install Ubuntu 22.04 first." ;; esac
fi
ok "System check done"

# ---- locale -----------------------------------------------------------------
step "Setting up locale (UTF-8)"
sudo apt-get update -y
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
ok "Locale set"

# ---- ROS2 Humble apt repo ---------------------------------------------------
step "Adding the ROS2 apt repository"
sudo apt-get install -y software-properties-common curl gnupg
sudo add-apt-repository universe -y
sudo mkdir -p /usr/share/keyrings
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBU_CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update -y
ok "ROS2 repo added"

# ---- ROS2 Humble + build tools ---------------------------------------------
step "Installing ROS2 Humble (this is the big download — grab a coffee)"
sudo apt-get install -y ros-humble-desktop ros-dev-tools \
     python3-colcon-common-extensions python3-rosdep git build-essential cmake
ok "ROS2 Humble installed"

# ---- project build dependencies --------------------------------------------
step "Installing project dependencies (PCL, Eigen, tf2, ...)"
sudo apt-get install -y \
     ros-humble-pcl-conversions ros-humble-pcl-ros \
     ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
     libpcl-dev libeigen3-dev
ok "Dependencies installed"

# ---- time sync tools (for the UM982 1PPS setup) -----------------------------
step "Installing time-sync tools (gpsd / chrony / pps-tools)"
sudo apt-get install -y gpsd gpsd-clients chrony pps-tools || warn "time-sync tools install had issues (non-fatal)"
ok "Time-sync tools installed (see docs/time_sync.md to configure PPS)"

# ---- rosdep -----------------------------------------------------------------
step "Initialising rosdep"
sudo rosdep init 2>/dev/null || true
rosdep update || warn "rosdep update had issues (non-fatal)"
ok "rosdep ready"

# ---- get the project --------------------------------------------------------
step "Downloading the project"
if [ -d "$REPO_DIR/.git" ]; then
  ok "Repo already present at $REPO_DIR — updating"
  git -C "$REPO_DIR" pull --ff-only || warn "could not fast-forward; leaving as-is"
else
  git clone "$REPO_URL" "$REPO_DIR" || die "git clone failed"
  ok "Cloned to $REPO_DIR"
fi

# ---- build our packages (self-contained; no vendor drivers needed) ----------
step "Building the mapping workspace"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd "$WS_DIR" || die "workspace dir not found: $WS_DIR"
colcon build --symlink-install || die "colcon build failed — send me the output above"
ok "Workspace built successfully"

# ---- best-effort: Livox ROS2 driver (needed at runtime for the Mid-40) ------
step "Setting up the Livox LiDAR ROS2 driver (best-effort)"
LIVOX_OK=1
if [ ! -d "$HOME/Livox-SDK2" ]; then
  git clone https://github.com/Livox-SDK/Livox-SDK2.git "$HOME/Livox-SDK2" || LIVOX_OK=0
fi
if [ "$LIVOX_OK" = "1" ] && [ -d "$HOME/Livox-SDK2" ]; then
  ( mkdir -p "$HOME/Livox-SDK2/build" && cd "$HOME/Livox-SDK2/build" \
    && cmake .. && make -j"$(nproc)" && sudo make install ) || LIVOX_OK=0
fi
if [ "$LIVOX_OK" = "1" ] && [ ! -d "$WS_DIR/src/livox_ros_driver2" ]; then
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
        "$WS_DIR/src/livox_ros_driver2" || LIVOX_OK=0
fi
if [ "$LIVOX_OK" = "1" ] && [ -f "$WS_DIR/src/livox_ros_driver2/build.sh" ]; then
  ( cd "$WS_DIR/src/livox_ros_driver2" && ./build.sh humble ) || LIVOX_OK=0
fi
if [ "$LIVOX_OK" = "1" ]; then
  ok "Livox driver installed"
else
  warn "Livox driver auto-install did not complete. Follow the Livox-SDK2 +"
  warn "livox_ros_driver2 READMEs on GitHub — it does not block our build."
fi

# ---- done -------------------------------------------------------------------
step "ALL DONE"
cat <<EOF

$(echo -e "${GREEN}Setup complete.${NC}")

Every new terminal must load ROS2 + this workspace. Add these two lines to the
end of your ~/.bashrc (once), then open a new terminal:

  source /opt/ros/humble/setup.bash
  source $WS_DIR/install/setup.bash

TIP — run this to add them automatically:
  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
  echo 'source $WS_DIR/install/setup.bash' >> ~/.bashrc

------------------------------------------------------------------- NEXT STEPS
1) Install the Hiwonder IM10A ROS2 driver from the IM10A docs (section
   "3.2 ROS2 Application Routine"). It publishes the IMU on /imu/data.

2) Plug in the UM982 and IM10A (USB) and note their ports:  ls /dev/ttyUSB*

3) Configure PPS time sync (optional but recommended):  see docs/time_sync.md

4) Launch the whole pipeline:
     source $WS_DIR/install/setup.bash
     ros2 launch livox_hp_mapping_bringup mapping_online.launch.py \\
          um982_port:=/dev/ttyUSB0 use_gnss_heading:=true

5) No hardware yet? See the point cloud the pipeline produces with the
   built-in self-test:   see tools/README.md

If anything above printed an error, copy the text and send it to me — I'll sort
it out quickly.
EOF
