# Livox High Precision Mapping — ROS2 Humble (Ubuntu 22.04)

A ROS2 Humble port and hardware refresh of
[Livox-SDK/livox_high_precision_mapping](https://github.com/Livox-SDK/livox_high_precision_mapping),
which was originally a ROS1 (catkin) project tested only on Ubuntu 16.04.

This version:

- **Builds and runs on Ubuntu 22.04 LTS with ROS2 Humble** (colcon/ament).
- **Replaces the Applanix APX-15** GNSS/INS with a modern component sensor set:
  - **Unicore UM982** — dual-antenna RTK GNSS → centimetre-level absolute
    position + true heading.
  - **Livox Avia built-in IMU** (default) → high-rate attitude (roll/pitch).
    A separate IMU is **not required** — the Avia's own 200 Hz IMU is used.
  - **Hiwonder IM10A** — optional external 10-axis IMU alternative, supported
    via `start_im10a:=true` if you prefer a standalone IMU.
- **Keeps the original mapping maths unchanged** — the Mercator georeferencing,
  per-point motion compensation ("deskew") and the `rtk2lidar` extrinsic
  transform are ported 1:1, so the point-cloud output is equivalent.

## Why this sensor split

The APX-15 was a single survey-grade unit that output fused position **and**
attitude. We reconstruct the same information from two devices:

| Original (APX-15)             | Replacement                                   |
| ----------------------------- | --------------------------------------------- |
| RTK GNSS position (cm)        | **UM982** GGA → `sensor_msgs/NavSatFix`       |
| Attitude (roll/pitch)         | **Avia built-in IMU** `/livox/imu` (or IM10A) |
| Heading (yaw)                 | **UM982** dual-antenna heading                |
| PPS hardware time sync        | UM982 1PPS (see *Time synchronisation*)       |

Position comes entirely from the UM982's RTK solution; the IMU only supplies
tilt (roll/pitch). By default that IMU is the **Avia's own built-in 200 Hz
IMU**, so no extra IMU box is needed. The UM982's dual-antenna heading gives an
absolute, drift-free yaw, which a MEMS magnetometer cannot provide reliably.
An external IM10A is still supported (`start_im10a:=true`) if preferred.

## Architecture / topics

```
 livox_ros_driver2 ──/livox/lidar (PointCloud2)──┐
                                                  │
 Avia built-in IMU ──/livox/imu──► imu_gnss_adapter ──/gnss_inertial/imu──┤
   (or im10a_driver ──/imu/data)     ▲                                   │
 um982_driver ──/um982_driver/heading─┘                                  ▼
     └────────────────/gnss_inertial/navsatfix (NavSatFix)──────────► livox_mapping
                                                                          │
                                              /pub_pointcloud2 (georeferenced cloud)
                                              /pub_odometry, TF camera_init→aft_mapped
                                              all_points.pcd (on shutdown)
```

## Packages

| Package                     | Role                                                        |
| --------------------------- | ----------------------------------------------------------- |
| `livox_mapping`             | The ported mapping node (core algorithm).                   |
| `um982_driver`              | UM982 RTK serial driver (NMEA GGA + heading). Unit-tested.  |
| `im10a_driver`              | IM10A IMU serial driver (WitMotion 0x55 → Imu). Unit-tested.|
| `imu_gnss_adapter`          | IM10A → `/gnss_inertial/imu`, optional GNSS-yaw fusion.     |
| `livox_mapping_interfaces`  | `CustomMsg`/`CustomPoint` messages (ROS2).                  |
| `livox_hp_mapping_bringup`  | Full-pipeline launch files + central params.               |

## Dependencies

Install ROS2 Humble, then the one **external** vendor driver:

- **Livox Avia** — [`livox_ros2_driver`](https://github.com/Livox-SDK/livox_ros2_driver)
  + [`Livox-SDK`](https://github.com/Livox-SDK/Livox-SDK) (v1). The Avia (and
  Mid-40/70, Horizon, Tele-15) use the original Livox-SDK; the newer Mid-360/HAP
  use Livox-SDK2 / `livox_ros_driver2` instead. Full setup, network config and
  broadcast-code steps are in [`docs/lidar_avia.md`](docs/lidar_avia.md).

The **IM10A does not need any vendor driver** — this workspace ships its own
`im10a_driver` that reads the IM10A directly (WitMotion 0x55 protocol, 115200
baud) and publishes `sensor_msgs/Imu` on `/imu/data`. It is validated with
gtest against real captured device bytes.

ROS2 build deps (from apt): `ros-humble-pcl-conversions`, `ros-humble-pcl-ros`,
`ros-humble-tf2-ros`, `libpcl-dev`, `libeigen3-dev`.

## Quick start on a fresh mini PC (one command)

If you're starting from a clean Ubuntu 22.04 install, the setup script installs
ROS2 Humble + all dependencies, downloads this project and builds it:

```bash
wget https://raw.githubusercontent.com/anirudhatalmale6-alt/livox_high_precision_mapping_ros2/main/scripts/setup_minipc.sh
bash setup_minipc.sh
```

Run it as your normal user (not `sudo`); it will ask for your password when it
needs it. When it finishes, follow the "NEXT STEPS" it prints. It is safe to
re-run. See `scripts/setup_minipc.sh` for exactly what it does.

## Build (manual)

```bash
cd ws
rosdep install --from-paths src --ignore-src -r -y   # optional but recommended
colcon build --symlink-install
source install/setup.bash
```

## Run — online mapping

```bash
ros2 launch livox_hp_mapping_bringup mapping_online.launch.py \
    um982_port:=/dev/gps um982_baud:=115200 \
    use_gnss_heading:=true
```

This starts the UM982 driver, the IMU adapter (fed by the Avia's built-in IMU on
`/livox/imu`) and the mapping node (plus RViz). Only the Livox Avia driver is
started separately (or add it to the bringup). No separate IMU is needed by
default. To use an external IM10A instead, add
`start_im10a:=true imu_input_topic:=/imu/data im10a_port:=/dev/imu`.

The accumulated cloud is published on `/pub_pointcloud2` and written to
`all_points.pcd` on shutdown (`map_file_path:=/path` to choose the directory).

### RTK corrections (fix quality 1 → 4)

Out of the box the UM982 runs a single-point fix (~1–2 m). For centimetre
accuracy, feed it an RTCM3 correction stream (`rtcm_source` = `ntrip` | `serial`
| `mavlink`). Three routes:

```bash
# over the internet (NTRIP caster / state CORS network)
    rtcm_source:=ntrip ntrip_host:=www.nysnet.com ntrip_port:=2101 \
    ntrip_mountpoint:=RTCM3_NEAR ntrip_user:=USER ntrip_password:=PASS

# your own base + a radio serial bridge into the Pi
    rtcm_source:=serial rtcm_serial_port:=/dev/rtcm rtcm_serial_baud:=57600

# tap a Pixhawk's MAVLink stream (reuse an existing drone-RTK setup)
    rtcm_source:=mavlink mavlink_serial_port:=/dev/pixhawk mavlink_serial_baud:=57600
```

The driver injects the corrections straight into the UM982 (no extra cable) and
the fix flips to quality 4. Full setup for all three routes is in
[`docs/rtk_corrections.md`](docs/rtk_corrections.md).

To run just the IMU driver on its own (handy for a first smoke test):

```bash
ros2 launch im10a_driver im10a.launch.py           # or set port/baud in config
ros2 topic echo /imu/data                          # watch live IMU data
```

## Run — offline (rosbag)

Record the three raw streams once, then replay for repeatable mapping:

```bash
ros2 bag record /livox/lidar /gnss_inertial/imu /gnss_inertial/navsatfix
# ... later ...
ros2 bag play <bag>
ros2 launch livox_mapping livox_mapping.launch.py
```

## Configuration notes

- **`rtk2lidar` extrinsic** (`livox_hp_mapping_bringup/config/pipeline.yaml`):
  the 4×4 transform from the GNSS/IMU body frame to the LiDAR frame. Calibrate
  this for your actual mounting; the default matches the reference rig.
- **`lidar_delta_time`**: LiDAR frame period (default `0.01`s = 100 Hz). Set to
  match your Mid-40 publish rate for correct per-point deskew.
- **`heading_offset_deg`**: yaw offset between the UM982 antenna baseline and the
  IMU x-axis, applied when `use_gnss_heading` is on.

## Time synchronisation

For best precision, feed the UM982 **1PPS** output to the host and discipline the
system clock with `chrony`/`gpsd`, and/or wire 1PPS to the Livox converter sync
input exactly as the original APX-15 PPS setup did. The mapping node aligns the
three streams by ROS timestamp, so a disciplined clock directly improves the
result. Software-only timestamp alignment also works for lower-speed capture.

## Tests

`um982_driver` ships gtest coverage for the NMEA/Unicore parser:

```bash
colcon test --packages-select um982_driver
colcon test-result --verbose
```

## Credit

Ported from the original Livox-SDK project (see upstream for the hardware
assembly guide and coordinate conventions). Mapping algorithm © Livox; ROS2 port
and UM982/IM10A integration added here.
