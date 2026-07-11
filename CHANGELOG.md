# Changelog / What changed from the original

You do **not** need to re-apply any of these by hand. Every fix below is
already baked into this repository and into `scripts/setup_minipc.sh`. If you
start from a clean Ubuntu 22.04 machine, run the setup script (or `git pull` on
a machine you already built), rebuild, and you have all of it. This page just
records what was done and why, so you never have to dig through chat.

Nothing here is a step you have to do — it is a history of work already in the code.

---

## 1. Ported the whole thing from ROS1 (Ubuntu 16.04) to ROS2 Humble (Ubuntu 22.04)

- The original was a ROS1 catkin package tested only on Ubuntu 16.04. This is a
  full ROS2 Humble / colcon rebuild that runs on Ubuntu 22.04.
- The core mapping maths (Mercator georeferencing, per-point motion
  "deskew", and the `rtk2lidar` extrinsic transform) was carried over 1:1, so
  the map output is equivalent to the original.
- Fixed a bug in the port where the global map anchor was reset every frame
  (would have collapsed every frame onto its own origin) — now a persistent
  anchor, exactly like the original.

## 2. Swapped the discontinued Applanix APX-15 for your sensor set

The APX-15 was one survey-grade GNSS/INS unit. It is rebuilt from two devices:

- **Unicore UM982** dual-antenna RTK GNSS — centimetre position + true heading.
  A from-scratch serial driver (`um982_driver`) reads its NMEA/Unicore output.
- **Hiwonder IM10A** 10-axis IMU — attitude. A from-scratch serial driver
  (`im10a_driver`) reads its WitMotion 0x55 protocol directly, so you do **not**
  need Hiwonder's vendor ROS driver at all.
- **Livox Avia** LiDAR — supported via `livox_ros2_driver` (SDK v1), configured
  for your unit (broadcast code, PPS time-sync, IMU off so the IM10A is the
  single IMU).

## 3. One-command setup script for a clean machine

- `scripts/setup_minipc.sh` installs ROS2 Humble + every dependency, clones the
  repo, and builds it. It is safe to re-run and auto-detects the CPU
  architecture, so it works on both x86 mini PCs and the Raspberry Pi (arm64).

## 4. Build fixes baked in (so your machine compiles cleanly)

These were real errors hit on your box during setup; all are now handled
automatically by the setup script / repo:

- Setup script no longer trips over ROS2's own `setup.bash` (`AMENT_TRACE_SETUP_FILES` unbound).
- `im10a_driver` compiles on GCC 11 (added the `<cstddef>` include, qualified `std::size_t`).
- Livox-SDK builds on GCC 11 (missing `<memory>` include added automatically).
- Livox driver builds with all its bundled sub-packages (`--packages-up-to`).
- Livox-SDK is built with `-fPIC` so the ROS2 driver links.

## 5. Runtime fixes that made the live map work

- **Stable device names.** Both sensors use the same USB chip, so `/dev/ttyUSB*`
  numbers can swap on boot. A udev rule pins them to `/dev/gps` and `/dev/imu`.
- **Blank-map fix (QoS).** The mapping node was silently dropping every IMU and
  GPS message because of a ROS2 "quality of service" mismatch — fixed, so the
  mapper actually receives the data now.
- **RViz.** The visualiser config was still using ROS1 plugin names; rewritten
  for ROS2, and coloured correctly for the point cloud.

## 6. RTK corrections — fix quality 1 to 4 (added later)

Out of the box the UM982 runs a standalone fix (~1-2 m). For centimetre
accuracy it needs an RTCM3 correction stream. Three selectable sources were
built in (`rtcm_source` launch argument), all injecting into the same UM982
port — no extra cable:

- `ntrip` — pull corrections over the internet from an NTRIP caster / CORS network.
- `serial` — feed corrections from your own base station over a radio serial bridge.
- `mavlink` — tap the `GPS_RTCM_DATA` stream off your drone's Pixhawk over MAVLink
  (reuses the base-station RTK you already have flying).

See [`docs/rtk_corrections.md`](docs/rtk_corrections.md) for the full setup of each.

---

## Where the detail lives

- Per-change history with reasons: `git log` in this repo (each commit message
  explains one fix).
- LiDAR setup: [`docs/lidar_avia.md`](docs/lidar_avia.md)
- RTK corrections: [`docs/rtk_corrections.md`](docs/rtk_corrections.md)
- Time sync (1PPS): [`docs/time_sync.md`](docs/time_sync.md)
- Everything to run it: [`README.md`](README.md)
