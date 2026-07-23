# Time synchronisation (UM982 1PPS + mini PC, Ubuntu 22.04)

The mapping node aligns the LiDAR, IMU and RTK streams by ROS timestamp, so the
better the host clock tracks GPS time, the sharper the map. With the UM982 1PPS
wired in, discipline the mini-PC clock to GPS as follows.

There are two independent uses of PPS in this rig; do both if you can.

But first, the easiest option, which needs no system setup at all:

## 0. Built-in software GPS-time (recommended — no OS config, no rewiring)

The drivers can put the whole recording on satellite (GPS/UTC) time by
themselves, without gpsd/chrony or any LiDAR sync wiring. Turn it on with a
single switch:

- One-command (Style B): `mapping_online.launch.py ... gps_time_sync:=true`
- Split (Style A): pass `gps_time_sync:=true` to **sensors.launch.py** AND
  `use_gps_time:=true` to **mapping.launch.py** (both terminals).

What it does under the hood:

- The UM982 driver asks the receiver for `RMC` (UTC date + time), stamps
  `NavSatFix` with that satellite time, and publishes the measured
  `(satellite − computer)` clock offset on `/gnss_inertial/time_offset`.
- The mapping node adds that offset to every LiDAR and IMU stamp, so all three
  streams end up on one satellite clock. The offset is smoothed, so a computer
  clock that is wrong — or that jumps mid-run — no longer affects the map.

Why this is enough: map sharpness only needs the three sensors to agree with
*each other*; this makes them agree on GPS time specifically, so the result is
both sharp and carries true absolute timestamps. Verify it's active — you'll see
`GPS-time sync ON` from the UM982 driver and a value on
`/gnss_inertial/time_offset`. Default is **off** (computer-clock stamping), which
also produces sharp maps because all sensors share the one computer clock.

Sections 1 and 2 below are the hardware-PPS routes. They add the last slice of
absolute precision (they remove the few-millisecond serial latency that the
software offset still carries), but they are optional on top of section 0.

## 1. Discipline the host clock with gpsd + chrony

Feed the UM982's NMEA (for coarse time) and 1PPS (for the sharp edge) to the
mini PC. If 1PPS is on a serial DCD line or a GPIO, wire it accordingly.

```bash
sudo apt install gpsd gpsd-clients chrony pps-tools

# Point gpsd at the UM982 serial device (adjust /dev/ttyUSB0 and baud).
sudo tee /etc/default/gpsd >/dev/null <<'EOF'
DEVICES="/dev/ttyUSB0"
GPSD_OPTIONS="-n"
EOF
sudo systemctl restart gpsd

# Verify PPS is arriving (if exposed as /dev/pps0):
sudo ppstest /dev/pps0
```

Add GPS + PPS reference clocks to `/etc/chrony/chrony.conf`:

```
# NMEA time from gpsd shared memory (coarse)
refclock SHM 0 refid NMEA offset 0.0 precision 1e-3 poll 3
# PPS edge (sharp) — locks the sub-second phase
refclock SHM 1 refid PPS precision 1e-7 prefer
```

```bash
sudo systemctl restart chrony
chronyc sources -v          # NMEA + PPS should appear and lock
```

## 2. LiDAR hardware timestamping (as in the original APX-15 rig)

The original design also fed PPS + a GNRMC-style time string into the Livox
converter so the Mid-40 hardware-timestamps its points to GPS time. If your
converter box supports the sync input, wire the UM982 1PPS to it and enable
`timesync` in the `livox_ros_driver2` config (device_name = the serial port that
carries the time string). This makes the per-point deskew maximally accurate.

## Notes

- With the RTK base station active, the UM982 outputs a fixed RTK solution
  (GGA quality = 4). The driver publishes that directly; no code change needed.
- If you cannot wire PPS for a given capture, software timestamp alignment still
  works — you simply lose the last bit of deskew precision at high speed.
