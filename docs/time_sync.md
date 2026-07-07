# Time synchronisation (UM982 1PPS + mini PC, Ubuntu 22.04)

The mapping node aligns the LiDAR, IMU and RTK streams by ROS timestamp, so the
better the host clock tracks GPS time, the sharper the map. With the UM982 1PPS
wired in, discipline the mini-PC clock to GPS as follows.

There are two independent uses of PPS in this rig; do both if you can:

## 1. Discipline the host clock with gpsd + chrony (recommended)

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
