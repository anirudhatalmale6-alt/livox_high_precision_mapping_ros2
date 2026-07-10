# RTK corrections (getting from fix quality 1 → 4)

The UM982 gives you a **standalone single-point fix** out of the box: GGA fix
quality `1`, roughly 1–2 metres. That wander is enough to smear an outdoor map,
because the mapping node places every LiDAR frame at the GNSS position.

To reach **centimetre accuracy** the receiver needs a stream of **RTCM3
corrections** fed into its serial port. Once corrections are flowing the fix
quality flips to `4` (RTK fixed, ~1–2 cm) or `5` (RTK float, ~decimetre).

The UM982 driver can source those corrections two ways. In both cases the driver
injects the RTCM bytes back into the same serial port the UM982 is already on —
you do **not** need a second cable to the receiver.

Check your current fix quality any time with:

```bash
ros2 topic echo /gnss_inertial/navsatfix --once
# status.status == 2 (GBAS_FIX) once RTK/DGPS is active; -1/0/1 before that
```

or watch the raw GGA quality field directly from the receiver.

---

## Route 1 — NTRIP over the internet (simplest, no base station)

Corrections come from a public/commercial NTRIP caster (for example a state
CORS network such as **NYSNet** in New York). The Pi needs internet where you
scan — a phone hotspot is fine. No base station, no radios.

You need four things from the caster operator:

* host (e.g. `www.nysnet.com`)
* port (usually `2101`)
* mountpoint nearest you (e.g. `RTCM3_NEAR`)
* username / password

Launch with:

```bash
ros2 launch livox_hp_mapping_bringup mapping_online.launch.py \
  rviz:=false \
  rtcm_source:=ntrip \
  ntrip_host:=www.nysnet.com \
  ntrip_port:=2101 \
  ntrip_mountpoint:=RTCM3_NEAR \
  ntrip_user:=YOUR_USER \
  ntrip_password:=YOUR_PASS
```

The driver connects to the caster, streams RTCM into the UM982, and (for VRS /
network mountpoints) reports the rover's position back with a GGA sentence so the
network computes corrections for exactly where you are. You'll see
`RTCM corrections flowing (... KB injected)` in the log, and the fix quality
climbs to 4 within a minute or two of open sky.

---

## Route 2 — Your own base station + a radio link (no internet)

Your base GNSS sits on a tripod at a fixed spot and outputs RTCM3. A pair of
telemetry radios (e.g. the 915 MHz SiK radios that ship with most drones) acts
as a transparent wireless serial cable: base RTCM into one radio, out the other
into the Pi. The rover radio shows up on the Pi as a serial device — point the
driver at it:

```bash
ros2 launch livox_hp_mapping_bringup mapping_online.launch.py \
  rviz:=false \
  rtcm_source:=serial \
  rtcm_serial_port:=/dev/rtcm \
  rtcm_serial_baud:=57600
```

Give the rover radio a stable name with a udev rule (same idea as `/dev/gps` and
`/dev/imu`) so it's always `/dev/rtcm`. Match `rtcm_serial_baud` to whatever the
radios are configured for (57600 is a common SiK default).

> Note: routing corrections *through the drone's Pixhawk* is not recommended —
> ArduPilot consumes injected RTCM for its own GPS and does not re-emit it on a
> serial port. Use the radios as a plain serial bridge instead, straight into the
> Pi, bypassing the flight controller.

---

## Heading vs. position

Two independent things improve your map:

* **Position RTK** (this document) — fixes the ~1–2 m wander so frames land in
  the right global spot. Needed for the sharp outdoor map.
* **Dual-antenna heading** — the UM982's absolute, drift-free yaw, fused in by
  `imu_gnss_adapter` when `use_gnss_heading:=true`. This is what keeps the map
  from smearing as you rotate outdoors.

For the best outdoor result run with RTK corrections **and**
`use_gnss_heading:=true`. Indoors (no sky) keep `use_gnss_heading:=false` and
just pan slowly in place — see the mapping notes.
