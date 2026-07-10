# RTK corrections (getting from fix quality 1 → 4)

The UM982 gives you a **standalone single-point fix** out of the box: GGA fix
quality `1`, roughly 1–2 metres. That wander is enough to smear an outdoor map,
because the mapping node places every LiDAR frame at the GNSS position.

To reach **centimetre accuracy** the receiver needs a stream of **RTCM3
corrections** fed into its serial port. Once corrections are flowing the fix
quality flips to `4` (RTK fixed, ~1–2 cm) or `5` (RTK float, ~decimetre).

The UM982 driver can source those corrections three ways (`rtcm_source` =
`ntrip` | `serial` | `mavlink`). In every case the driver injects the RTCM bytes
back into the same serial port the UM982 is already on — you do **not** need a
second cable to the receiver.

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

---

## Route 3 — tap a Pixhawk's MAVLink stream (reuse your existing drone RTK)

If your RTK base **already feeds a Pixhawk/ArduPilot flight controller** (base
GNSS on the ground station → SiK radio → autopilot, which is a very common
drone-RTK setup), the corrections are already travelling over the MAVLink link
as `GPS_RTCM_DATA` messages. ArduPilot forwards those broadcast messages on to
its other MAVLink links, so a companion Pi can pull the **exact same
corrections** back out and feed them to the UM982 — no separate correction
radio for the mapping receiver at all.

Connect the Pi to the MAVLink stream (a serial/telemetry link to the flight
controller, or the Pi's own paired SiK radio) and launch:

```bash
ros2 launch livox_hp_mapping_bringup mapping_online.launch.py \
  rviz:=false \
  rtcm_source:=mavlink \
  mavlink_serial_port:=/dev/pixhawk \
  mavlink_serial_baud:=57600
```

The driver decodes MAVLink (v1 and v2), extracts and reassembles the
`GPS_RTCM_DATA` fragments, CRC-checks every frame, and injects the RTCM into the
UM982. Use `57600` for a SiK telemetry link, or `115200` for a direct USB link
to the flight controller. Give the FC link a stable udev name (`/dev/pixhawk`)
the same way as the sensors.

> Note the distinction: this taps the RTCM that flows **over the MAVLink link**,
> which works cleanly. What does *not* work is trying to pull corrections out of
> the flight controller's **GPS UART** — ArduPilot consumes those for its own
> receiver and doesn't re-emit them. Always take them from the MAVLink stream.

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
