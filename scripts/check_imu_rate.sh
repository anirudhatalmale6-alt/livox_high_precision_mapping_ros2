#!/usr/bin/env bash
# =============================================================================
# check_imu_rate.sh — work out WHY the Avia's IMU reads ~50 Hz instead of 200.
#
#   bash scripts/check_imu_rate.sh [seconds]     (default 15)
#
# check_rates.sh answers "what is the rate". This answers "where did the other
# 150 Hz go", which is a different question and needs different evidence.
#
# There are only three ways a 200 Hz sensor shows up as 50 Hz, and each one
# leaves a different fingerprint in the timing:
#
#   1. The device really is pushing 50 Hz.
#      -> samples arrive evenly, ~19 ms apart, and the DEVICE timestamps are
#         also ~19 ms apart. Nothing is lost; it was never sent.
#
#   2. The device pushes 200 Hz but packets are being lost (network / USB).
#      -> device timestamps come in multiples of ~5 ms with gaps: 5, 5, 20,
#         5, 15 ... The missing samples leave holes in the device clock.
#
#   3. The device pushes 200 Hz but the driver publishes them in bursts.
#      -> several arrive back-to-back (sub-millisecond apart) and then a long
#         pause. Device timestamps stay an even ~5 ms apart - nothing is lost,
#         it is just delivered in clumps.
#
# So this records both the ARRIVAL time and the DEVICE timestamp of every
# message, prints a histogram of the gaps in each, and reads the network error
# counters over the same window. The histogram shape is the answer.
# =============================================================================
set -u
SECS="${1:-15}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)/ws"

GREEN='\033[1;32m'; YELLOW='\033[1;33m'; BLUE='\033[1;34m'; NC='\033[0m'
step() { echo -e "\n${BLUE}==== $* ====${NC}"; }

# ---- must run as root --------------------------------------------------------
# mapper-field.service runs as root, so every ROS node on this box is root's.
# ROS2 hands messages between processes on the same machine through shared
# memory in /dev/shm, and those segments are owned by root. A subscriber running
# as an ordinary user still DISCOVERS the publishers over the network - so the
# topic looks alive and `get_publishers_info_by_topic` lists it - but not one
# message is ever delivered.
#
# That is indistinguishable from a dead sensor, and it fooled this script once
# already. So don't allow the situation to arise.
if [ "$(id -u)" != "0" ]; then
  echo "  (re-running as root - ROS2 shared memory belongs to the field service)"
  if sudo -E true 2>/dev/null; then
    exec sudo -E bash "$SCRIPT_DIR/$(basename "$0")" "$@"
  fi
  exec sudo bash "$SCRIPT_DIR/$(basename "$0")" "$@"
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"
set -u

# ---- machine load ------------------------------------------------------------
# A board that cannot keep up will drop the IMU before it drops the point cloud,
# so "is this thing saturated" has to be part of the answer.
step "Machine"
echo "  cores    : $(nproc 2>/dev/null || echo '?')"
echo "  load avg : $(cut -d' ' -f1-3 /proc/loadavg 2>/dev/null)"
if command -v free >/dev/null 2>&1; then
  echo "  memory   : $(free -h | awk '/^Mem:/ {print $3 " used of " $2 ", " $7 " available"}')"
fi
echo "  busiest processes:"
ps -eo pcpu,comm --sort=-pcpu 2>/dev/null | head -6 | sed 's/^/    /'

# ---- network counters, before ------------------------------------------------
step "Network error counters (before)"
snap_net() {
  for d in /sys/class/net/*; do
    ifn="$(basename "$d")"
    [ "$ifn" = "lo" ] && continue
    [ -r "$d/statistics/rx_packets" ] || continue
    printf '%s %s %s %s %s\n' "$ifn" \
      "$(cat "$d/statistics/rx_packets" 2>/dev/null || echo 0)" \
      "$(cat "$d/statistics/rx_dropped" 2>/dev/null || echo 0)" \
      "$(cat "$d/statistics/rx_errors"  2>/dev/null || echo 0)" \
      "$(cat "$d/statistics/rx_missed_errors" 2>/dev/null || echo 0)"
  done
}
NET_BEFORE="$(snap_net)"
echo "$NET_BEFORE" | awk '{printf "  %-10s rx=%s dropped=%s errors=%s missed=%s\n",$1,$2,$3,$4,$5}'

# ---- the measurement ---------------------------------------------------------
step "Listening to /livox/imu for ${SECS}s"
SECS="$SECS" python3 - <<'PY'
import os
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

# The IMU arrives in bursts of ~20 every ~100 ms. qos_profile_sensor_data keeps
# only the last 5, so most of a burst is discarded before any callback runs -
# the tool would report the size of its own queue rather than the rate being
# published. That is exactly the mistake this script exists to catch, so it
# must not make it itself.
BURST_QOS = QoSProfile(depth=2000, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT)

SECS = float(os.environ.get('SECS', '15'))

rclpy.init()
node = Node('imu_rate_check')

arrivals = []     # host clock, when the message reached us
stamps = []       # header.stamp, when the device says the sample was taken


def cb(msg):
    arrivals.append(time.time())
    s = msg.header.stamp
    stamps.append(s.sec + s.nanosec * 1e-9)


info = node.get_publishers_info_by_topic('/livox/imu')
if not info:
    print('  NO PUBLISHER on /livox/imu - the Livox driver is not running.')
    print('  Nothing to measure. Check:  systemctl status mapper-field')
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(0)
for p in info:
    q = p.qos_profile
    print('  publisher: %s   %s / %s' % (p.node_name, q.reliability.name,
                                         q.durability.name))

node.create_subscription(Imu, '/livox/imu', cb, BURST_QOS)

executor = SingleThreadedExecutor()
executor.add_node(node)
spinner = threading.Thread(target=executor.spin, daemon=True)
spinner.start()
time.sleep(SECS)
executor.shutdown()
spinner.join(timeout=2.0)
node.destroy_node()
rclpy.shutdown()

n = len(arrivals)
print('  received %d messages' % n)
if n == 0:
    print('')
    print('  A publisher exists but nothing arrived. Running as root rules out')
    print('  the shared-memory permission problem, so this time it really is')
    print('  no data: the Avia is not sending, or the driver is not reading it.')
    print('  Check:  journalctl -u mapper-field -n 50')
    raise SystemExit(0)
if n < 20:
    print('  Too few to analyse.')
    raise SystemExit(0)

span = arrivals[-1] - arrivals[0]
arr_hz = (n - 1) / span if span > 0 else 0.0
print('')
print('  MEASURED RATE : %.1f Hz   (%d messages in %.1f s)' % (arr_hz, n, span))


def gaps(seq):
    return [b - a for a, b in zip(seq, seq[1:]) if b > a]


def histogram(label, vals, note):
    """Buckets to the nearest millisecond and shows the common ones. The shape
    is what matters, so the buckets are printed in time order, not by size."""
    if not vals:
        print('  %s: no usable gaps' % label)
        return
    buckets = {}
    for v in vals:
        ms = round(v * 1000.0)
        buckets[ms] = buckets.get(ms, 0) + 1
    total = len(vals)
    top = sorted(buckets.items(), key=lambda kv: -kv[1])[:8]
    top.sort()
    print('')
    print('  %s  (%s)' % (label, note))
    for ms, count in top:
        pct = 100.0 * count / total
        bar = '#' * max(1, int(pct / 2))
        print('    %4d ms  %6.1f%%  %s' % (ms, pct, bar))
    ordered = sorted(vals)
    print('    median gap %.2f ms' % (ordered[len(ordered) // 2] * 1000.0))


arr_gaps = gaps(arrivals)
histogram('Gaps between ARRIVALS', arr_gaps,
          'even = steady stream, mixed tiny+large = delivered in bursts')

# Are the device timestamps real, or is the driver just stamping with host
# time? If they track the host clock exactly, the device-clock analysis below
# is meaningless and saying otherwise would be inventing evidence.
offsets = [a - s for a, s in zip(arrivals, stamps)]
off_spread = max(offsets) - min(offsets)
mean_off = sum(offsets) / len(offsets)
print('')
print('  arrival minus device timestamp: mean %.3f s, spread %.3f s'
      % (mean_off, off_spread))

if abs(mean_off) < 0.0005 and off_spread < 0.002:
    print('  -> these look like HOST timestamps, not device ones.')
    print('     Cannot tell dropped packets from a slow device this way.')
else:
    st_gaps = gaps(stamps)
    histogram('Gaps between DEVICE timestamps', st_gaps,
              'clean ~5 ms = device at 200 Hz, ~19 ms = device really at 50 Hz')

    if st_gaps:
        ordered = sorted(st_gaps)
        med_ms = ordered[len(ordered) // 2] * 1000.0
        # The shortest interval the device demonstrably produces. If ANY two
        # samples are 5 ms apart then the sensor is sampling at 200 Hz, full
        # stop - no amount of downstream loss can invent a short gap.
        p05_ms = ordered[max(0, int(len(ordered) * 0.05))] * 1000.0
        # How even are the gaps? This is the part that matters: dropped packets
        # and a slow sensor can produce the SAME median. What separates them is
        # regularity - real UDP loss is never perfectly periodic over thousands
        # of packets, whereas a sensor's own sample clock is.
        modal_ms = round(med_ms)
        near_modal = sum(1 for g in st_gaps if abs(g * 1000.0 - modal_ms) < 1.0)
        uniform = 100.0 * near_modal / len(st_gaps)
        print('')
        print('  shortest gap seen (5th pct) : %.1f ms' % p05_ms)
        print('  gaps within 1 ms of %-3d ms  : %.1f%%' % (modal_ms, uniform))
        print('')
        print('  ---- READING ----')
        if p05_ms < 8:
            print('  Some samples are only %.1f ms apart, so the Avia IS' % p05_ms)
            print('  sampling at 200 Hz - downstream loss cannot create a gap')
            print('  shorter than the sensor produces. So the samples are being')
            print('  lost or clumped between the LiDAR and here: check the')
            print('  arrival histogram above and the network counters below.')
        elif uniform > 98.0:
            print('  Every gap is %.0f ms, to the millisecond. Dropped packets are' % med_ms)
            print('  never that regular over thousands of samples, so this is the')
            print('  Avia pacing its own IMU at about %.0f Hz - the other samples' % (1000.0 / med_ms))
            print('  are not being lost, they are never sent.')
            print('')
            print('  Next: press APPLY CONFIG on the dashboard with IMU Push')
            print('  Frequency set to 200 Hz, then run this again.')
            print('  (If the counters below show packets dropping, tell me -')
            print('   that would change this answer.)')
        else:
            print('  Gaps vary (median %.0f ms) but none are near 5 ms. That is' % med_ms)
            print('  the pattern for irregular packet loss rather than a slow')
            print('  sensor. Check the network counters below and send me this.')
PY

# ---- network counters, after -------------------------------------------------
step "Network error counters (after)"
NET_AFTER="$(snap_net)"
echo "$NET_AFTER" | awk '{printf "  %-10s rx=%s dropped=%s errors=%s missed=%s\n",$1,$2,$3,$4,$5}'

echo
echo "  Change over the test window:"
awk -v before="$NET_BEFORE" '
BEGIN {
  n = split(before, lines, "\n")
  for (i = 1; i <= n; i++) {
    split(lines[i], f, " ")
    if (f[1] != "") { rx[f[1]]=f[2]; dr[f[1]]=f[3]; er[f[1]]=f[4]; ms[f[1]]=f[5] }
  }
}
{
  printf "    %-10s +%d packets, +%d dropped, +%d errors, +%d missed\n", \
    $1, $2-rx[$1], $3-dr[$1], $4-er[$1], $5-ms[$1]
}' <<< "$NET_AFTER"

cat <<'EOF'

  Any non-zero "dropped" or "missed" on the LiDAR's interface means packets are
  being lost on the way in - which on a USB-attached Ethernet adapter usually
  means the USB bus, not the network.

Send me everything above and I'll tell you which of the three cases it is.
EOF
