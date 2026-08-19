#!/usr/bin/env bash
# =============================================================================
# check_rates.sh — measure the true publish rate of the LiDAR and IMU topics.
#
#   bash scripts/check_rates.sh
#
# Why not `ros2 topic hz`: sensor topics are published BEST_EFFORT, while the
# CLI subscribes RELIABLE. Incompatible QoS means the subscription never
# matches, so it prints nothing at all - no error, no warning, just silence
# that looks exactly like a dead sensor. And `ros2 topic hz` has no QoS flags
# to fix it with (those exist on `ros2 topic echo`, not `hz`).
#
# So this subscribes with the correct sensor QoS itself, and also prints what
# each publisher is actually offering, which is the thing worth knowing when a
# topic looks silent.
# =============================================================================
set -u
SECS="${1:-10}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)/ws"

# ---- must run as root --------------------------------------------------------
# The field service runs as root, so ROS2's shared-memory segments in /dev/shm
# belong to root. An ordinary user discovers the publishers over the network but
# receives nothing at all - which looks precisely like an unplugged sensor, and
# was in fact misread as one here.
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

SECS="$SECS" python3 - <<'PY'
import os
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

SECS = float(os.environ.get('SECS', '10'))
TOPICS = [('/livox/lidar', PointCloud2), ('/livox/imu', Imu)]

rclpy.init()
node = Node('rate_check')

counts = {}
first = {}
last = {}


def make_cb(name):
    def cb(_msg):
        now = time.time()
        counts[name] = counts.get(name, 0) + 1
        first.setdefault(name, now)
        last[name] = now
    return cb


print('')
print('What each publisher is offering:')
for name, msg_type in TOPICS:
    info = node.get_publishers_info_by_topic(name)
    if not info:
        print('  %-14s NO PUBLISHER - nothing is producing this topic' % name)
        continue
    for p in info:
        q = p.qos_profile
        print('  %-14s %s / %s   (publisher: %s)' % (
            name, q.reliability.name, q.durability.name, p.node_name))
    node.create_subscription(msg_type, name, make_cb(name),
                             qos_profile_sensor_data)

print('')
print('Measuring for %d seconds...' % SECS)
# Spin on its own thread. Calling spin_once() in a timing loop handles ONE
# callback per call, which caps what can be counted at the loop rate - that is
# exactly the bug being investigated here, and measuring it with the same flaw
# would just reproduce the wrong answer.
executor = SingleThreadedExecutor()
executor.add_node(node)
spinner = threading.Thread(target=executor.spin, daemon=True)
spinner.start()
time.sleep(SECS)
executor.shutdown()
spinner.join(timeout=2.0)

print('')
print('Measured rates:')
silent = 0
for name, _ in TOPICS:
    n = counts.get(name, 0)
    if n < 2:
        print('  %-14s %d messages - nothing arriving' % (name, n))
        silent += 1
        continue
    span = last[name] - first[name]
    hz = (n - 1) / span if span > 0 else 0.0
    print('  %-14s %6.1f Hz   (%d messages in %.1f s)' % (name, hz, n, span))

print('')
if silent == len(TOPICS):
    # Publishers existing while nothing flows is a specific, useful symptom:
    # the driver is running and has advertised its topics, but the device is
    # not feeding it. That is hardware, not software.
    print('BOTH topics are silent while their publishers exist.')
    print('')
    print('This script runs as root, so the usual cause - shared-memory')
    print('segments owned by the root-run field service and unreadable by an')
    print('ordinary user - is already ruled out. This is genuinely no data.')
    print('')
    print('The IMU is inside the Avia, so no LiDAR means no IMU either; both')
    print('go quiet together. Check, in this order:')
    print('  1. Is the Avia powered and its cable connected?')
    print('  2. Is it on the network?   ping 192.168.1.1xx  (the LiDAR IP)')
    print('  3. Is the unit running?    systemctl status mapper-field')
    print('  4. Driver log:             journalctl -u mapper-field -n 50')
    print('  5. Compare with the dashboard: if the web page shows the topics')
    print('     streaming while this shows nothing, the fault is here, not on')
    print('     the hardware - tell me and I will fix the tool.')
else:
    print('Expected: /livox/lidar about 10 Hz, /livox/imu about 200 Hz.')
    print('If the IMU comes out near 200, the dashboard was under-counting and')
    print('is now fixed. If it really is near 50, the LiDAR itself is pushing')
    print('at that rate and the next place to look is the Avia IMU setting.')

node.destroy_node()
rclpy.shutdown()
PY
