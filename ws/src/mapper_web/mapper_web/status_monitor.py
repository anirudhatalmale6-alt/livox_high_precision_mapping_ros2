# Live sensor-health monitor.
#
# Subscribes to the streams the mapper depends on and keeps the shared state
# fresh so the dashboard's status lights are honest:
#   /livox/lidar             -> LiDAR streaming? at what rate
#   /livox/imu               -> IMU streaming? at what rate
#   /gnss_inertial/navsatfix -> GNSS fix quality + sat count
#   /livox/lidar_status      -> Avia device health (temp/volt/motor/dust/...)
#
# It also owns the command publisher for /livox/lidar_cmd - the LoggingController
# registers send_command() as its config sink so an APPLY on the dashboard turns
# into a real command to the forked Livox driver (the "LiDAR control link").
#
# Runs rclpy in its own thread. If ROS2 isn't importable (dev box), it falls
# back to a simulator so the whole service - and the UI - still runs.
import json
import threading
import time
from collections import deque

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
    from std_msgs.msg import Float64, String
    from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                           HistoryPolicy, ReliabilityPolicy)
    _HAVE_ROS = True
except Exception:
    _HAVE_ROS = False


_FIX = {  # NavSatFix.status.status -> label
    -1: 'No fix', 0: 'GPS (SPS)', 1: 'SBAS/DGPS', 2: 'RTK Fixed',
}

# Livox device-status code -> human label (raw severities from the driver).
_WORK = {0: 'Initializing', 1: 'Working Normally', 2: 'Power Saving',
         3: 'Standby', 4: 'Error', 5: 'Unknown'}
_TEMP = {0: 'Normal', 1: 'High/Low', 2: 'Extreme'}
_VOLT = {0: 'Normal', 1: 'High', 2: 'Extreme High'}
_MOTOR = {0: 'Normal', 1: 'Warning', 2: 'Error'}
_DIRTY = {0: 'Clean', 1: 'Dirty/Blocked'}
_LIFE = {0: 'OK', 1: 'Near end of life'}
_SYNC = {0: 'No PPS', 1: 'PTP sync', 2: 'GPS sync', 3: 'PPS sync',
         4: 'Sync error'}
_UNKNOWN_DEVICE = {
    'work_mode': 'Unknown', 'pps': 'Unknown', 'temperature': 'Unknown',
    'voltage': 'Unknown', 'motor': 'Unknown', 'dust': 'Unknown',
    'service_life': 'Unknown',
}


def gnss_label(fix, last_msg_t, now, stale_after=3.0):
    """What to show on the GNSS row: (label, is_live).

    "No fix" used to cover two completely different faults - a receiver that is
    connected and still searching, and one that is not talking to us at all
    (unplugged, wrong port, driver down). They call for opposite responses, so
    they must not share a label.
    """
    if last_msg_t and (now - last_msg_t) < stale_after:
        return fix, True
    if not last_msg_t:
        return 'Not connected', False
    return 'Receiver silent', False


class _RateTracker:
    """Rolling message rate over a short window.

    A deque, not a list: tick() runs on the IMU callback at 200 Hz, and the old
    version rebuilt the whole list on every single message.
    """
    def __init__(self, window=2.0):
        self.window = window
        self.stamps = deque()

    def tick(self):
        now = time.time()
        self.stamps.append(now)
        cutoff = now - self.window
        while self.stamps and self.stamps[0] < cutoff:
            self.stamps.popleft()

    def hz(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return round((len(self.stamps) - 1) / span, 1) if span > 0 else 0.0

    def alive(self, gap=1.5):
        return bool(self.stamps) and (time.time() - self.stamps[-1]) < gap


class StatusMonitor:
    def __init__(self, shared, simulate=False):
        self.s = shared
        self.simulate = simulate or not _HAVE_ROS
        self._stop = threading.Event()
        self._cmd_pub = None          # /livox/lidar_cmd, set once ROS is up
        self._cmd_lock = threading.Lock()

    def start(self):
        if self.simulate:
            threading.Thread(target=self._sim_loop, daemon=True).start()
        else:
            threading.Thread(target=self._ros_loop, daemon=True).start()

    def stop(self):
        self._stop.set()

    # ---- real ROS2 path ---------------------------------------------------
    def _ros_loop(self):
        # Do NOT let rclpy install signal handlers.
        #
        # rcl installs its own SIGINT (and on some builds SIGTERM) handler that
        # sets a shutdown flag rather than ending the process. The dashboard's
        # main thread is blocked in serve_forever(), which only unwinds on a
        # KeyboardInterrupt that then never comes - so the process sits there
        # until systemd gives up and SIGKILLs it. server.py installs handlers
        # that actually stop the HTTP server; these must not be replaced.
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError, AttributeError):
            rclpy.init()      # older rclpy - keep the previous behaviour
        node = Node('mapper_web_status')
        lidar, imu = _RateTracker(), _RateTracker()
        gnss_state = {'fix': 'No fix', 'sats': 0, 't': 0.0}
        dev_state = {'device': None, 't': 0.0}

        # The IMU arrives in bursts of ~20 every ~100 ms (the Livox driver only
        # drains its IMU queue when point-cloud data wakes its publish loop).
        # qos_profile_sensor_data keeps the last 5, so most of each burst was
        # dropped before the counter saw it and the row read ~50 Hz for a
        # 200 Hz sensor. Count what is actually being sent.
        burst_qos = QoSProfile(depth=2000,
                               history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(PointCloud2, '/livox/lidar',
                                 lambda m: lidar.tick(), burst_qos)
        node.create_subscription(Imu, '/livox/imu',
                                 lambda m: imu.tick(), burst_qos)

        def on_fix(m):
            gnss_state['fix'] = _FIX.get(int(m.status.status), 'No fix')
            # NavSatFix has no sat count; expose service bitmask width as a proxy
            gnss_state['sats'] = int(getattr(m.status, 'service', 0)) or gnss_state['sats']
            gnss_state['t'] = time.time()
        node.create_subscription(NavSatFix, '/gnss_inertial/navsatfix',
                                 on_fix, qos_profile_sensor_data)

        def on_status(m):
            dev_state['device'] = self._parse_status(m.data)
            dev_state['t'] = time.time()
        node.create_subscription(String, '/livox/lidar_status',
                                 on_status, 10)

        # The UM982 driver publishes (satellite - computer) seconds only while
        # GPS-time sync is actually working. Its presence is the proof that
        # scans are stamped with satellite time; its absence means the mapper
        # is quietly using the computer clock.
        tsync = {'offset': 0.0, 't': 0.0}

        def on_offset(m):
            tsync['offset'] = float(m.data)
            tsync['t'] = time.time()
        node.create_subscription(Float64, '/gnss_inertial/time_offset',
                                 on_offset, qos_profile_sensor_data)

        # Command channel to the forked Livox driver (the control link).
        with self._cmd_lock:
            self._cmd_pub = node.create_publisher(String, '/livox/lidar_cmd', 10)

        # Spin on a dedicated thread instead of calling spin_once() in this
        # loop. spin_once() handles ONE callback per call, so the number of
        # messages we could ever count was capped by how fast this loop went
        # round - roughly 50 per second once the status merging and JSON work
        # is included. The IMU publishes at 200 Hz, so the dashboard reported
        # about 50 Hz and looked like failing hardware when the only thing
        # falling behind was our own counter. The LiDAR at 10 Hz fitted under
        # that ceiling, which is why it always looked correct.
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()

        while not self._stop.is_set():
            time.sleep(0.2)
            self.s.merge('lidar', ok=lidar.alive(), rate_hz=lidar.hz())
            self.s.merge('imu', ok=imu.alive(), rate_hz=imu.hz())
            gnss_fix, gnss_ok = gnss_label(gnss_state['fix'], gnss_state['t'],
                                           time.time())
            self.s.merge('gnss', ok=gnss_ok, fix=gnss_fix,
                         sats=gnss_state['sats'] if gnss_ok else 0)
            # 5 s: the offset is republished with every GPS fix (~1 Hz), so
            # anything older than a few seconds means it has stopped.
            ts_ok = (time.time() - tsync['t']) < 5.0
            self.s.merge('time_sync', ok=ts_ok,
                         source='GPS satellite time' if ts_ok else 'Computer clock',
                         offset_s=round(tsync['offset'], 3) if ts_ok else 0.0)
            # Device rows: apply the latest status if it's fresh, else Unknown
            # (driver not running / lidar unplugged) so we never show stale data.
            if dev_state['device'] and (time.time() - dev_state['t']) < 4.0:
                self.s.merge('device', **dev_state['device'])
            else:
                self.s.merge('device', **_UNKNOWN_DEVICE)
            self.s.update(connected=lidar.alive())
        with self._cmd_lock:
            self._cmd_pub = None
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def _parse_status(raw):
        """Map a /livox/lidar_status JSON payload to device-row labels."""
        try:
            d = json.loads(raw)
        except (ValueError, TypeError):
            return None
        if not d.get('present'):
            return dict(_UNKNOWN_DEVICE)
        dev = {'work_mode': _WORK.get(int(d.get('work_state', 5)), 'Unknown')}
        fw = d.get('firmware')
        if fw and fw != 'unknown':
            dev['firmware'] = fw
        if d.get('err_valid'):
            dev['temperature'] = _TEMP.get(int(d.get('temp', 0)), 'Unknown')
            dev['voltage'] = _VOLT.get(int(d.get('volt', 0)), 'Unknown')
            dev['motor'] = _MOTOR.get(int(d.get('motor', 0)), 'Unknown')
            dev['dust'] = _DIRTY.get(int(d.get('dirty', 0)), 'Unknown')
            dev['service_life'] = _LIFE.get(int(d.get('service_life', 0)), 'Unknown')
            dev['pps'] = _SYNC.get(int(d.get('time_sync', 0)), 'Unknown')
        return dev

    # ---- command sink (registered on the LoggingController) --------------
    def send_command(self, cfg):
        """Publish a LiDAR config command to the driver. cfg is the dict of
        LiDAR settings from the dashboard APPLY (echo/work/imu/scan/coord/
        high-sensitivity). Returns (ok, message)."""
        with self._cmd_lock:
            pub = self._cmd_pub
        if pub is None:
            return False, ('LiDAR control link not up yet - is the Livox '
                           'driver running?')
        try:
            msg = String()
            msg.data = json.dumps(cfg)
            pub.publish(msg)
            return True, ('sent to LiDAR: ' +
                          ', '.join('%s=%s' % (k, v) for k, v in cfg.items()))
        except Exception as e:
            return False, 'could not send to LiDAR: ' + str(e)

    # ---- simulator (dev box) ---------------------------------------------
    def _sim_loop(self):
        self.s.update(connected=True)
        self.s.merge('lidar', ok=True, rate_hz=10.0)
        self.s.merge('imu', ok=True, rate_hz=200.0)
        self.s.merge('gnss', ok=True, fix='RTK Fixed', sats=28)
        self.s.merge('time_sync', ok=True, source='GPS satellite time',
                     offset_s=0.042)
        self.s.merge('device', work_mode='Working Normally', pps='No PPS',
                     temperature='Normal', voltage='Normal', motor='Normal',
                     dust='Clean', service_life='OK', firmware='11.08.0006')
        while not self._stop.is_set():
            time.sleep(0.5)
