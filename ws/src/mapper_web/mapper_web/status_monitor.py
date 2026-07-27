# Live sensor-health monitor.
#
# Subscribes to the three streams the mapper depends on and keeps the shared
# state fresh so the dashboard's status lights are honest:
#   /livox/lidar            -> LiDAR streaming? at what rate
#   /livox/imu              -> IMU streaming? at what rate
#   /gnss_inertial/navsatfix -> GNSS fix quality + sat count
#
# Runs rclpy in its own thread. If ROS2 isn't importable (dev box), it falls
# back to a simulator so the whole service - and the UI - still runs.
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
    from rclpy.qos import qos_profile_sensor_data
    _HAVE_ROS = True
except Exception:
    _HAVE_ROS = False


_FIX = {  # NavSatFix.status.status -> label
    -1: 'No fix', 0: 'GPS (SPS)', 1: 'SBAS/DGPS', 2: 'RTK Fixed',
}


class _RateTracker:
    """Rolling message rate over a short window."""
    def __init__(self, window=2.0):
        self.window = window
        self.stamps = []

    def tick(self):
        now = time.time()
        self.stamps.append(now)
        self.stamps = [t for t in self.stamps if now - t <= self.window]

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

    def start(self):
        if self.simulate:
            threading.Thread(target=self._sim_loop, daemon=True).start()
        else:
            threading.Thread(target=self._ros_loop, daemon=True).start()

    def stop(self):
        self._stop.set()

    # ---- real ROS2 path ---------------------------------------------------
    def _ros_loop(self):
        rclpy.init()
        node = Node('mapper_web_status')
        lidar, imu = _RateTracker(), _RateTracker()
        gnss_state = {'fix': 'No fix', 'sats': 0, 't': 0.0}

        node.create_subscription(PointCloud2, '/livox/lidar',
                                 lambda m: lidar.tick(), qos_profile_sensor_data)
        node.create_subscription(Imu, '/livox/imu',
                                 lambda m: imu.tick(), qos_profile_sensor_data)

        def on_fix(m):
            gnss_state['fix'] = _FIX.get(int(m.status.status), 'No fix')
            # NavSatFix has no sat count; expose service bitmask width as a proxy
            gnss_state['sats'] = int(getattr(m.status, 'service', 0)) or gnss_state['sats']
            gnss_state['t'] = time.time()
        node.create_subscription(NavSatFix, '/gnss_inertial/navsatfix',
                                 on_fix, qos_profile_sensor_data)

        while not self._stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
            self.s.merge('lidar', ok=lidar.alive(), rate_hz=lidar.hz())
            self.s.merge('imu', ok=imu.alive(), rate_hz=imu.hz())
            gnss_ok = (time.time() - gnss_state['t']) < 3.0
            self.s.merge('gnss', ok=gnss_ok,
                         fix=gnss_state['fix'] if gnss_ok else 'No fix',
                         sats=gnss_state['sats'])
            self.s.update(connected=lidar.alive())
        node.destroy_node()
        rclpy.shutdown()

    # ---- simulator (dev box) ---------------------------------------------
    def _sim_loop(self):
        self.s.update(connected=True)
        self.s.merge('lidar', ok=True, rate_hz=10.0)
        self.s.merge('imu', ok=True, rate_hz=200.0)
        self.s.merge('gnss', ok=True, fix='RTK Fixed', sats=28)
        self.s.merge('device', work_mode='Working Normally', pps='No PPS',
                     temperature='Normal', voltage='Normal', motor='Normal',
                     dust='Clean', service_life='OK', firmware='11.08.0006')
        while not self._stop.is_set():
            time.sleep(0.5)
