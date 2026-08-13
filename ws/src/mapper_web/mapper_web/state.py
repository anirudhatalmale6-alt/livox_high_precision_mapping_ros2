# Shared, thread-safe state for the mapper web dashboard.
#
# One MapperState instance is the single source of truth. The status monitor,
# logging controller, USB manager and GPIO button all update it; the web server
# (REST + SSE) reads it. A single lock keeps it consistent across threads.
import copy
import threading
import time


# High-level logging lifecycle, mirrored 1:1 in the dashboard.
IDLE = 'idle'                       # nothing running, LiDAR in power-saving
INITIAL = 'initial_data_logging'   # laser spinning up + running data checks
ACTIVE = 'active_logging'          # recording to USB
STOPPING = 'stopping'              # spin down, save map, eject USB


class MapperState:
    def __init__(self):
        self._lock = threading.Lock()
        self._d = {
            'connected': False,          # LiDAR reachable
            'logging_state': IDLE,
            'log_message': 'Idle',       # human line shown under the buttons
            'last_map': None,            # filename of the most recent .pcd
            'record_started': 0.0,       # epoch when active logging began

            # live sensor health (updated by the status monitor)
            'lidar': {'ok': False, 'rate_hz': 0.0},
            'imu': {'ok': False, 'rate_hz': 0.0},
            'gnss': {'ok': False, 'fix': 'No fix', 'sats': 0},
            # Which clock the scan is stamped with. The mapper falls back to
            # the computer clock when the UM982 is not publishing a satellite
            # time offset, and that fallback is silent - so it is surfaced here
            # and checked before logging starts.
            'time_sync': {'ok': False, 'source': 'Computer clock',
                          'offset_s': 0.0},

            # LiDAR device status (read-only indicators from the driver)
            'device': {
                'work_mode': 'Unknown',
                'pps': 'Unknown',
                'temperature': 'Unknown',
                'voltage': 'Unknown',
                'motor': 'Unknown',
                'dust': 'Unknown',
                'service_life': 'Unknown',
                'firmware': '-',
            },

            # editable configuration (last applied)
            'config': {
                'echo_type': 'Single - First Return',
                'work_mode': 'Working Normally',
                'imu_freq': '200 Hz',
                'scan_mode': 'Non-repetitive (Circular)',
                'coordinate': 'Cartesian',
                'high_sensitivity': 'Enabled',
                # correction-data source for the UM982 RTK (GNSS bringup)
                'rtk_source': 'NTRIP',
            },

            # USB / logging target
            'usb': {
                'present': False,
                'mounted': False,
                'label': '-',
                'total_gb': 0.0,
                'free_gb': 0.0,
                'free_pct': 0,
                'path': '',
            },

            # button / LED feedback
            'button_hint': '',           # transient "logging armed" / "shutdown armed"
            'updated': time.time(),
        }

    def snapshot(self):
        """Return a deep copy safe to serialise for the web layer."""
        with self._lock:
            self._d['updated'] = time.time()
            if self._d['logging_state'] == ACTIVE and self._d['record_started']:
                self._d['elapsed_s'] = int(time.time() - self._d['record_started'])
            else:
                self._d['elapsed_s'] = 0
            return copy.deepcopy(self._d)

    def update(self, **kwargs):
        with self._lock:
            for k, v in kwargs.items():
                self._d[k] = v

    def merge(self, key, **kwargs):
        """Update a nested dict, e.g. merge('lidar', ok=True, rate_hz=10)."""
        with self._lock:
            self._d[key].update(kwargs)

    def get(self, key):
        with self._lock:
            return copy.deepcopy(self._d[key])
