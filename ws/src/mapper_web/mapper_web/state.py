# Shared, thread-safe state for the mapper web dashboard.
#
# One MapperState instance is the single source of truth. The status monitor,
# logging controller, USB manager and GPIO button all update it; the web server
# (REST + SSE) reads it. A single lock keeps it consistent across threads.
import copy
import json
import os
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

            # Recent activity, newest first. The status line under the buttons
            # only ever shows what is happening NOW, so a run that started and
            # finished while you were away from the screen left no trace - and
            # on a unit driven by a button, that is most of them.
            'events': [],                # [{t, time, text}], newest first

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

    def add_event(self, text, keep=5):
        """Record one thing that happened, newest first.

        Kept deliberately short: this is the "what did it just do" box, not a
        log file. The service journal is still the place for detail.
        """
        with self._lock:
            now = time.time()
            self._d['events'].insert(0, {
                't': now,
                'time': time.strftime('%H:%M:%S', time.localtime(now)),
                'text': str(text),
            })
            del self._d['events'][keep:]
            events = list(self._d['events'])
        self._save_events(events)

    # ---- persistence ------------------------------------------------------
    # Held in a small file so the box is not empty every time the unit is
    # powered up in the field - which is exactly when you want to see what the
    # last run did. Never allowed to break anything: a unit that cannot write
    # its history still records maps perfectly well.
    def set_events_path(self, path):
        self.events_path = path
        try:
            with open(path) as f:
                loaded = json.load(f)
            if isinstance(loaded, list):
                with self._lock:
                    self._d['events'] = [e for e in loaded if isinstance(e, dict)][:5]
        except (OSError, ValueError):
            pass

    def _save_events(self, events):
        self._write_json(getattr(self, 'events_path', ''), events)

    # The LiDAR settings have to survive a restart too, and for a sharper
    # reason than the event list. The dashboard shows what is in 'config'; the
    # device is brought up from the Livox driver's own config file. Losing the
    # saved settings does not just blank the panel, it makes the panel state
    # something the LiDAR is not - it read "Single - First Return" while the
    # client's chosen "Single - Strongest Return" had quietly been lost at the
    # last restart.
    def set_config_path(self, path):
        self.config_path = path
        try:
            with open(path) as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                with self._lock:
                    for k, v in loaded.items():
                        if k in self._d['config'] and isinstance(v, str):
                            self._d['config'][k] = v
        except (OSError, ValueError):
            pass

    def save_config(self):
        with self._lock:
            cfg = dict(self._d['config'])
        self._write_json(getattr(self, 'config_path', ''), cfg)

    def _write_json(self, path, obj):
        if not path:
            return
        try:
            d = os.path.dirname(path)
            if d:
                os.makedirs(d, exist_ok=True)
            tmp = path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(obj, f)
            os.replace(tmp, path)     # never leave a half-written file behind
        except (OSError, ValueError, TypeError):
            pass
