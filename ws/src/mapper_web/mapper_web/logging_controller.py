# Logging controller - the one brain that starts and stops a mapping run.
#
# Both the web dashboard and the GPIO button call THIS, so the two can never
# disagree about whether we're recording. A run is the existing mapping bringup
# launched as a child process; stopping sends SIGINT so livox_mapping saves its
# .pcd (+ .geo.txt) exactly as it does on Ctrl-C, then we move it onto the USB.
import glob
import os
import shutil
import signal
import subprocess
import threading
import time

from . import state as st


def _duration(started):
    """' (2m 14s)' for a run that began at `started`, or '' if unknown."""
    if not started:
        return ''
    secs = int(time.time() - started)
    if secs < 0:
        return ''
    return ' (%dm %02ds)' % (secs // 60, secs % 60)


class LoggingController:
    def __init__(self, shared, usb, led=None, simulate=False,
                 workspace='/opt/mapper/ws', mount_point='/media/log',
                 require_rtk=True, on_fail='wait', require_gps_time=True):
        self.s = shared
        self.usb = usb
        self.led = led
        self.simulate = simulate
        self.workspace = workspace
        self.mount_point = mount_point
        self.require_rtk = require_rtk        # gate active logging on an RTK fix
        self.on_fail = on_fail                # 'wait' (retry) or 'abort'
        # gate logging on satellite time being live, not the computer clock
        self.require_gps_time = require_gps_time
        self._proc = None
        self._source = 'dashboard'
        self._lock = threading.Lock()
        # Wired by the LiDAR control link once the forked driver is running: a
        # callable(dict) -> (ok, msg) that pushes settings to the Avia via SDK.
        self._config_sink = None

    def is_logging(self):
        return self.s.get('logging_state') in (st.INITIAL, st.ACTIVE)

    # ---- start ------------------------------------------------------------
    def start(self, source='dashboard'):
        with self._lock:
            if self.is_logging():
                return False, 'already logging'
            # Who pressed it matters on a unit you drive from a button and
            # then walk away from.
            self._source = source
            # Phase 1: initial data logging - spin the head up, run checks.
            self.s.update(logging_state=st.INITIAL,
                          log_message='Initial data logging - spinning up + checks')
            if self.led:
                self.led.set_logging(True)

        threading.Thread(target=self._start_sequence, daemon=True).start()
        return True, 'starting'

    # Work modes in which the laser is not running, so no data will ever
    # arrive however long we wait for it.
    ASLEEP_MODES = ('Power Saving', 'Standby')

    def _wake_lidar(self):
        """If the laser is parked, ask for it back. Returns True if we asked.

        The client parks it in Power Saving between scans - a normal thing to
        do, and it works. But pressing START while it is parked used to sit
        through the whole 60 s check window waiting for data that could not
        possibly arrive, and then abort with "LiDAR not ready". Pressing START
        LOGGING is not an ambiguous request; wake it.
        """
        mode = self.s.get('device').get('work_mode', 'Unknown')
        if mode not in self.ASLEEP_MODES:
            return False
        ok, _msg = self.apply_lidar_config({'work_mode': 'Working Normally'})
        self.s.add_event('Waking the LiDAR from %s' % mode)
        return ok

    def _start_sequence(self):
        # Wait for the sensors to be alive (and, if required, an RTK fix) before
        # we commit to recording, so we never save a bad run.
        #
        # A wake takes as long as it takes: the driver only restarts sampling
        # once the device reports it has REACHED Normal, and the client's log
        # has that landing anywhere from 16 s to nearly two minutes after the
        # command. So give it a window that can actually accommodate it rather
        # than failing a request we just issued ourselves.
        woke = self._wake_lidar()
        if not self._data_checks(timeout_s=180 if woke else 60):
            return   # _data_checks handles abort messaging
        if not self._launch():
            return
        self.s.update(logging_state=st.ACTIVE, record_started=time.time(),
                      log_message='Active logging - recording to USB')
        self.s.add_event('Recording started (%s)' % self._source)

    def _data_checks(self, timeout_s=60):
        deadline = time.time() + timeout_s
        while True:
            usb_ok = self.simulate or self.usb.healthy_for_logging()
            lidar_ok = self.s.get('lidar')['ok'] or self.simulate
            imu_ok = self.s.get('imu')['ok'] or self.simulate
            gnss = self.s.get('gnss')
            gnss_ok = (not self.require_rtk) or gnss['fix'].lower().startswith('rtk') or self.simulate
            # Scans must carry satellite time, not the computer clock. The
            # mapper's fallback to the computer clock is silent, so without
            # this check a run could be recorded on a wrong clock and look
            # perfectly normal until the data was analysed.
            tsync_ok = ((not self.require_gps_time)
                        or self.s.get('time_sync')['ok'] or self.simulate)

            if usb_ok and lidar_ok and imu_ok and gnss_ok and tsync_ok:
                return True

            missing = []
            if not usb_ok: missing.append('USB')
            if not lidar_ok: missing.append('LiDAR')
            if not imu_ok: missing.append('IMU')
            if not gnss_ok: missing.append('RTK fix')
            if not tsync_ok: missing.append('GPS time')
            msg = 'Waiting for: ' + ', '.join(missing)

            if self.on_fail == 'abort' or time.time() > deadline:
                self.s.update(logging_state=st.IDLE,
                              log_message='Start aborted - ' + ', '.join(missing) + ' not ready')
                self.s.add_event('Start aborted - no ' + ', '.join(missing))
                if self.led:
                    self.led.error_blink()
                    self.led.set_logging(False)
                return False

            self.s.update(log_message=msg)
            time.sleep(1.0)

    def _launch(self):
        if self.simulate:
            self._proc = 'SIM'
            self.s.merge('lidar', ok=True, rate_hz=10.0)
            return True
        # The dashboard is itself launched from a sourced workspace, so the
        # child inherits ROS_DISTRO / AMENT_PREFIX_PATH and can find ros2 and
        # the bringup package directly - no hardcoded workspace path needed.
        # If --workspace is given and ros2 isn't already on PATH, source it.
        prefix = ''
        if self.workspace and not shutil.which('ros2'):
            prefix = 'source {ws}/install/setup.bash && '.format(ws=self.workspace)
        # write to wherever the USB is actually mounted (auto-detected)
        self._active_dir = self.usb.logging_dir() or self.mount_point or os.getcwd()
        cmd = (prefix +
               'ros2 launch livox_hp_mapping_bringup mapping.launch.py '
               'rviz:=false use_gps_time:=true map_file_path:={mp}'
               ).format(mp=self._active_dir)
        try:
            self._proc = subprocess.Popen(
                ['bash', '-c', cmd], env=os.environ.copy(), preexec_fn=os.setsid)
            return True
        except OSError as e:
            self.s.update(logging_state=st.IDLE, log_message='Launch failed: ' + str(e))
            self.s.add_event('Launch failed: %s' % e)
            if self.led:
                self.led.set_logging(False)
            return False

    # ---- stop -------------------------------------------------------------
    def stop(self, source='dashboard'):
        with self._lock:
            if not self.is_logging():
                return False, 'not logging'
            self._source = source
            self.s.update(logging_state=st.STOPPING,
                          log_message='Stopping - saving map + spinning down')
        threading.Thread(target=self._stop_sequence, daemon=True).start()
        return True, 'stopping'

    def _stop_sequence(self):
        # SIGINT the mapping so it writes the .pcd exactly like a Ctrl-C.
        if self._proc and self._proc != 'SIM':
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
                # Must outlast ros2 launch's own escalation inside this group
                # (SIGINT, then SIGTERM after sigterm_timeout, then SIGKILL
                # after sigkill_timeout - 45 s each in mapping.launch.py).
                # Killing the group at 30 s would destroy the very save those
                # timeouts exist to allow, which is the failure this pair of
                # numbers was raised to fix.
                self._proc.wait(timeout=100)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._proc = None

        last = self._collect_map()
        # TODO(next slice): command the LiDAR to Standby (spin down) here via
        # the Livox control link.
        # Safely eject the USB so it can be pulled.
        if not self.simulate:
            self.usb.eject()
        # Read the start time BEFORE it is cleared, so the entry can say how
        # long the run actually was - the one number you want afterwards.
        began = self.s.get('record_started')
        self.s.update(logging_state=st.IDLE, record_started=0.0,
                      last_map=last,
                      log_message='Idle - saved ' + (last or 'no map') + ', USB safe to remove')
        if last:
            self.s.add_event('Saved %s%s' % (last, _duration(began)))
        else:
            self.s.add_event('Stopped - no map file was written')
        if self.led:
            self.led.set_logging(False)

    def _collect_map(self):
        """Move the freshest .pcd (+ .geo.txt) onto the USB, return its name."""
        if self.simulate:
            return time.strftime('livox_map_%Y-%m-%d_%H-%M-%S.pcd')
        dest = getattr(self, '_active_dir', '') or self.usb.logging_dir()
        try:
            pcds = sorted(glob.glob(os.path.join(dest, '*.pcd')) +
                          glob.glob('*.pcd'), key=os.path.getmtime)
            if not pcds:
                return None
            newest = pcds[-1]
            if dest and os.path.dirname(newest) != dest and os.path.isdir(dest):
                for ext in ('', '.geo.txt'):
                    src = newest + ext if ext else newest
                    if os.path.exists(src):
                        shutil.move(src, os.path.join(dest, os.path.basename(src)))
                newest = os.path.join(dest, os.path.basename(newest))
            return os.path.basename(newest)
        except OSError:
            return None

    # ---- LiDAR config -----------------------------------------------------
    def apply_lidar_config(self, cfg):
        """Push echo/IMU/scan/coordinate/sensitivity/work-mode to the Avia.

        The actual SDK calls live in the forked livox driver (the control
        link). When that's running it registers a sink via set_config_sink();
        until then we honestly report that the values are saved and will apply
        once the control link is up, rather than pretending the device changed.
        """
        if self._config_sink is not None:
            try:
                return self._config_sink(cfg)
            except Exception as e:               # never crash the request
                return False, 'LiDAR did not accept config: ' + str(e)
        return True, ('saved - applies once the LiDAR control link is running '
                      'on the device')

    def set_config_sink(self, fn):
        self._config_sink = fn

    # ---- toggle (button) --------------------------------------------------
    def toggle(self, source='button'):
        return (self.stop(source) if self.is_logging()
                else self.start(source))
