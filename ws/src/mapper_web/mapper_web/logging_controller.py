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


class LoggingController:
    def __init__(self, shared, usb, led=None, simulate=False,
                 workspace='/opt/mapper/ws', mount_point='/media/log',
                 require_rtk=True, on_fail='wait'):
        self.s = shared
        self.usb = usb
        self.led = led
        self.simulate = simulate
        self.workspace = workspace
        self.mount_point = mount_point
        self.require_rtk = require_rtk        # gate active logging on an RTK fix
        self.on_fail = on_fail                # 'wait' (retry) or 'abort'
        self._proc = None
        self._lock = threading.Lock()

    def is_logging(self):
        return self.s.get('logging_state') in (st.INITIAL, st.ACTIVE)

    # ---- start ------------------------------------------------------------
    def start(self):
        with self._lock:
            if self.is_logging():
                return False, 'already logging'
            # Phase 1: initial data logging - spin the head up, run checks.
            self.s.update(logging_state=st.INITIAL,
                          log_message='Initial data logging - spinning up + checks')
            self.s.merge('device', work_mode='Working Normally')
            if self.led:
                self.led.set_logging(True)

        threading.Thread(target=self._start_sequence, daemon=True).start()
        return True, 'starting'

    def _start_sequence(self):
        # Wait for the sensors to be alive (and, if required, an RTK fix) before
        # we commit to recording, so we never save a bad run.
        if not self._data_checks():
            return   # _data_checks handles abort messaging
        if not self._launch():
            return
        self.s.update(logging_state=st.ACTIVE, record_started=time.time(),
                      log_message='Active logging - recording to USB')

    def _data_checks(self, timeout_s=60):
        deadline = time.time() + timeout_s
        while True:
            usb_ok = self.simulate or self.usb.healthy_for_logging()
            lidar_ok = self.s.get('lidar')['ok'] or self.simulate
            imu_ok = self.s.get('imu')['ok'] or self.simulate
            gnss = self.s.get('gnss')
            gnss_ok = (not self.require_rtk) or gnss['fix'].lower().startswith('rtk') or self.simulate

            if usb_ok and lidar_ok and imu_ok and gnss_ok:
                return True

            missing = []
            if not usb_ok: missing.append('USB')
            if not lidar_ok: missing.append('LiDAR')
            if not imu_ok: missing.append('IMU')
            if not gnss_ok: missing.append('RTK fix')
            msg = 'Waiting for: ' + ', '.join(missing)

            if self.on_fail == 'abort' or time.time() > deadline:
                self.s.update(logging_state=st.IDLE,
                              log_message='Start aborted - ' + ', '.join(missing) + ' not ready')
                self.s.merge('device', work_mode='Power Saving')
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
        cmd = (prefix +
               'ros2 launch livox_hp_mapping_bringup mapping.launch.py '
               'rviz:=false use_gps_time:=true map_file_path:={mp}'
               ).format(mp=self.mount_point)
        try:
            self._proc = subprocess.Popen(
                ['bash', '-c', cmd], env=os.environ.copy(), preexec_fn=os.setsid)
            return True
        except OSError as e:
            self.s.update(logging_state=st.IDLE, log_message='Launch failed: ' + str(e))
            if self.led:
                self.led.set_logging(False)
            return False

    # ---- stop -------------------------------------------------------------
    def stop(self):
        with self._lock:
            if not self.is_logging():
                return False, 'not logging'
            self.s.update(logging_state=st.STOPPING,
                          log_message='Stopping - saving map + spinning down')
        threading.Thread(target=self._stop_sequence, daemon=True).start()
        return True, 'stopping'

    def _stop_sequence(self):
        # SIGINT the mapping so it writes the .pcd exactly like a Ctrl-C.
        if self._proc and self._proc != 'SIM':
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
                self._proc.wait(timeout=30)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._proc = None

        last = self._collect_map()
        # Spin the head back down into power saving.
        self.s.merge('device', work_mode='Power Saving')
        # Safely eject the USB so it can be pulled.
        if not self.simulate:
            self.usb.eject()
        self.s.update(logging_state=st.IDLE, record_started=0.0,
                      last_map=last,
                      log_message='Idle - saved ' + (last or 'no map') + ', USB safe to remove')
        if self.led:
            self.led.set_logging(False)

    def _collect_map(self):
        """Move the freshest .pcd (+ .geo.txt) onto the USB, return its name."""
        if self.simulate:
            return time.strftime('livox_map_%Y-%m-%d_%H-%M-%S.pcd')
        try:
            pcds = sorted(glob.glob(os.path.join(self.mount_point, '*.pcd')) +
                          glob.glob('*.pcd'), key=os.path.getmtime)
            if not pcds:
                return None
            newest = pcds[-1]
            if os.path.dirname(newest) != self.mount_point and os.path.ismount(self.mount_point):
                for ext in ('', '.geo.txt'):
                    src = newest + ext if ext else newest
                    if os.path.exists(src):
                        shutil.move(src, os.path.join(self.mount_point, os.path.basename(src)))
            return os.path.basename(newest)
        except OSError:
            return None

    # ---- toggle (button) --------------------------------------------------
    def toggle(self):
        return self.stop() if self.is_logging() else self.start()
