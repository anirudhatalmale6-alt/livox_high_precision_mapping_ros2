# Mapper web dashboard - HTTP server (Python standard library only).
#
# Deliberately no Flask/FastAPI: a field Pi shouldn't need extra pip installs.
# ThreadingHTTPServer + Server-Sent Events gives live status with zero deps.
#
# Routes
#   GET  /                     dashboard page
#   GET  /app.js /style.css    static assets
#   GET  /api/status           one JSON snapshot
#   GET  /api/events           SSE stream of snapshots (~2 Hz)
#   POST /api/logging/start    start a run (also the button 3-4s hold)
#   POST /api/logging/stop     stop + save + eject USB
#   POST /api/config           {echo_type, work_mode}
#   POST /api/usb/<check|detach|format>
#   POST /api/system/<restart|shutdown>
import argparse
import json
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .state import MapperState
from .usb import UsbManager
from .led import StatusLed
from .logging_controller import LoggingController
from .status_monitor import StatusMonitor
from .button import PushButton

def _find_web_dir():
    """Locate the web assets whether run from source or a colcon install."""
    candidates = [
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'web')),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, os.path.join(
            get_package_share_directory('mapper_web'), 'web'))
    except Exception:
        pass
    for c in candidates:
        if os.path.isdir(c):
            return c
    return candidates[-1]


WEB_DIR = _find_web_dir()

# Config keys the dashboard can set. LiDAR settings map to Livox SDK calls in the
# driver control link; rtk_source picks the UM982 correction transport.
CONFIG_KEYS = ('echo_type', 'work_mode', 'imu_freq', 'scan_mode',
               'coordinate', 'high_sensitivity', 'rtk_source')


class App:
    """Holds the wired-up components; the handler talks to this."""
    def __init__(self, opts):
        self.opts = opts
        self.state = MapperState()
        # Recent-activity history survives a reboot. Not in simulate mode -
        # a demo should not leave state on the machine it ran on.
        if not opts.simulate:
            self.state.set_events_path(
                os.environ.get('MAPPER_EVENTS_FILE', '/var/lib/mapper/events.json'))
            self.state.set_config_path(
                os.environ.get('MAPPER_CONFIG_FILE',
                               '/var/lib/mapper/lidar_config.json'))
        self.usb = UsbManager(mount_point=opts.mount_point, simulate=opts.simulate)
        self.led = StatusLed(red=opts.led_red, green=opts.led_green,
                             blue=opts.led_blue, mono=opts.led_gpio,
                             shared=self.state)
        self.ctl = LoggingController(
            self.state, self.usb, led=self.led, simulate=opts.simulate,
            workspace=opts.workspace, mount_point=opts.mount_point,
            require_rtk=opts.require_rtk, on_fail=opts.on_fail,
            require_gps_time=opts.require_gps_time)
        self.monitor = StatusMonitor(self.state, simulate=opts.simulate)
        # Wire the LiDAR control link: dashboard APPLY -> LoggingController
        # -> StatusMonitor publishes the command to the forked Livox driver.
        self.ctl.set_config_sink(self.monitor.send_command)
        self.button = PushButton(self.ctl, led=self.led, pin=opts.button_gpio,
                                 simulate=opts.simulate)

    def start(self):
        # Mount the stick at boot rather than waiting for someone to open the
        # dashboard - the button has to work on a unit with no screen attached.
        self.refresh_usb()
        self.monitor.start()
        self._restore_lidar_config()

    # How long to watch the LiDAR after a restore before calling it survivable.
    # The failure we are guarding against takes the driver down within seconds
    # of the command, so this does not need to be long - and it must NOT be, or
    # an ordinary `systemctl restart` in the middle of the window would be
    # counted as a crash and cost the operator their settings.
    RESTORE_PROOF_S = 15.0

    # The setting the Livox driver's own config file uses, and the only value
    # we know cannot take the driver down with it.
    SAFE_ECHO = 'Single - First Return'

    # Seconds between the individual commands of a restore.
    RESTORE_GAP_S = 3.0

    # Never replayed at startup.
    #
    # rtk_source is not a LiDAR setting at all. work_mode is excluded for a
    # sharper reason: sending "Working Normally" makes the driver call
    # LidarSetMode(Normal), which schedules a sampling RESTART - and the
    # client's log shows that restart landing a full two minutes later
    # ("device reached Normal - restarting sampling"). The device is already
    # sampling when we get here, so the command buys nothing and costs a
    # restart landing on top of whatever else we just changed. Restoring
    # Standby or Power Saving would be worse still: the unit would come back
    # from a reboot not scanning, with nothing on screen to say why.
    RESTORE_SKIP = ('rtk_source', 'work_mode')

    def _restore_payload(self):
        """The restore, as a list of one-setting commands.

        Two deliberate reductions, both from the same evidence.

        The client set Double Return by hand at 17:18 and it applied perfectly:
        DataType 2 -> 4, ack 0, and four scans recorded after it. The SAME
        value, replayed by this restore at 17:43 as part of one six-setting
        command, stopped the LiDAR dead. So the value is fine and the batch is
        not - six SDK calls arriving together, one of which restarts sampling.

        So: send only what the operator actually changed away from stock (the
        driver applies coordinate, return mode and IMU rate from its own config
        file at connect anyway), and send them one at a time.
        """
        defaults = MapperState().get('config')
        cfg = self.state.get('config')
        return [{k: v} for k, v in cfg.items()
                if k not in self.RESTORE_SKIP and defaults.get(k) != v]

    def _restore_lidar_config(self):
        """Push the saved LiDAR settings back to the device once it is up.

        The Avia comes up on whatever is in the Livox driver's own config file,
        so a setting chosen on the dashboard is lost at every restart - the
        panel would go on displaying it while the device had reverted. Without
        this the client's "Single - Strongest Return", which visibly produced
        his best scan, would silently be first-return again after a reboot.

        GUARDED, because restoring a setting automatically is only safe if that
        setting is safe. The client is currently able to kill the Livox driver
        by switching return mode. Saved and replayed at every boot, that turns a
        thing he did once into a thing the unit does to itself forever - a
        crash loop that gets worse the more reliable the restore is.

        So: a marker is written before the command goes out and removed once the
        unit has stayed up for RESTORE_PROOF_S afterwards. Finding the marker
        still there at startup means the last restore did not survive, and we
        do not repeat it.

        Runs in the background: the control link needs the driver to be up, and
        nothing here is allowed to delay the dashboard coming online.
        """
        if self.opts.simulate:
            return

        marker = os.environ.get(
            'MAPPER_RESTORE_MARKER', '/var/lib/mapper/restore_in_progress.json')
        strikes = self._read_marker(marker)

        if strikes:
            # Last boot's restore did not survive. Do not send it again.
            self._clear_marker(marker)
            if strikes >= 2:
                # Reverting the echo type alone did not help, so something else
                # in there is doing it. Drop the lot back to defaults.
                self.state.merge('config', **MapperState().get('config'))
                self.state.save_config()
                self.state.add_event('LiDAR settings reset - restore kept failing')
            else:
                bad = self.state.get('config').get('echo_type')
                self.state.merge('config', echo_type=self.SAFE_ECHO)
                self.state.save_config()
                self.state.add_event('Reverted echo type - %s crashed the LiDAR'
                                     % bad)
            return

        def worker():
            steps = self._restore_payload()
            if not steps:
                return          # everything is already at stock; nothing to do
            deadline = time.time() + 120
            while time.time() < deadline:
                # Straight to the control link, not through
                # apply_lidar_config(): that reports success when the link is
                # not up yet ("saved - applies once ... running"), which is the
                # honest answer to a person pressing APPLY but would end this
                # retry loop before anything reached the device.
                self._write_marker(marker, strikes + 1)
                ok, msg = self.monitor.send_command(steps[0])
                if ok:
                    # One command at a time, with a gap. See _restore_payload.
                    for extra in steps[1:]:
                        time.sleep(self.RESTORE_GAP_S)
                        self.monitor.send_command(extra)
                    self.state.add_event('LiDAR settings restored')
                    # Judge the restore on the LiDAR itself, not on the clock.
                    # "Did the unit stay up" would also be satisfied by the
                    # operator restarting the service for their own reasons,
                    # and would then cost them their settings for nothing.
                    # "Is the LiDAR still streaming" measures the actual thing.
                    time.sleep(self.RESTORE_PROOF_S)
                    if self.state.get('lidar')['ok']:
                        self._clear_marker(marker)
                    else:
                        self.state.add_event(
                            'LiDAR stopped right after applying settings')
                    return
                self._clear_marker(marker)
                time.sleep(3.0)
            self.state.add_event('Could not restore LiDAR settings')

        threading.Thread(target=worker, daemon=True).start()

    # ---- restore marker ---------------------------------------------------
    # Deliberately its own small file rather than a key in the settings file:
    # it must be possible to clear the marker without rewriting the settings,
    # and a half-written settings file must never be able to lose them.
    @staticmethod
    def _read_marker(path):
        try:
            with open(path) as f:
                d = json.load(f)
            return int(d.get('strikes', 0)) if isinstance(d, dict) else 0
        except (OSError, ValueError, TypeError):
            return 0

    @staticmethod
    def _write_marker(path, strikes):
        try:
            d = os.path.dirname(path)
            if d:
                os.makedirs(d, exist_ok=True)
            with open(path, 'w') as f:
                json.dump({'strikes': int(strikes), 't': time.time()}, f)
        except (OSError, ValueError, TypeError):
            pass

    @staticmethod
    def _clear_marker(path):
        try:
            os.remove(path)
        except OSError:
            pass

    def refresh_usb(self):
        s = self.usb.status()
        if self.usb.ensure_mounted(s):
            s = self.usb.status()      # it tried; report the result, not the guess
        self.state.update(usb=s)


class Handler(BaseHTTPRequestHandler):
    app = None   # set on the class before serving

    def log_message(self, *a):
        pass      # keep the console clean

    # ---- GET --------------------------------------------------------------
    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/':
            return self._file('index.html', 'text/html')
        if path == '/app.js':
            return self._file('app.js', 'application/javascript')
        if path == '/style.css':
            return self._file('style.css', 'text/css')
        if path == '/api/status':
            self.app.refresh_usb()
            return self._json(self.app.state.snapshot())
        if path == '/api/events':
            return self._sse()
        return self._err(404, 'not found')

    # ---- POST -------------------------------------------------------------
    def do_POST(self):
        path = self.path.split('?')[0]
        body = self._read_body()
        try:
            if path == '/api/logging/start':
                ok, msg = self.app.ctl.start()
            elif path == '/api/logging/stop':
                ok, msg = self.app.ctl.stop()
            elif path == '/api/config':
                self.app.state.merge('config', **{k: body[k] for k in
                                     CONFIG_KEYS if k in body})
                ok, msg = self._apply_config(body)
                # Saved even when the device rejected it: the panel shows what
                # was chosen, and the panel must not go back to disagreeing
                # with itself after a restart.
                self.app.state.save_config()
            elif path == '/api/usb/check':
                self.app.refresh_usb(); ok, msg = True, 'checked'
            elif path == '/api/usb/attach':
                ok, msg = self.app.usb.mount(); self.app.refresh_usb()
            elif path == '/api/usb/detach':
                ok, msg = self.app.usb.eject(); self.app.refresh_usb()
            elif path == '/api/usb/format':
                ok, msg = self.app.usb.format(); self.app.refresh_usb()
            elif path == '/api/system/restart-sensors':
                ok, msg = self._restart_sensors()
            elif path == '/api/system/restart':
                ok, msg = self._system('reboot')
            elif path == '/api/system/shutdown':
                ok, msg = self._system('shutdown')
            else:
                return self._err(404, 'not found')
        except Exception as e:               # never 500 the field unit
            ok, msg = False, str(e)
        return self._json({'ok': ok, 'message': msg})

    # ---- helpers ----------------------------------------------------------
    def _apply_config(self, body):
        # Two kinds of settings arrive here:
        #
        #  LiDAR settings -> pushed to the Avia via the Livox SDK (driver
        #  control link). Each maps to one SDK call:
        #    echo_type        -> LidarSetPointCloudReturnMode
        #    work_mode        -> LidarSetMode (Normal / PowerSaving / Standby)
        #    imu_freq         -> LidarSetImuPushFrequency (0 / 200 Hz)
        #    scan_mode        -> LidarSetScanPattern (non-repetitive / repetitive)
        #    coordinate       -> SetCartesianCoordinate / SetSphericalCoordinate
        #    high_sensitivity -> LidarEnableHighSensitivity / ...Disable...
        #
        #  rtk_source (Serial / NTRIP / MavLink) is a GNSS-bringup choice, not a
        #  LiDAR command - we persist it so the next system restart brings the
        #  UM982 up on that transport.
        if 'rtk_source' in body:
            ok, note = self._persist_rtk_source(body['rtk_source'])
            if not ok:
                return False, note
            return True, 'RTK source set to ' + str(body['rtk_source']) + \
                ' - applies on next restart'
        if self.app.opts.simulate:
            return True, 'config applied (sim)'
        # Live: hand the LiDAR settings to the control link.
        return self.app.ctl.apply_lidar_config(
            {k: body[k] for k in CONFIG_KEYS if k in body and k != 'rtk_source'})

    def _persist_rtk_source(self, source):
        """Save the chosen RTK transport where field.launch can read it."""
        valid = ('NTRIP', 'Serial', 'MavLink')
        if source not in valid:
            return False, 'unknown RTK source: ' + str(source)
        if self.app.opts.simulate:
            return True, 'ok (sim)'
        try:
            path = os.environ.get('MAPPER_RTK_SOURCE_FILE',
                                  '/etc/mapper/rtk_source')
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w') as f:
                f.write(source + '\n')
            return True, 'ok'
        except OSError as e:
            return False, 'could not save RTK source (' + str(e) + ')'

    def _restart_sensors(self):
        """Restart the whole field service - drivers and this dashboard.

        Putting the LiDAR into Power Saving stops the laser, and the command to
        bring it back has to travel the same control link. If that link is down
        the LiDAR cannot be woken from the dashboard at all, and recovery meant
        an SSH session or a full reboot of the machine. This is the smaller
        hammer.

        --no-block matters: without it systemd would stop this very process
        while it is still trying to write the HTTP response, and the browser
        would show a failure for something that actually worked.
        """
        if self.app.opts.simulate:
            return True, 'sensors restarted (sim)'
        import subprocess
        cmd = ['systemctl', 'restart', '--no-block', 'mapper-field']
        if os.geteuid() != 0:
            cmd = ['sudo', '-n'] + cmd
        try:
            subprocess.Popen(cmd)
        except OSError as e:
            return False, 'could not restart the sensors: ' + str(e)
        return True, ('restarting the drivers - the dashboard will drop for '
                      'about 30 s and come back on its own')

    def _system(self, action):
        if self.app.opts.simulate:
            return True, action + ' (sim)'
        # Park the laser before the machine goes away. Not on a reboot: the
        # unit is back in half a minute and the driver brings the LiDAR up
        # again itself, so parking would only risk it coming back asleep.
        if action != 'reboot':
            self.app.ctl.park_lidar()
        from .power import power_off
        return power_off(reboot=(action == 'reboot'))

    def _read_body(self):
        n = int(self.headers.get('Content-Length', 0) or 0)
        if not n:
            return {}
        raw = self.rfile.read(n)
        try:
            return json.loads(raw or b'{}')
        except ValueError:
            return {}

    def _file(self, name, ctype):
        try:
            with open(os.path.join(WEB_DIR, name), 'rb') as f:
                data = f.read()
        except OSError:
            return self._err(404, 'missing ' + name)
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _json(self, obj):
        data = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _sse(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        try:
            while True:
                self.app.refresh_usb()
                payload = json.dumps(self.app.state.snapshot())
                self.wfile.write(('data: ' + payload + '\n\n').encode())
                self.wfile.flush()
                time.sleep(0.5)
        except (BrokenPipeError, ConnectionResetError):
            return

    def _err(self, code, msg):
        self.send_response(code)
        self.send_header('Content-Type', 'text/plain')
        self.end_headers()
        self.wfile.write(msg.encode())


def build_parser():
    p = argparse.ArgumentParser(description='Livox mapper web dashboard')
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--workspace', default='/opt/mapper/ws')
    p.add_argument('--mount-point', default='')
    # Pins are strings, not ints: a plain number is a Raspberry Pi BCM pin,
    # while CHIP:LINE (e.g. 0:26) addresses a kernel GPIO line and works on any
    # board. BCM numbering is a Raspberry Pi concept and means nothing on an
    # Orange Pi, so a second form is needed rather than a different number.
    p.add_argument('--button-gpio', default='26')
    # RGB status LED (client's wiring: R=16 G=20 B=21). Use --led-gpio instead
    # for a single-colour LED.
    p.add_argument('--led-red', default='16')
    p.add_argument('--led-green', default='20')
    p.add_argument('--led-blue', default='21')
    p.add_argument('--led-gpio', default=None)
    # Gate logging on an RTK fix. Default OFF so you can test/log on a plain GPS
    # fix; turn ON for survey work where cm accuracy is required.
    p.add_argument('--require-rtk', action='store_true', default=False)
    p.add_argument('--no-require-rtk', dest='require_rtk', action='store_false')
    # Refuse to start a scan on the computer clock. On by default: the mapper's
    # fallback from satellite time to the computer clock is silent, so without
    # this a run can be recorded against a wrong clock and look entirely normal.
    p.add_argument('--require-gps-time', action='store_true', default=True)
    p.add_argument('--no-require-gps-time', dest='require_gps_time',
                   action='store_false')
    p.add_argument('--on-fail', choices=['wait', 'abort'], default='wait')
    p.add_argument('--simulate', action='store_true',
                   help='run with no ROS2/GPIO/USB (dev + demo)')
    return p


def main(argv=None):
    import sys
    raw = list(sys.argv[1:] if argv is None else argv)
    # ros2 launch appends "--ros-args -r __node:=..." - strip it so argparse
    # (and this non-rclpy main process) doesn't choke on ROS remap args.
    if '--ros-args' in raw:
        raw = raw[:raw.index('--ros-args')]
    opts, _unknown = build_parser().parse_known_args(raw)
    if os.environ.get('MAPPER_SIMULATE') == '1':
        opts.simulate = True
    app = App(opts)
    app.start()
    Handler.app = app
    srv = ThreadingHTTPServer((opts.host, opts.port), Handler)
    mode = 'SIMULATE' if opts.simulate else 'LIVE'
    print('Mapper dashboard [{}] on http://{}:{}'.format(mode, opts.host, opts.port))

    # Come down on SIGTERM as well as SIGINT.
    #
    # serve_forever() only ever unwound on KeyboardInterrupt, so a plain
    # `systemctl stop` left this process sitting there until systemd lost
    # patience and SIGKILLed it 90 seconds later - and a stop that does not
    # finish is how two dashboards ended up alive at the same time, with the
    # older one still holding the GPS serial port.
    def _shutdown(signum, frame):
        # shutdown() must not be called from inside serve_forever()'s own
        # thread or it deadlocks; a handler runs in the main thread, which IS
        # that thread, so hand it to a helper.
        threading.Thread(target=srv.shutdown, daemon=True).start()

    for _sig in (signal.SIGTERM, signal.SIGINT):
        try:
            signal.signal(_sig, _shutdown)
        except (ValueError, OSError):
            pass          # not the main thread - fall back to the old behaviour

    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.led.close()
        app.monitor.stop()
        srv.server_close()


if __name__ == '__main__':
    main()
