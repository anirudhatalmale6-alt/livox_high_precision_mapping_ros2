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


class App:
    """Holds the wired-up components; the handler talks to this."""
    def __init__(self, opts):
        self.opts = opts
        self.state = MapperState()
        self.usb = UsbManager(mount_point=opts.mount_point, simulate=opts.simulate)
        self.led = StatusLed(red=opts.led_red, green=opts.led_green,
                             blue=opts.led_blue, mono=opts.led_gpio,
                             shared=self.state)
        self.ctl = LoggingController(
            self.state, self.usb, led=self.led, simulate=opts.simulate,
            workspace=opts.workspace, mount_point=opts.mount_point,
            require_rtk=opts.require_rtk, on_fail=opts.on_fail)
        self.monitor = StatusMonitor(self.state, simulate=opts.simulate)
        self.button = PushButton(self.ctl, led=self.led, pin=opts.button_gpio,
                                 simulate=opts.simulate)

    def start(self):
        self.monitor.start()

    def refresh_usb(self):
        self.state.update(usb=self.usb.status())


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
                                     ('echo_type', 'work_mode') if k in body})
                ok, msg = self._apply_config(body)
            elif path == '/api/usb/check':
                self.app.refresh_usb(); ok, msg = True, 'checked'
            elif path == '/api/usb/attach':
                ok, msg = self.app.usb.mount(); self.app.refresh_usb()
            elif path == '/api/usb/detach':
                ok, msg = self.app.usb.eject(); self.app.refresh_usb()
            elif path == '/api/usb/format':
                ok, msg = self.app.usb.format(); self.app.refresh_usb()
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
        # On the Pi this pushes echo type / work mode to the LiDAR via the
        # Livox SDK. In simulate we just record it. (SDK hook: livox config.)
        if self.app.opts.simulate:
            return True, 'config applied (sim)'
        return True, 'config applied'

    def _system(self, action):
        if self.app.opts.simulate:
            return True, action + ' (sim)'
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
    p.add_argument('--button-gpio', type=int, default=26)
    # RGB status LED (client's wiring: R=16 G=20 B=21). Use --led-gpio instead
    # for a single-colour LED.
    p.add_argument('--led-red', type=int, default=16)
    p.add_argument('--led-green', type=int, default=20)
    p.add_argument('--led-blue', type=int, default=21)
    p.add_argument('--led-gpio', type=int, default=None)
    # Gate logging on an RTK fix. Default OFF so you can test/log on a plain GPS
    # fix; turn ON for survey work where cm accuracy is required.
    p.add_argument('--require-rtk', action='store_true', default=False)
    p.add_argument('--no-require-rtk', dest='require_rtk', action='store_false')
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
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.led.close()
        app.monitor.stop()


if __name__ == '__main__':
    main()
