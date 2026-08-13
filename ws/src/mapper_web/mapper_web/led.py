# Status LED - field feedback for the screenless button.
#
# Supports an RGB LED (recommended) or a single-colour LED. Colour encodes the
# system state so you can read the unit at a glance in the field:
#
#   solid GREEN    : idle and ready to log (USB mounted, sensors alive)
#   solid RED      : not ready (no USB, or LiDAR not streaming)
#   blink YELLOW   : initial data logging / stopping (checks, spin up/down)
#   blink BLUE     : ACTIVE logging (recording)
#   double GREEN   : "logging armed" - you've held the button ~3 s
#   double RED     : "shutdown armed" - you've held the button ~8 s
#   fast RED       : a check failed / start aborted
#
# RGB default wiring (client's): GPIO 16 = Red, 20 = Green, 21 = Blue.
# Uses gpiozero if present; otherwise a no-op stub so the service runs off-Pi.
import threading
import time

from . import gpio as _gpio

_HAVE_GPIO = _gpio.available()

from . import state as st

# colours as (r, g, b), each 0..1
OFF = (0, 0, 0)
GREEN = (0, 1, 0)
RED = (1, 0, 0)
BLUE = (0, 0, 1)
YELLOW = (1, 1, 0)


def color_for_state(snap):
    """Pure decision: given a state snapshot, return (color, blink?).

    Kept separate from the hardware so it can be unit-tested without a Pi.
    """
    ls = snap.get('logging_state')
    if ls == st.ACTIVE:
        return BLUE, True
    if ls in (st.INITIAL, st.STOPPING):
        return YELLOW, True
    # idle: green only if we're actually ready to start a run
    usb = snap.get('usb', {})
    ready = usb.get('mounted') and usb.get('free_gb', 0) >= 1 and snap.get('lidar', {}).get('ok')
    return (GREEN, False) if ready else (RED, False)


class StatusLed:
    def __init__(self, red=None, green=None, blue=None, mono=None, shared=None):
        self.shared = shared
        self.rgb = None
        self.mono = None
        self.enabled = False
        self._flash_until = 0.0
        self._flash_color = RED
        self._stop = threading.Event()

        # Importing gpiozero is NOT proof that GPIO works. On a board it does not
        # recognise (Orange Pi, generic SBCs) the import succeeds and the pin
        # object then raises BadPinFactory. That used to escape and kill the
        # whole dashboard on startup - no web page at all, because of an LED.
        # The LED is a nicety; the dashboard is not. Never let it take the page
        # down.
        # Three separate on/off lines rather than gpiozero's RGBLED, so the
        # same code drives a Pi (BCM numbers) and any other board (CHIP:LINE).
        try:
            if _HAVE_GPIO and red is not None and green is not None and blue is not None:
                self.rgb = [_gpio.OutputPin(red), _gpio.OutputPin(green),
                            _gpio.OutputPin(blue)]
                self.enabled = True
            elif _HAVE_GPIO and mono is not None:
                self.mono = _gpio.OutputPin(mono)
                self.enabled = True
        except Exception as e:
            self.rgb = self.mono = None
            self.enabled = False
            print('[mapper_web] status LED disabled (no usable GPIO on this '
                  'board): %s' % e, flush=True)
            print('[mapper_web]   on a non-Raspberry-Pi board give the pins as '
                  'CHIP:LINE (e.g. --led-red 0:26); run scripts/list_gpio.sh '
                  'to find them.', flush=True)

        if self.enabled:
            threading.Thread(target=self._loop, daemon=True).start()

    # ---- momentary cues (button / abort) ---------------------------------
    def armed_flash(self):
        self._flash(GREEN, 0.6)

    def shutdown_flash(self):
        self._flash(RED, 0.8)

    def error_blink(self):
        self._flash(RED, 1.2)

    def set_logging(self, on):
        # Kept for API compatibility; the loop reads state directly.
        pass

    def _flash(self, color, dur):
        self._flash_color = color
        self._flash_until = time.time() + dur

    # ---- render loop ------------------------------------------------------
    def _loop(self):
        phase = False
        while not self._stop.is_set():
            phase = not phase
            now = time.time()
            if now < self._flash_until:
                # fast momentary blink of the flash colour
                self._write(self._flash_color if phase else OFF)
                time.sleep(0.1)
                continue
            snap = self.shared.snapshot() if self.shared else {}
            color, blink = color_for_state(snap) if snap else (OFF, False)
            self._write(color if (not blink or phase) else OFF)
            time.sleep(0.4 if blink else 0.2)

    def _write(self, color):
        try:
            if self.rgb is not None:
                # colours here are only ever 0 or 1 per channel, so plain
                # on/off per line reproduces every state exactly.
                for pin, level in zip(self.rgb, color):
                    pin.set(bool(level))
            elif self.mono is not None:
                # mono LED: on for any non-off colour
                self.mono.set(color != OFF)
        except Exception:
            pass

    def close(self):
        self._stop.set()
        try:
            if self.rgb is not None:
                self.rgb.off()
            elif self.mono is not None:
                self.mono.off()
        except Exception:
            pass
