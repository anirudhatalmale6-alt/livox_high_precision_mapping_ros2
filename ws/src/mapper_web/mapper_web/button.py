# GPIO 26 pushbutton - single button, hold time decides the action.
#
#   hold 3-4 s  -> toggle logging (start if idle, stop + save if logging)
#   hold 8-9 s  -> shut the Pi down
#
# The laser head spin-up is the audible cue that START took; the optional LED
# gives cues for the 3 s and 8 s thresholds so you can feel them without a
# screen. Active-low by default (button wired GPIO26 -> GND, internal pull-up).
#
# Uses gpiozero if present; otherwise a no-op stub so the service runs off-Pi.
import threading
import time

from . import gpio as _gpio

_HAVE_GPIO = _gpio.available()

# hold-time thresholds (seconds)
LOG_MIN, LOG_MAX = 3.0, 6.0     # toggle logging when released in this band
SHUTDOWN = 8.0                  # >= this -> shutdown


class PushButton:
    def __init__(self, controller, led=None, pin=26, active_low=True, simulate=False):
        self.ctl = controller
        self.led = led
        self.pin = pin
        self.simulate = simulate or not _HAVE_GPIO
        self._armed_log = False
        self._armed_shutdown = False
        if not self.simulate:
            # Importing gpiozero is NOT proof that GPIO works: on a board it
            # does not recognise (Orange Pi, generic SBCs) the import succeeds
            # and Button() then raises BadPinFactory. That used to escape and
            # kill the dashboard on startup, so the web page never appeared
            # because of a pushbutton. Fall back to "no button" instead - the
            # web page can still do everything the button does.
            try:
                # active_low: button wired pin -> GND, with a pull-up.
                self._btn = _gpio.InputPin(pin, active_low=active_low)
                threading.Thread(target=self._watch_hold, daemon=True).start()
            except Exception as e:
                self.simulate = True
                self._btn = None
                print('[mapper_web] pushbutton disabled (no usable GPIO on this '
                      'board): %s' % e, flush=True)
                print('[mapper_web]   on a non-Raspberry-Pi board give the pin '
                      'as CHIP:LINE (e.g. --button-gpio 0:26); run '
                      'scripts/list_gpio.sh to find it.', flush=True)

    # ---- hardware callbacks ----------------------------------------------
    def _pressed(self):
        self._t0 = time.time()
        self._armed_log = self._armed_shutdown = False

    def _watch_hold(self):
        """Poll the button: detect press/release and flash at each threshold.

        Polled rather than using gpiozero's callbacks so that one code path
        serves both backends - the kernel-line backend has no equivalent
        callback API, and polling at 20 Hz is far finer than the 3-9 second
        holds this button uses.
        """
        was_down = False
        while True:
            btn = getattr(self, '_btn', None)
            down = bool(btn and btn.read())
            if down and not was_down:
                self._pressed()
            elif was_down and not down:
                self._released()
            elif down:
                held = time.time() - self._t0
                if held >= SHUTDOWN and not self._armed_shutdown:
                    self._armed_shutdown = True
                    if self.led: self.led.shutdown_flash()
                elif LOG_MIN <= held < SHUTDOWN and not self._armed_log:
                    self._armed_log = True
                    if self.led: self.led.armed_flash()
            was_down = down
            time.sleep(0.05)

    def _released(self):
        held = time.time() - self._t0
        self._dispatch(held)

    # ---- action dispatch (shared by hardware + simulate) ------------------
    def _dispatch(self, held):
        if held >= SHUTDOWN:
            self._shutdown()
        elif LOG_MIN <= held < SHUTDOWN:
            self.ctl.toggle()
        # < LOG_MIN: ignored as an accidental tap

    def _shutdown(self):
        # Park the laser first - the same as the dashboard's SHUTDOWN button.
        # This is the path taken on a unit with no screen attached, so it is
        # the one that most needs to leave the hardware tidy.
        try:
            self.ctl.park_lidar()
        except Exception:
            pass          # never let this stop the unit powering down
        from .power import power_off
        power_off(reboot=False)

    # ---- test / simulate hook --------------------------------------------
    def simulate_hold(self, seconds):
        """Pretend the button was held for `seconds` (used by tests / dev)."""
        self._dispatch(seconds)
