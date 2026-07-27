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

try:
    from gpiozero import Button as _Button   # type: ignore
    _HAVE_GPIO = True
except Exception:
    _HAVE_GPIO = False

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
            # pull_up=True => pressed reads active; matches GPIO26 -> GND wiring.
            self._btn = _Button(pin, pull_up=active_low, hold_time=0.05)
            self._btn.when_pressed = self._pressed
            self._btn.when_released = self._released
            threading.Thread(target=self._watch_hold, daemon=True).start()

    # ---- hardware callbacks ----------------------------------------------
    def _pressed(self):
        self._t0 = time.time()
        self._armed_log = self._armed_shutdown = False

    def _watch_hold(self):
        """While held, flash the LED as each threshold is crossed."""
        while True:
            if getattr(self, '_btn', None) and self._btn.is_pressed:
                held = time.time() - self._t0
                if held >= SHUTDOWN and not self._armed_shutdown:
                    self._armed_shutdown = True
                    if self.led: self.led.shutdown_flash()
                elif LOG_MIN <= held < SHUTDOWN and not self._armed_log:
                    self._armed_log = True
                    if self.led: self.led.armed_flash()
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
        from .power import power_off
        power_off(reboot=False)

    # ---- test / simulate hook --------------------------------------------
    def simulate_hold(self, seconds):
        """Pretend the button was held for `seconds` (used by tests / dev)."""
        self._dispatch(seconds)
