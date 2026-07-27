# Optional status LED - field feedback for the screenless button.
#
#   solid off      : idle
#   slow blink     : logging
#   one flash      : "logging armed" (3s hold reached)
#   double flash   : "shutdown armed" (8s hold reached)
#   fast error blip: a check failed / start aborted
#
# Uses gpiozero if present; otherwise a no-op stub so the service runs on a dev
# box (or on a Pi with no LED wired) without touching hardware.
import threading
import time

try:
    from gpiozero import LED as _LED     # type: ignore
    _HAVE_GPIO = True
except Exception:
    _HAVE_GPIO = False


class StatusLed:
    def __init__(self, pin=None):
        self.enabled = bool(pin) and _HAVE_GPIO
        self._led = _LED(pin) if self.enabled else None
        self._logging = False
        self._stop = threading.Event()
        if self.enabled:
            threading.Thread(target=self._loop, daemon=True).start()

    def set_logging(self, on):
        self._logging = on
        if self.enabled and not on:
            self._led.off()

    def armed_flash(self):
        self._blip(1)

    def shutdown_flash(self):
        self._blip(2)

    def error_blink(self):
        self._blip(5, gap=0.08)

    # ---- internals --------------------------------------------------------
    def _blip(self, n, gap=0.15):
        if not self.enabled:
            return
        for _ in range(n):
            self._led.on(); time.sleep(gap)
            self._led.off(); time.sleep(gap)

    def _loop(self):
        while not self._stop.is_set():
            if self._logging:
                self._led.on(); time.sleep(0.4)
                self._led.off(); time.sleep(0.4)
            else:
                time.sleep(0.1)

    def close(self):
        self._stop.set()
        if self.enabled:
            self._led.off()
