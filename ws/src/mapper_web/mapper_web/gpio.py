# Board-agnostic GPIO access for the pushbutton and status LED.
#
# gpiozero is Raspberry Pi specific. On an Orange Pi (or most other SBCs) it
# imports perfectly happily and then throws BadPinFactory the moment you ask
# for a pin, because it cannot work out what board it is on. That is how the
# field unit ended up with no button and no LED after moving to an Orange Pi.
#
# The Linux kernel exposes GPIO in a board-independent way through the gpiochip
# character devices, and the `lgpio` library talks to those directly. It works
# on a Pi, an Orange Pi and anything else with a mainline GPIO driver - but it
# needs the line addressed as CHIP:LINE rather than by Pi "BCM" number, because
# a BCM number is a Raspberry Pi concept that means nothing on another board.
#
# So a pin here can be written two ways:
#
#   26        - a Raspberry Pi BCM number, driven through gpiozero
#   0:26      - line 26 on /dev/gpiochip0, driven through lgpio (any board)
#   gpiochip1:5
#
# Plain numbers keep every existing Raspberry Pi setup working untouched. Use
# `bash scripts/list_gpio.sh` on a new board to find the right CHIP:LINE.
import threading

try:
    from gpiozero import LED as _GzLED, Button as _GzButton   # type: ignore
    _HAVE_GPIOZERO = True
except Exception:
    _HAVE_GPIOZERO = False

try:
    import lgpio as _lgpio                                     # type: ignore
    _HAVE_LGPIO = True
except Exception:
    _HAVE_LGPIO = False


def parse_spec(spec):
    """'26' -> ('bcm', 26).  '0:26' / 'gpiochip0:26' -> ('line', 0, 26)."""
    if spec is None:
        return None
    s = str(spec).strip()
    if ':' not in s:
        return ('bcm', int(s))
    chip_s, line_s = s.split(':', 1)
    chip_s = chip_s.strip().lower().replace('gpiochip', '')
    return ('line', int(chip_s), int(line_s))


# lgpio handles are per-chip and must be shared: opening the same chip twice
# and claiming lines from both handles fails.
_chips = {}
_chips_lock = threading.Lock()


def _chip_handle(chip):
    with _chips_lock:
        if chip not in _chips:
            _chips[chip] = _lgpio.gpiochip_open(chip)
        return _chips[chip]


def _bias_flag(*names):
    """First of `names` that this lgpio build actually defines, else 0.

    lgpio renamed these between versions: SET_BIAS_PULL_UP on current releases,
    SET_PULL_UP on older ones. Referring to the wrong one raises AttributeError
    while the argument is being BUILT - before the claim call it was meant for,
    and so before any try/except around that call can catch it. That is exactly
    how the pushbutton came to be silently disabled on the Orange Pi while the
    LED, which needs no bias flag, kept working.
    """
    for n in names:
        v = getattr(_lgpio, n, None)
        if v is not None:
            return v
    return 0


class OutputPin:
    """One output line. set(True/False). Never raises after construction."""
    def __init__(self, spec):
        self.kind = None
        self._dev = None
        self._h = None
        self._line = None
        p = parse_spec(spec)
        if p is None:
            return
        if p[0] == 'bcm':
            if not _HAVE_GPIOZERO:
                raise RuntimeError('gpiozero not available for pin %s' % spec)
            self._dev = _GzLED(p[1])
            self.kind = 'bcm'
        else:
            if not _HAVE_LGPIO:
                raise RuntimeError('lgpio not installed - needed for %s' % spec)
            _, chip, line = p
            self._h = _chip_handle(chip)
            _lgpio.gpio_claim_output(self._h, line, 0)
            self._line = line
            self.kind = 'line'

    def set(self, on):
        try:
            if self.kind == 'bcm':
                self._dev.on() if on else self._dev.off()
            elif self.kind == 'line':
                _lgpio.gpio_write(self._h, self._line, 1 if on else 0)
        except Exception:
            pass          # a flickering LED must never take the dashboard down

    def close(self):
        try:
            if self.kind == 'bcm':
                self._dev.close()
            elif self.kind == 'line':
                _lgpio.gpio_free(self._h, self._line)
        except Exception:
            pass


class InputPin:
    """One input line with a pull-up. read() -> True while pressed.

    Polled rather than interrupt-driven: the button code already runs a poll
    loop for hold timing, and polling behaves identically on both backends.
    """
    def __init__(self, spec, active_low=True):
        self.kind = None
        self.active_low = active_low
        self._dev = None
        self._h = None
        self._line = None
        p = parse_spec(spec)
        if p is None:
            return
        if p[0] == 'bcm':
            if not _HAVE_GPIOZERO:
                raise RuntimeError('gpiozero not available for pin %s' % spec)
            self._dev = _GzButton(p[1], pull_up=active_low)
            self.kind = 'bcm'
        else:
            if not _HAVE_LGPIO:
                raise RuntimeError('lgpio not installed - needed for %s' % spec)
            _, chip, line = p
            self._h = _chip_handle(chip)
            flags = (_bias_flag('SET_BIAS_PULL_UP', 'SET_PULL_UP') if active_low
                     else _bias_flag('SET_BIAS_PULL_DOWN', 'SET_PULL_DOWN'))
            try:
                if flags:
                    _lgpio.gpio_claim_input(self._h, line, flags)
                else:
                    _lgpio.gpio_claim_input(self._h, line)
            except Exception:
                # Not every SoC exposes bias control; fall back to a plain
                # input and rely on an external resistor.
                _lgpio.gpio_claim_input(self._h, line)
            self._line = line
            self.kind = 'line'

    def read(self):
        """True when the button is pressed."""
        try:
            if self.kind == 'bcm':
                return bool(self._dev.is_pressed)
            if self.kind == 'line':
                v = _lgpio.gpio_read(self._h, self._line)
                return (v == 0) if self.active_low else (v == 1)
        except Exception:
            pass
        return False

    def close(self):
        try:
            if self.kind == 'bcm':
                self._dev.close()
            elif self.kind == 'line':
                _lgpio.gpio_free(self._h, self._line)
        except Exception:
            pass


def available():
    """Is any backend usable at all?"""
    return _HAVE_GPIOZERO or _HAVE_LGPIO
