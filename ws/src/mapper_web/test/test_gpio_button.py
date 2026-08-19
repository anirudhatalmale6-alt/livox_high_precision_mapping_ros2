import io, os, sys, types, contextlib

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


def make_lgpio(naming, reads):
    lg = types.ModuleType('lgpio')
    if naming == 'new':          # his Orange Pi build
        lg.SET_BIAS_PULL_UP, lg.SET_BIAS_PULL_DOWN = 32, 64
    elif naming == 'old':
        lg.SET_PULL_UP, lg.SET_PULL_DOWN = 4, 8
    lg.claimed = []
    lg.gpiochip_open = lambda c: 100 + c
    def claim_in(h, line, flags=None):
        lg.claimed.append(('in', line, flags))
    def claim_out(h, line, val):
        lg.claimed.append(('out', line, val))
    lg.gpio_claim_input = claim_in
    lg.gpio_claim_output = claim_out
    lg.gpio_write = lambda h, l, v: None
    lg.gpio_free = lambda h, l: None
    lg.gpio_read = lambda h, l: reads.pop(0) if reads else 1
    return lg


def load(naming, reads):
    for m in [k for k in sys.modules if k.startswith('mapper_web')]:
        del sys.modules[m]
    sys.modules.pop('lgpio', None)
    sys.modules.pop('gpiozero', None)
    sys.modules['lgpio'] = make_lgpio(naming, reads)
    sys.path.insert(0, REPO)
    import mapper_web.gpio as g
    return g


fails = []


def check(label, got, want):
    ok = got == want
    print(('  PASS  ' if ok else '  FAIL  ') + label)
    if not ok:
        print('          got  %r\n          want %r' % (got, want))
        fails.append(label)


for naming, expect in (('new', 32), ('old', 4)):
    print('lgpio with %s naming' % naming)
    g = load(naming, [])
    pin = g.InputPin('0:100', active_low=True)
    check('InputPin constructed', pin.kind, 'line')
    check('claimed line 100 with the right bias flag',
          sys.modules['lgpio'].claimed[-1], ('in', 100, expect))

print('lgpio with NO bias flags at all')
g = load('none', [])
pin = g.InputPin('0:100', active_low=True)
check('still constructs', pin.kind, 'line')
check('claims with no bias rather than crashing',
      sys.modules['lgpio'].claimed[-1], ('in', 100, None))

print('polarity: pull-up means LOW = pressed')
g = load('new', [1, 0, 1])
pin = g.InputPin('0:100', active_low=True)
check('resting (line high) -> not pressed', pin.read(), False)
check('pulled to GND (low)  -> pressed', pin.read(), True)
check('released (high)      -> not pressed', pin.read(), False)

print('the LED path must keep working regardless')
g = load('none', [])
led = g.OutputPin('0:98')
check('OutputPin constructed', led.kind, 'line')

print('')
print('FAILURES: %d' % len(fails))
sys.exit(1 if fails else 0)
