"""Pressing START while the laser is parked must wake it, not time out.

The client parks the LiDAR in Power Saving between scans - a normal thing to do
and it works fine. But the laser is off in that state, so the pre-flight checks
would sit through their whole window waiting for data that could not arrive,
then abort with "LiDAR not ready". Pressing START LOGGING is not an ambiguous
request.
"""
import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from mapper_web.logging_controller import LoggingController
from mapper_web.state import MapperState

fails = []
def check(label, got, want):
    ok = got == want
    print(('  PASS  ' if ok else '  FAIL  ') + label)
    if not ok:
        print('          got  %r\n          want %r' % (got, want)); fails.append(label)


class FakeUsb:
    simulate = False
    def healthy_for_logging(self, *a, **k): return True
    def logging_dir(self): return '/tmp'
    def eject(self): return True, 'ejected'

def controller(work_mode):
    s = MapperState()
    s.merge('device', work_mode=work_mode)
    c = LoggingController(s, FakeUsb(), simulate=False)
    sent = []
    c.set_config_sink(lambda cfg: (sent.append(cfg), (True, 'sent'))[1])
    return c, s, sent


print('a parked laser is woken')
for mode in ('Power Saving', 'Standby'):
    c, s, sent = controller(mode)
    woke = c._wake_lidar()
    check('%s -> wake requested' % mode, woke, True)
    check('%s -> asked for Working Normally' % mode, sent,
          [{'work_mode': 'Working Normally'}])
    check('%s -> said so in the action window' % mode,
          any('Waking the LiDAR' in e['text'] for e in s.snapshot()['events']),
          True)

print('a running laser is left completely alone')
for mode in ('Working Normally', 'Initializing', 'Unknown'):
    c, s, sent = controller(mode)
    check('%s -> no wake' % mode, c._wake_lidar(), False)
    check('%s -> nothing sent' % mode, sent, [])
    check('%s -> nothing logged' % mode, s.snapshot()['events'], [])

print('the wake decision does not depend on the DROPDOWN, only the device')
# config['work_mode'] is what the operator last chose; device['work_mode'] is
# what the LiDAR is actually doing. Waking must follow the device, or a stale
# dropdown would either skip a needed wake or issue a pointless one.
s = MapperState()
s.merge('device', work_mode='Power Saving')
s.merge('config', work_mode='Working Normally')      # dropdown disagrees
c = LoggingController(s, FakeUsb(), simulate=False)
sent = []
c.set_config_sink(lambda cfg: (sent.append(cfg), (True, 'sent'))[1])
check('follows the device, not the dropdown', c._wake_lidar(), True)

print('a wake buys a longer check window, and only then')
c, s, sent = controller('Power Saving')
seen = {}
c._data_checks = lambda timeout_s=60: seen.setdefault('t', timeout_s) and False
c._launch = lambda: False
c._start_sequence()
check('woken -> 180 s', seen.get('t'), 180)

c2, s2, sent2 = controller('Working Normally')
seen2 = {}
c2._data_checks = lambda timeout_s=60: seen2.setdefault('t', timeout_s) and False
c2._launch = lambda: False
c2._start_sequence()
check('already running -> the usual 60 s', seen2.get('t'), 60)

print('a control link that refuses the wake is not treated as woken')
s3 = MapperState()
s3.merge('device', work_mode='Power Saving')
c3 = LoggingController(s3, FakeUsb(), simulate=False)
c3.set_config_sink(lambda cfg: (False, 'link down'))
check('refused wake reports False', c3._wake_lidar(), False)

print()
print('FAILURES: %d' % len(fails))
sys.exit(1 if fails else 0)
