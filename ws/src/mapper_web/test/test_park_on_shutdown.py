"""SHUTDOWN parks the laser before the machine goes away.

The Avia has its own power supply: shutting the Pi down does not switch the
LiDAR off, it only removes the one thing that was talking to it. Without this
the laser keeps spinning, unattended, with nothing left able to stop it.

Client's request. These checks cover both routes to a shutdown - the dashboard
button and the 8-second hold on the physical button - because the second is the
one used on a unit with no screen attached.
"""
import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from mapper_web.logging_controller import LoggingController
from mapper_web.button import PushButton
from mapper_web.state import MapperState

fails = []
def check(label, got, want):
    ok = got == want
    print(('  PASS  ' if ok else '  FAIL  ') + label)
    if not ok:
        print('          got  %r\n          want %r' % (got, want)); fails.append(label)


class FakeUsb:
    def healthy_for_logging(self, *a, **k): return True
    def logging_dir(self): return '/tmp'
    def eject(self): return True, 'ejected'

def controller(link_ok=True):
    s = MapperState()
    c = LoggingController(s, FakeUsb(), simulate=False)
    c.PARK_SETTLE_S = 0.05           # keep the test quick
    sent = []
    c.set_config_sink(
        lambda cfg: (sent.append(cfg), (link_ok, 'sent' if link_ok else 'down'))[1])
    return c, s, sent


print('parking asks for Power Saving and says so')
c, s, sent = controller()
check('returns True', c.park_lidar(), True)
check('asked for Power Saving', sent, [{'work_mode': 'Power Saving'}])
check('recorded in the action window',
      any('parked for shutdown' in e['text'] for e in s.snapshot()['events']), True)

print('a dead control link does not stop the shutdown')
c2, s2, sent2 = controller(link_ok=False)
check('returns False rather than raising', c2.park_lidar(), False)
check('and says so plainly',
      any('Could not park' in e['text'] for e in s2.snapshot()['events']), True)

print('it waits for the command to land before the caller powers off')
c3, s3, _ = controller()
c3.PARK_SETTLE_S = 0.4
t0 = time.time()
c3.park_lidar()
elapsed = time.time() - t0
check('blocks for the settle time', elapsed >= 0.35, True)
# A failed park must NOT burn the settle time - there is nothing to wait for,
# and the operator is waiting on a shutdown.
c4, s4, _ = controller(link_ok=False)
c4.PARK_SETTLE_S = 0.4
t0 = time.time()
c4.park_lidar()
check('a failed park returns immediately', (time.time() - t0) < 0.2, True)

print('simulate mode parks nothing')
s5 = MapperState()
c5 = LoggingController(s5, FakeUsb(), simulate=True)
sent5 = []
c5.set_config_sink(lambda cfg: (sent5.append(cfg), (True, 'x'))[1])
check('no command in sim', (c5.park_lidar(), sent5), (True, []))

print('the physical button parks too, and powers off even if parking throws')
class Boom(LoggingController):
    def park_lidar(self): raise RuntimeError('control link exploded')

powered = []
import mapper_web.power as power
real_power_off = power.power_off
power.power_off = lambda reboot=False: powered.append(reboot) or (True, 'off')
try:
    c6, s6, sent6 = controller()
    b = PushButton(c6, simulate=True)
    b._shutdown()
    check('button parked the laser', sent6, [{'work_mode': 'Power Saving'}])
    check('button still powered off', powered, [False])

    powered.clear()
    b2 = PushButton(Boom(MapperState(), FakeUsb(), simulate=False), simulate=True)
    b2._shutdown()
    check('a parking failure never blocks power-off', powered, [False])
finally:
    power.power_off = real_power_off

print()
print('FAILURES: %d' % len(fails))
sys.exit(1 if fails else 0)
