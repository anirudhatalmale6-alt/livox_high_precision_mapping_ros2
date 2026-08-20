"""The startup restore must not be able to put the unit in a crash loop.

Restoring the saved LiDAR settings at boot is only safe if those settings are
safe. The client can currently kill the Livox driver by switching return mode -
saved and replayed at every boot, that turns something he did once into
something the unit does to itself forever, and the more reliable the restore is
the worse the loop gets.

These checks cover the guard: a restore that survives leaves nothing behind, a
restore that does not is never repeated, and a repeat offender loses the whole
saved config rather than just the echo type.
"""
import json, os, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from mapper_web.server import App
from mapper_web.state import MapperState
import time as _t

fails = []
def check(label, got, want):
    ok = got == want
    print(('  PASS  ' if ok else '  FAIL  ') + label)
    if not ok:
        print('          got  %r\n          want %r' % (got, want)); fails.append(label)


d = tempfile.mkdtemp()
marker = os.path.join(d, 'sub', 'restore_in_progress.json')

print('marker round-trip')
check('absent reads as zero strikes', App._read_marker(marker), 0)
App._write_marker(marker, 1)
check('written strike reads back', App._read_marker(marker), 1)
App._clear_marker(marker)
check('cleared reads as zero', App._read_marker(marker), 0)

print('a corrupt marker is treated as no marker, not as a crash')
for junk in ('', 'not json', '[]', 'null', '{"strikes": "lots"}'):
    os.makedirs(os.path.dirname(marker), exist_ok=True)
    with open(marker, 'w') as f:
        f.write(junk)
    check('survives %r' % junk[:16], App._read_marker(marker), 0)

print('clearing a marker that is not there is not an error')
App._clear_marker(os.path.join(d, 'never-existed.json'))
check('no exception', True, True)

print('an unwritable location cannot stop the unit booting')
App._write_marker('/proc/nope/marker.json', 1)      # must not raise
check('write survived', App._read_marker('/proc/nope/marker.json'), 0)


# ---- the guard decision itself ----------------------------------------------
class FakeOpts:
    simulate = False

class FakeApp(App):
    """Just the restore logic, with the wiring App.__init__ would build."""
    def __init__(self, state):
        self.opts = FakeOpts()
        self.state = state
        self.sent = []
        self.monitor = self
    def send_command(self, cfg):
        self.sent.append(cfg)
        return True, 'sent'

def fresh_state():
    s = MapperState()
    s.set_config_path(os.path.join(d, 'cfg_%d.json' % len(os.listdir(d))))
    return s

print('the restore payload is only what the operator changed, one per command')
stp = fresh_state()
p0 = FakeApp(stp)._restore_payload()
check('nothing changed -> nothing to send', p0, [])
stp.merge('config', echo_type='Double Return')
p1 = FakeApp(stp)._restore_payload()
check('one change -> one command', p1, [{'echo_type': 'Double Return'}])
stp.merge('config', coordinate='Spherical')
p2 = FakeApp(stp)._restore_payload()
check('two changes -> two separate commands', len(p2), 2)
check('never batched into one', all(len(c) == 1 for c in p2), True)
stp.merge('config', work_mode='Standby', rtk_source='Serial')
p3 = FakeApp(stp)._restore_payload()
keys = sorted(k for c in p3 for k in c)
check('work_mode and rtk_source are never replayed', keys,
      ['coordinate', 'echo_type'])

print('a restore whose LiDAR keeps streaming clears the marker')
os.environ['MAPPER_RESTORE_MARKER'] = marker
App._clear_marker(marker)
stg = fresh_state()
stg.merge('config', echo_type='Single - Strongest Return')
stg.merge('lidar', ok=True, rate_hz=10.0)
g = FakeApp(stg)
g.RESTORE_PROOF_S = 0.2
g._restore_lidar_config()
_t.sleep(1.2)
check('marker cleared - restore judged safe', App._read_marker(marker), 0)

print('a restore whose LiDAR goes silent LEAVES the strike behind')
App._clear_marker(marker)
sth = fresh_state()
sth.merge('config', echo_type='Double Return')
sth.merge('lidar', ok=False, rate_hz=0.0)      # the LiDAR stopped
h = FakeApp(sth)
h.RESTORE_PROOF_S = 0.2
h._restore_lidar_config()
_t.sleep(1.2)
check('strike recorded for the next boot', App._read_marker(marker), 1)
check('it says the LiDAR stopped',
      any('LiDAR stopped' in e['text'] for e in sth.snapshot()['events']), True)

print('with no marker, the saved settings ARE restored')
os.environ['MAPPER_RESTORE_MARKER'] = marker
App._clear_marker(marker)
st = fresh_state()
st.merge('config', echo_type='Single - Strongest Return')
a = FakeApp(st)
a._restore_lidar_config()
_t.sleep(0.5)
check('a command went out', len(a.sent) >= 1, True)
check('it carried the saved echo type',
      a.sent[0].get('echo_type') if a.sent else None,
      'Single - Strongest Return')
check('rtk_source is not sent to the LiDAR', 'rtk_source' in (a.sent[0] if a.sent else {}), False)

print('one strike: the restore is skipped and the echo type reverts')
App._write_marker(marker, 1)
st2 = fresh_state()
st2.merge('config', echo_type='Double Return', work_mode='Standby')
b = FakeApp(st2)
b._restore_lidar_config()
check('nothing was sent to the LiDAR', b.sent, [])
check('echo type reverted to the safe one', st2.get('config')['echo_type'],
      'Single - First Return')
check('other settings are left alone', st2.get('config')['work_mode'], 'Standby')
check('marker cleared so the next boot may try again',
      App._read_marker(marker), 0)
check('it says what it did',
      any('Reverted echo type' in e['text'] for e in st2.snapshot()['events']),
      True)

print('two strikes: the whole config goes back to defaults')
App._write_marker(marker, 2)
st3 = fresh_state()
st3.merge('config', echo_type='Triple Return', work_mode='Standby',
          coordinate='Spherical')
c = FakeApp(st3)
c._restore_lidar_config()
check('still nothing sent', c.sent, [])
check('echo type default', st3.get('config')['echo_type'], 'Single - First Return')
check('work mode default too', st3.get('config')['work_mode'], 'Working Normally')
check('coordinate default too', st3.get('config')['coordinate'], 'Cartesian')
check('it says what it did',
      any('kept failing' in e['text'] for e in st3.snapshot()['events']), True)

print('simulate mode never touches any of this')
st4 = fresh_state()
class SimOpts:
    simulate = True
e = FakeApp(st4)
e.opts = SimOpts()
App._write_marker(marker, 2)
e._restore_lidar_config()
check('no command, no revert', (e.sent, st4.get('config')['echo_type']),
      ([], 'Single - First Return'))

print()
print('FAILURES: %d' % len(fails))
sys.exit(1 if fails else 0)
