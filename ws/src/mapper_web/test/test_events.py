import os, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from mapper_web.state import MapperState

fails = []
def check(label, got, want):
    ok = got == want
    print(('  PASS  ' if ok else '  FAIL  ') + label)
    if not ok:
        print('          got  %r\n          want %r' % (got, want)); fails.append(label)

print('ring buffer keeps only the newest 5, newest first')
s = MapperState()
for i in range(8):
    s.add_event('event %d' % i)
evs = s.snapshot()['events']
check('exactly 5 kept', len(evs), 5)
check('newest first', [e['text'] for e in evs],
      ['event 7','event 6','event 5','event 4','event 3'])
check('each has a clock time', all(len(e['time']) == 8 for e in evs), True)

print('survives a restart')
d = tempfile.mkdtemp()
p = os.path.join(d, 'sub', 'events.json')
s1 = MapperState(); s1.set_events_path(p)     # nothing there yet
check('starts empty', s1.snapshot()['events'], [])
s1.add_event('Recording started (button)')
s1.add_event('Saved map_001.pcd (2m 14s)')
s2 = MapperState(); s2.set_events_path(p)     # a fresh boot
check('history reloaded', [e['text'] for e in s2.snapshot()['events']],
      ['Saved map_001.pcd (2m 14s)', 'Recording started (button)'])

print('a broken or missing history file must never take the unit down')
s3 = MapperState(); s3.set_events_path(os.path.join(d, 'nope', 'events.json'))
check('missing file tolerated', s3.snapshot()['events'], [])
bad = os.path.join(d, 'bad.json'); open(bad, 'w').write('{not json')
s4 = MapperState(); s4.set_events_path(bad)
check('corrupt file tolerated', s4.snapshot()['events'], [])
s4.add_event('still works')
check('and still records', len(s4.snapshot()['events']), 1)

s5 = MapperState(); s5.set_events_path('/proc/cannot/write/here.json')
s5.add_event('unwritable')
check('unwritable path tolerated', len(s5.snapshot()['events']), 1)

print('')
print('FAILURES: %d' % len(fails))
sys.exit(1 if fails else 0)
