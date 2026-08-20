import json, os, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from mapper_web.state import MapperState

fails = []
def check(label, got, want):
    ok = got == want
    print(('  PASS  ' if ok else '  FAIL  ') + label)
    if not ok:
        print('          got  %r\n          want %r' % (got, want)); fails.append(label)

d = tempfile.mkdtemp()
path = os.path.join(d, 'sub', 'lidar_config.json')

print('a chosen setting survives a restart')
s = MapperState()
s.set_config_path(path)
s.merge('config', echo_type='Single - Strongest Return')
s.save_config()
check('file written through a missing directory', os.path.exists(path), True)

s2 = MapperState()
s2.set_config_path(path)
check('echo type restored', s2.get('config')['echo_type'],
      'Single - Strongest Return')
check('untouched keys keep their defaults', s2.get('config')['coordinate'],
      'Cartesian')

print('the panel and the file agree on everything, not just the edited key')
s2.merge('config', work_mode='Standby')
s2.save_config()
s3 = MapperState()
s3.set_config_path(path)
check('both survive together', (s3.get('config')['echo_type'],
                               s3.get('config')['work_mode']),
      ('Single - Strongest Return', 'Standby'))

print('a corrupt or hostile file can never stop the unit booting')
for junk in ('not json at all', '[]', 'null', '{"echo_type": 42}',
             '{"nonsense_key": "x"}', ''):
    with open(path, 'w') as f:
        f.write(junk)
    s4 = MapperState()
    s4.set_config_path(path)          # must not raise
    check('survives %r' % junk[:18], s4.get('config')['echo_type'],
          'Single - First Return')

print('a value of the wrong type is ignored rather than displayed')
with open(path, 'w') as f:
    json.dump({'echo_type': ['a', 'list'], 'work_mode': 'Standby'}, f)
s5 = MapperState()
s5.set_config_path(path)
check('bad value rejected', s5.get('config')['echo_type'], 'Single - First Return')
check('good value beside it still loads', s5.get('config')['work_mode'], 'Standby')

print('an unwritable path is survivable - a unit that cannot save its settings')
print('still scans perfectly well')
s6 = MapperState()
s6.set_config_path('/proc/nonexistent/lidar_config.json')
s6.merge('config', echo_type='Double Return')
s6.save_config()                      # must not raise
check('still reports what was chosen', s6.get('config')['echo_type'],
      'Double Return')

print('with no path set at all, saving is a no-op rather than an error')
s7 = MapperState()
s7.merge('config', echo_type='Triple Return')
s7.save_config()
check('no crash without a path', s7.get('config')['echo_type'], 'Triple Return')

print()
print('FAILED: %d' % len(fails) if fails else 'all passed')
sys.exit(1 if fails else 0)
