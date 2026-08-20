"""process_alive(): the check that decides which fault the operator has.

A dead control link with the driver process GONE is a crash. The same link
dead with the process ALIVE is something in the ROS layer. They need opposite
investigations, and getting this check wrong sends the operator - and me - the
wrong way entirely.
"""
import os, subprocess, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from mapper_web.status_monitor import process_alive, LIVOX_DRIVER_PROC

fails = []
def check(label, got, want):
    ok = got == want
    print(('  PASS  ' if ok else '  FAIL  ') + label)
    if not ok:
        print('          got  %r\n          want %r' % (got, want)); fails.append(label)

def named_process(name, seconds=30):
    """Start a process that really carries `name` in /proc/<pid>/comm."""
    script = ('import time,ctypes;'
              'ctypes.CDLL("libc.so.6").prctl(15, b"%s", 0, 0, 0);'
              'time.sleep(%d)' % (name, seconds))
    return subprocess.Popen([sys.executable, '-c', script])


print('finds a process that is really there, and only while it is there')
# A UNIQUE name, not something like "sleep": this machine runs other work, and
# a shared name makes the "gone" case pass or fail on what else happens to be
# running. That is exactly the kind of test that lies.
uniq = 'mw_probe_%d' % os.getpid()
p = named_process(uniq)
time.sleep(0.6)
check('found while running', process_alive(uniq), True)
p.kill(); p.wait()
time.sleep(0.6)
check('and not after it exits', process_alive(uniq), False)

print('does not invent processes')
check('a name nothing uses', process_alive('zzz_no_such_process'), False)
check('an empty name matches nothing real', process_alive('') in (True, False), True)

print('the 15-character kernel truncation is handled')
# /proc/<pid>/comm is capped at 15 chars, so the driver's 22-character name is
# stored truncated. A naive full-name compare would never match and the
# dashboard would report a healthy driver as missing.
check('driver name is longer than the cap', len(LIVOX_DRIVER_PROC) > 15, True)
long_name = 'mw_long_name_probe_abcdef'      # 25 chars
q = named_process(long_name)
time.sleep(0.6)
found = process_alive(long_name)
q.kill(); q.wait()
check('a >15-char process name is still found', found, True)

print('a name that only differs past character 15 is NOT distinguishable')
# Worth stating rather than pretending otherwise: the kernel has thrown those
# characters away, so this check cannot tell such names apart. It is fine here
# because nothing else on the unit begins with "livox_ros2_driv".
check('documented limitation', LIVOX_DRIVER_PROC[:15], 'livox_ros2_driv')

print()
print('FAILURES: %d' % len(fails))
sys.exit(1 if fails else 0)
