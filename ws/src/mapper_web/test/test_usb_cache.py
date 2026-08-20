"""status() is cached, because it used to fork ~14 processes a second.

One uncached status() runs findmnt once per system path plus lsblk - seven
child processes - and the SSE stream asks for it twice a second for every open
browser tab. These checks pin the cache down: that it actually stops the
repeated work, that it never hides a change the operator just made, and that
the cache cannot outlive its window.
"""
import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from mapper_web.usb import UsbManager

fails = []
def check(label, got, want):
    ok = got == want
    print(('  PASS  ' if ok else '  FAIL  ') + label)
    if not ok:
        print('          got  %r\n          want %r' % (got, want)); fails.append(label)


class CountingUsb(UsbManager):
    """Counts how many times the expensive path actually runs."""
    def __init__(self, **kw):
        super().__init__(**kw)
        self.looks = 0

    def _status_uncached(self):
        self.looks += 1
        return {'present': True, 'mounted': True, 'label': 'USB',
                'total_gb': 251.6, 'free_gb': 200.0, 'free_pct': 79,
                'path': '/media/log', 'device': '/dev/sda1'}


print('repeated polling does not repeat the work')
u = CountingUsb()
for _ in range(40):                      # 20 s of SSE ticks at 2 Hz
    u.status()
check('40 polls -> 1 look', u.looks, 1)

print('the cache expires, so a stick pulled out is still noticed')
u2 = CountingUsb()
u2.CACHE_S = 0.2
u2.status()
time.sleep(0.35)
u2.status()
check('a poll after the window looks again', u2.looks, 2)

print('force=True always looks')
u3 = CountingUsb()
u3.status()
u3.status(force=True)
check('force bypasses the cache', u3.looks, 2)

print('an action the operator just took is never answered from cache')
for action in ('mount', 'eject', 'format'):
    u4 = CountingUsb()
    u4.status()                          # warm it
    before = u4.looks
    try:
        getattr(u4, action)()
    except Exception:
        pass                             # the real command is absent here
    u4.status()
    check('%s() invalidates' % action, u4.looks > before, True)

print('the caller cannot corrupt the cache by editing what it got back')
u5 = CountingUsb()
first = u5.status()
first['free_gb'] = 0.0                   # a caller mutating its copy
check('cache is not aliased', u5.status()['free_gb'], 200.0)

print('simulate mode is unaffected')
u6 = UsbManager(simulate=True)
check('sim still reports mounted', u6.status()['mounted'], True)
u6.eject()
check('sim eject still works', u6.status()['mounted'], False)

print()
print('FAILURES: %d' % len(fails))
sys.exit(1 if fails else 0)
