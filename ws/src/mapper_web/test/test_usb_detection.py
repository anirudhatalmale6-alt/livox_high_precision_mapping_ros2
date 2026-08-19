import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from mapper_web.usb import UsbManager

# ---- fake block layout: Orange Pi eMMC/SD (the OS) + optional USB stick ------
# The eMMC reports hotplug=1 exactly like a USB stick. That is the whole bug.
EMMC = {
    'name': 'mmcblk0', 'type': 'disk', 'tran': None, 'rm': False,
    'hotplug': True, 'mountpoint': None, 'size': 127000000000,
    'label': None, 'fstype': None,
    'children': [
        {'name': 'mmcblk0p1', 'type': 'part', 'mountpoint': '/',
         'size': 127000000000, 'label': 'opi_root', 'fstype': 'ext4'},
    ],
}
USB_UNMOUNTED = {
    'name': 'sda', 'type': 'disk', 'tran': 'usb', 'rm': True, 'hotplug': True,
    'mountpoint': None, 'size': 64000000000, 'label': None, 'fstype': None,
    'children': [
        {'name': 'sda1', 'type': 'part', 'mountpoint': None,
         'size': 64000000000, 'label': 'LOGDATA', 'fstype': 'vfat'},
    ],
}
USB_MOUNTED = json.loads(json.dumps(USB_UNMOUNTED))
USB_MOUNTED['children'][0]['mountpoint'] = '/media/orangepi/LOGDATA'


def make(disks, root_src='/dev/mmcblk0p1'):
    m = UsbManager()
    calls = []

    def fake_run(cmd):
        calls.append(cmd)
        if cmd[0] == 'lsblk' and '-J' in cmd:
            return 0, json.dumps({'blockdevices': disks})
        if cmd[0] == 'findmnt':
            return 0, root_src + '\n'
        if cmd[0] == 'mkfs.vfat':
            raise AssertionError('mkfs.vfat RAN on ' + cmd[-1])
        return 0, ''

    m._run = fake_run
    # sysfs on this dev box has no mmcblk0p1, so resolve parents from the name
    m._parent_disk = staticmethod(
        lambda n: 'mmcblk0' if n.startswith('mmcblk0') else n.rstrip('0123456789'))
    return m, calls


fails = []


def check(label, got, want):
    ok = got == want
    print(('  PASS  ' if ok else '  FAIL  ') + label)
    if not ok:
        print('          got  %r' % (got,))
        print('          want %r' % (want,))
        fails.append(label)


print('A) OS disk only, no USB plugged in')
m, _ = make([EMMC])
s = m.status()
check('present is False', s['present'], False)
check('label is not opi_root', s['label'], '-')
check('logging_dir is empty', m.logging_dir(), '')
check('healthy_for_logging False', m.healthy_for_logging(), False)

print('B) OS disk + USB stick present but not mounted')
m, _ = make([EMMC, USB_UNMOUNTED])
s = m.status()
check('present is True', s['present'], True)
check('device is the USB', s['device'], '/dev/sda1')
check('label is LOGDATA', s['label'], 'LOGDATA')
check('mounted is False', s['mounted'], False)

print('C) OS disk + USB stick mounted')
m, _ = make([EMMC, USB_MOUNTED])
s = m.status()
check('device is the USB', s['device'], '/dev/sda1')
check('mounted is True', s['mounted'], True)
check('path is the USB path', s['path'], '/media/orangepi/LOGDATA')

print('D) format() must never touch the system disk')
m, _ = make([EMMC])
m.device_hint = '/dev/mmcblk0p1'
ok, msg = m.format()
check('format refused', ok, False)
check('reason names the system disk', 'boots from' in msg, True)

print('E) format() on a real USB is still allowed to proceed')
m, calls = make([EMMC, USB_UNMOUNTED])
try:
    m.format()
except AssertionError as e:
    print('  PASS  reached mkfs on the USB (%s)' % e)
else:
    print('  FAIL  never reached mkfs')
    fails.append('E')



# ---- auto-mount --------------------------------------------------------------
print('F) a detected-but-unmounted stick is mounted without anyone pressing ATTACH')
m, _ = make([EMMC, USB_UNMOUNTED])
tried = []
m.mount = lambda: tried.append(1) or (True, 'mounted')
check('mount attempted', m.ensure_mounted(), True)
check('exactly one attempt', len(tried), 1)
check('immediate retry is rate-limited', m.ensure_mounted(), False)
check('still one attempt', len(tried), 1)

print('G) nothing is auto-mounted when it should not be')
m, _ = make([EMMC, USB_MOUNTED])
m.mount = lambda: (_ for _ in ()).throw(AssertionError('mounted an already-mounted stick'))
check('already mounted -> no attempt', m.ensure_mounted(), False)

m, _ = make([EMMC])          # OS disk only, no stick
m.mount = lambda: (_ for _ in ()).throw(AssertionError('mounted with no stick present'))
check('no stick -> no attempt', m.ensure_mounted(), False)

m = UsbManager(simulate=True)
m.mount = lambda: (_ for _ in ()).throw(AssertionError('mounted in simulate mode'))
check('simulate -> no attempt', m.ensure_mounted(), False)

print('H) GNSS row must distinguish "searching" from "not connected"')
from mapper_web.status_monitor import gnss_label
check('live RTK fix',        gnss_label('RTK Fixed', 100.0, 101.0), ('RTK Fixed', True))
check('live, still searching', gnss_label('No fix', 100.0, 101.0), ('No fix', True))
check('never heard from',    gnss_label('No fix', 0.0, 500.0), ('Not connected', False))
check('was talking, stopped', gnss_label('RTK Fixed', 100.0, 200.0), ('Receiver silent', False))

print('')
print('FAILURES: %d' % len(fails))
sys.exit(1 if fails else 0)
