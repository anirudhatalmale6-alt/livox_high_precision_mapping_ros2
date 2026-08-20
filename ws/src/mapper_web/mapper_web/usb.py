# USB storage manager - the logging target.
#
# Auto-detects the USB stick wherever the system put it (desktop auto-mount
# lands it at /media/<user>/<label>, not a fixed path), reports capacity, and
# safely mounts / unmounts / formats it. Logs are written to the stick, never
# the Pi's SD card.
#
# Detection is by block device, not by a hardcoded mount point, so "just plug it
# in" works. Everything degrades gracefully so the service still runs on a box
# with no USB attached (simulation).
import json
import re
import os
import shutil
import subprocess
import time


class UsbManager:
    def __init__(self, mount_point='', device_hint='', simulate=False):
        self.fallback_mount = mount_point   # only used if auto-detect finds nothing
        self.device_hint = device_hint      # force a device, e.g. /dev/sda1
        self.simulate = simulate
        self._last_mount_try = -1e9   # rate-limits ensure_mounted()
        self._cache = None            # see status()
        self._cache_t = 0.0
        self._sim = {
            'present': True, 'mounted': True, 'label': 'SIM-USB',
            'total_gb': 128.0, 'free_gb': 80.6, 'free_pct': 63,
            'path': '/media/log', 'device': '/dev/sda1',
        }

    # ---- status -----------------------------------------------------------
    # Cached, because this is not cheap and it is asked for constantly.
    #
    # One status() is seven child processes: findmnt once per system path, then
    # lsblk. The SSE stream calls it twice a second FOR EVERY OPEN BROWSER, so a
    # single dashboard tab was forking about fourteen processes a second, and
    # two tabs twice that - forever, on a board with eight small cores.
    #
    # On 20 Aug 2026 the client's unit had to be SIGKILLed with THIRTEEN
    # processes named mapper_web still alive, nine of them with consecutive
    # PIDs. A child keeps its parent's name until execve() completes, so a fork
    # storm on a loaded machine shows up as exactly that. Whether or not it is
    # the whole story, spawning fourteen processes a second to answer "is the
    # USB stick still there" is indefensible on its own.
    #
    # A USB stick does not appear and disappear inside two seconds, and every
    # action that DOES change it (mount, eject, format) clears the cache itself.
    CACHE_S = 2.0

    def status(self, force=False):
        if self.simulate:
            return dict(self._sim)
        now = time.time()
        if not force and self._cache is not None and \
                (now - self._cache_t) < self.CACHE_S:
            return dict(self._cache)
        info = self._status_uncached()
        self._cache = dict(info)
        self._cache_t = now
        return info

    def invalidate(self):
        """Force the next status() to go and look properly."""
        self._cache = None

    def _status_uncached(self):
        f = self._find()
        info = {'present': False, 'mounted': False, 'label': '-',
                'total_gb': 0.0, 'free_gb': 0.0, 'free_pct': 0,
                'path': '', 'device': ''}
        if not f:
            # last resort: an explicit mount point that's actually mounted
            if self.fallback_mount and os.path.ismount(self.fallback_mount):
                return self._usage({'present': True, 'mounted': True,
                                    'label': 'USB', 'path': self.fallback_mount,
                                    'device': ''})
            return info
        info['present'] = True
        info['device'] = f['device']
        info['label'] = f['label'] or 'USB'
        if f['mountpoint']:
            info['mounted'] = True
            info['path'] = f['mountpoint']
            info = self._usage(info)
        return info

    def _usage(self, info):
        try:
            u = shutil.disk_usage(info['path'])
            info['total_gb'] = round(u.total / 1e9, 1)
            info['free_gb'] = round(u.free / 1e9, 1)
            info['free_pct'] = int(u.free * 100 / u.total) if u.total else 0
        except OSError:
            pass
        return info

    def logging_dir(self):
        """Where a run should write - the live USB mount, or '' if none."""
        s = self.status()
        return s['path'] if s['mounted'] else ''

    def ensure_mounted(self, current=None, retry_after_s=10.0):
        """Mount a detected stick that isn't mounted yet. Returns True if a
        mount was attempted, so the caller knows to re-read the status.

        A unit that needs someone to press ATTACH before it can record is not
        "power on and go" - and an unmounted stick otherwise just stalls the
        pre-flight checks until they time out, with nothing saying why. Safe to
        do automatically: detection excludes the system disk, and mounting is
        reversible.
        """
        if self.simulate:
            return False
        s = current if current is not None else self.status()
        if not s['present'] or s['mounted']:
            return False
        now = time.monotonic()
        if now - self._last_mount_try < retry_after_s:
            return False        # don't hammer a stick that keeps refusing
        self._last_mount_try = now
        self.mount()
        return True

    def healthy_for_logging(self, min_free_gb=1.0):
        s = self.status()
        if self.ensure_mounted(s):
            s = self.status()
        return s['mounted'] and s['free_gb'] >= min_free_gb

    # ---- operations -------------------------------------------------------
    def mount(self):
        if self.simulate:
            self._sim['mounted'] = True
            return True, 'mounted (sim)'
        self.invalidate()      # this changes the answer; do not serve a stale one
        f = self._find()
        dev = self.device_hint or (f['device'] if f else '')
        if not dev:
            return False, 'no USB device found'
        if f and f['mountpoint']:
            return True, 'already mounted at ' + f['mountpoint']
        # udisksctl mounts as the desktop user (auto path); fall back to mount.
        rc, out = self._run(['udisksctl', 'mount', '-b', dev])
        if rc == 0:
            return True, out.strip() or 'mounted'
        mp = self.fallback_mount or '/media/log'
        os.makedirs(mp, exist_ok=True)
        rc, out = self._run(['mount', dev, mp])
        return rc == 0, out.strip() or ('mounted at ' + mp)

    def eject(self):
        """Flush and unmount so the stick is safe to physically pull."""
        if self.simulate:
            self._sim['mounted'] = False
            return True, 'ejected (sim)'
        self.invalidate()
        self._run(['sync'])
        f = self._find()
        dev = self.device_hint or (f['device'] if f else '')
        if dev:
            rc, out = self._run(['udisksctl', 'unmount', '-b', dev])
            if rc == 0:
                return True, out.strip() or 'ejected'
        # fall back to unmounting by path
        path = (f['mountpoint'] if f else '') or self.fallback_mount
        if path:
            rc, out = self._run(['umount', path])
            return rc == 0, out.strip() or 'ejected'
        return False, 'no USB to eject'

    def format(self, label='LOGDATA'):
        """Erase the drive (vfat). Destructive - callers must confirm first."""
        if self.simulate:
            self._sim['free_gb'] = self._sim['total_gb']
            self._sim['free_pct'] = 100
            return True, 'formatted (sim)'
        self.invalidate()
        f = self._find()
        dev = self.device_hint or (f['device'] if f else '')
        if not dev:
            return False, 'no USB device found'
        # Last line of defence. _find() already skips the system disk, but a
        # device_hint bypasses it and a wrong answer here erases the OS, so the
        # check is repeated at the point of no return.
        if self._is_system_device(dev):
            return False, ('refusing to format ' + dev + ' - that is the disk '
                           'this system boots from, not a USB stick')
        if f and f['mountpoint']:
            self._run(['udisksctl', 'unmount', '-b', dev])
        rc, out = self._run(['mkfs.vfat', '-F', '32', '-n', label, dev])
        if rc == 0:
            self.mount()
        return rc == 0, out.strip() or 'formatted'

    # ---- what must never be touched ---------------------------------------
    # An SD card or eMMC is reported by the kernel as removable/hotplug exactly
    # like a USB stick. On an Orange Pi that is the disk the OS boots from, so
    # "removable" on its own picked the system disk as the logging target - and
    # offered to format it. Nothing here may act on the disk carrying the OS.
    SYSTEM_PATHS = ('/', '/boot', '/boot/firmware', '/usr', '/var', '/home')

    @staticmethod
    def _parent_disk(name):
        """'mmcblk0p1' -> 'mmcblk0', 'sda1' -> 'sda', 'sda' -> 'sda'.

        Read from sysfs rather than by trimming the name, because the naming
        rules differ between sd*, mmcblk* and nvme*.
        """
        base = '/sys/class/block/' + name
        try:
            if os.path.exists(base + '/partition'):
                return os.path.basename(os.path.dirname(os.path.realpath(base)))
            if os.path.exists(base):
                return name              # sysfs says it is a whole disk
        except OSError:
            pass
        # sysfs could not answer (container, unusual kernel). This guard is what
        # stops the OS being formatted, so fall back to the naming rules rather
        # than returning a name that matches no disk and protects nothing.
        m = re.match(r'^(.*\d)p\d+$', name)      # mmcblk0p1, nvme0n1p2
        if m:
            return m.group(1)
        if name.startswith(('mmcblk', 'nvme', 'loop')):
            return name       # whole disk - its digits are part of the name
        return name.rstrip('0123456789')          # sda1 -> sda

    def _system_disks(self):
        """Disks carrying the OS. Never a logging target, never formattable."""
        protected = set()
        for path in self.SYSTEM_PATHS:
            # --target answers for the filesystem CONTAINING the path, so this
            # works whether or not /boot is a separate mount.
            rc, out = self._run(['findmnt', '-no', 'SOURCE', '--target', path])
            if rc != 0 or not out:
                continue
            src = out.strip().split('\n')[0].strip()
            if not src.startswith('/dev/'):
                continue                   # overlay, tmpfs, zfs - not a disk
            protected.add(self._parent_disk(os.path.basename(src)))
        protected.discard('')
        return protected

    def _is_system_device(self, dev):
        if not dev:
            return False
        return self._parent_disk(os.path.basename(dev)) in self._system_disks()

    # ---- detection --------------------------------------------------------
    def _find(self):
        """Return {device, mountpoint, label, fstype} for the best USB
        partition, or None. Auto-detects via lsblk - no fixed mount path."""
        rc, out = self._run(['lsblk', '-J', '-b', '-o',
                             'NAME,TYPE,TRAN,RM,HOTPLUG,MOUNTPOINT,SIZE,LABEL,FSTYPE'])
        if rc != 0:
            return None
        try:
            tree = json.loads(out).get('blockdevices', [])
        except ValueError:
            return None

        protected = self._system_disks()
        best = None
        for disk in tree:
            if disk.get('type') != 'disk':
                continue
            if disk.get('name') in protected:
                continue                   # the OS lives here - skip entirely
            removable = (disk.get('tran') == 'usb'
                         or str(disk.get('hotplug')) in ('1', 'True', 'true')
                         or str(disk.get('rm')) in ('1', 'True', 'true'))
            if not removable:
                continue
            # candidate partitions with a filesystem (or a whole-disk fs)
            cands = [c for c in disk.get('children', []) if c.get('fstype')]
            if not cands and disk.get('fstype'):
                cands = [disk]
            # prefer a mounted candidate, else the first with a filesystem
            cands.sort(key=lambda c: (0 if c.get('mountpoint') else 1))
            for c in cands:
                mp = c.get('mountpoint') or ''
                if mp in self.SYSTEM_PATHS or mp.startswith('/boot'):
                    continue               # belt and braces
                cand = {'device': '/dev/' + c['name'],
                        'mountpoint': mp,
                        'label': c.get('label') or '',
                        'fstype': c.get('fstype') or ''}
                if cand['mountpoint']:
                    return cand           # a mounted USB fs is the winner
                best = best or cand        # remember an unmounted one
        return best

    @staticmethod
    def _run(cmd):
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            return p.returncode, (p.stdout or p.stderr)
        except (subprocess.SubprocessError, FileNotFoundError) as e:
            return 1, str(e)
