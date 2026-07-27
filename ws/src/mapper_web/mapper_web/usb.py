# USB storage manager - the logging target.
#
# Detects the mounted USB stick, reports capacity, and safely mounts / unmounts
# / formats it. Logs are written here, never on the Pi's SD card.
#
# The mount point defaults to /media/log. On the field Pi we expect the drive to
# auto-mount there (via /etc/fstab or udisks). Everything degrades gracefully so
# the service still runs on a dev box with no USB attached (simulation).
import os
import shutil
import subprocess


class UsbManager:
    def __init__(self, mount_point='/media/log', device_hint='', simulate=False):
        self.mount_point = mount_point
        self.device_hint = device_hint      # e.g. /dev/sda1; auto-detect if empty
        self.simulate = simulate
        self._sim = {
            'present': True, 'mounted': True, 'label': 'SIM-USB',
            'total_gb': 128.0, 'free_gb': 80.6, 'free_pct': 63,
            'path': mount_point,
        }

    # ---- status -----------------------------------------------------------
    def status(self):
        if self.simulate:
            return dict(self._sim)
        mounted = os.path.ismount(self.mount_point)
        info = {
            'present': mounted, 'mounted': mounted, 'label': '-',
            'total_gb': 0.0, 'free_gb': 0.0, 'free_pct': 0,
            'path': self.mount_point if mounted else '',
        }
        if mounted:
            try:
                u = shutil.disk_usage(self.mount_point)
                info['total_gb'] = round(u.total / 1e9, 1)
                info['free_gb'] = round(u.free / 1e9, 1)
                info['free_pct'] = int(u.free * 100 / u.total) if u.total else 0
                info['label'] = self._label()
            except OSError:
                pass
        else:
            # drive may be inserted but not mounted yet
            info['present'] = bool(self._device())
        return info

    def healthy_for_logging(self, min_free_gb=1.0):
        s = self.status()
        return s['mounted'] and s['free_gb'] >= min_free_gb

    # ---- operations -------------------------------------------------------
    def mount(self):
        if self.simulate:
            self._sim['mounted'] = True
            return True, 'mounted (sim)'
        dev = self._device()
        if not dev:
            return False, 'no USB device found'
        os.makedirs(self.mount_point, exist_ok=True)
        rc, out = self._run(['mount', dev, self.mount_point])
        return rc == 0, out or 'mounted'

    def eject(self):
        """Flush and unmount so the stick is safe to physically pull."""
        if self.simulate:
            self._sim['mounted'] = False
            return True, 'ejected (sim)'
        self._run(['sync'])
        rc, out = self._run(['umount', self.mount_point])
        return rc == 0, out or 'ejected'

    def format(self, label='LOGDATA'):
        """Erase the drive (vfat). Destructive - callers must confirm first."""
        if self.simulate:
            self._sim['free_gb'] = self._sim['total_gb']
            self._sim['free_pct'] = 100
            return True, 'formatted (sim)'
        dev = self._device()
        if not dev:
            return False, 'no USB device found'
        if os.path.ismount(self.mount_point):
            self._run(['umount', self.mount_point])
        rc, out = self._run(['mkfs.vfat', '-F', '32', '-n', label, dev])
        if rc == 0:
            self.mount()
        return rc == 0, out or 'formatted'

    # ---- helpers ----------------------------------------------------------
    def _device(self):
        if self.device_hint:
            return self.device_hint if os.path.exists(self.device_hint) else ''
        # pick the first USB partition reported by lsblk
        rc, out = self._run(['lsblk', '-rno', 'NAME,TRAN,TYPE'])
        if rc != 0:
            return ''
        for line in out.splitlines():
            f = line.split()
            if len(f) >= 3 and f[1] == 'usb' and f[2] == 'part':
                return '/dev/' + f[0]
        return ''

    def _label(self):
        rc, out = self._run(['lsblk', '-rno', 'LABEL', self._device()]) if self._device() else (1, '')
        return out.strip() or 'USB'

    @staticmethod
    def _run(cmd):
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            return p.returncode, (p.stdout or p.stderr).strip()
        except (subprocess.SubprocessError, FileNotFoundError) as e:
            return 1, str(e)
