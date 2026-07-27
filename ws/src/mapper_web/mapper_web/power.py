# Power control - shut down / reboot the Pi without ever hanging on a password.
#
# The web SHUTDOWN button and the GPIO 8-9 s hold both come here. We must NOT
# call plain `sudo`, or it blocks waiting for a password in whatever terminal
# the process happens to own. So:
#   - running as root (the boot service does)  -> call shutdown directly
#   - otherwise                                -> `sudo -n` (non-interactive):
#     it works if passwordless sudo is set up, and fails instantly (no prompt)
#     if not, with a clear message pointing at the one-time setup script.
import os
import subprocess


def _can_sudo():
    try:
        r = subprocess.run(['sudo', '-n', 'true'], capture_output=True, timeout=5)
        return r.returncode == 0
    except (subprocess.SubprocessError, FileNotFoundError):
        return False


def power_off(reboot=False):
    action = '-r' if reboot else '-h'
    word = 'reboot' if reboot else 'shutdown'
    base = ['shutdown', action, 'now']
    try:
        if os.geteuid() == 0:
            subprocess.Popen(base)
            return True, word + ' issued'
        if _can_sudo():
            subprocess.Popen(['sudo', '-n'] + base)
            return True, word + ' issued'
        return (False,
                'passwordless power-off not set up - run '
                'scripts/allow-poweroff.sh once (or start via the boot service)')
    except OSError as e:
        return False, str(e)
