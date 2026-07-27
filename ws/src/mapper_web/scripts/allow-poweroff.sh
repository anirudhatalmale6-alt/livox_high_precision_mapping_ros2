#!/bin/sh
# Let the dashboard (and the GPIO button) power off / reboot the Pi WITHOUT a
# password prompt. Run this ONCE. Only needed if you launch the dashboard as
# your normal user - the boot service (mapper-field.service) runs as root and
# never prompts.
#
#   sh allow-poweroff.sh            # allow the current user
#   sh allow-poweroff.sh drone1     # allow a specific user
#
set -e
USER_TO_ALLOW="${1:-$(id -un)}"
RULE="$USER_TO_ALLOW ALL=(root) NOPASSWD: /sbin/shutdown, /usr/sbin/shutdown, /sbin/reboot, /usr/sbin/reboot, /bin/systemctl poweroff, /bin/systemctl reboot"

echo "Adding passwordless power-off for user: $USER_TO_ALLOW"
echo "$RULE" | sudo tee /etc/sudoers.d/mapper-poweroff >/dev/null
sudo chmod 440 /etc/sudoers.d/mapper-poweroff

# Validate so we never leave a broken sudoers file.
if sudo visudo -cf /etc/sudoers.d/mapper-poweroff >/dev/null 2>&1; then
    echo "Done. The SHUTDOWN button and the 8-9 s button hold now work without a password."
else
    echo "sudoers check FAILED - removing the rule to be safe." >&2
    sudo rm -f /etc/sudoers.d/mapper-poweroff
    exit 1
fi
