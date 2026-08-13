#!/usr/bin/env bash
# =============================================================================
# set_pins.sh — set the button / LED pins in /etc/mapper/field.env and restart.
#
#   bash scripts/set_pins.sh BUTTON RED GREEN BLUE
#
# e.g. Orange Pi Zero 3W, button on PD4, LED on PD2 / PB8 / PB7:
#   bash scripts/set_pins.sh 0:100 0:98 0:40 0:39
#
# A plain number is a Raspberry Pi BCM pin; CHIP:LINE is a kernel GPIO line and
# works on any board. Convert an Allwinner name (PD4) with:
#   bash scripts/list_gpio.sh names PD4
#
# Handy when the LED colours come out swapped - just reorder the last three
# arguments and run it again. No rebuild needed.
# =============================================================================
set -u

ENV_FILE=/etc/mapper/field.env
GREEN='\033[1;32m'; RED='\033[1;31m'; NC='\033[0m'

if [ $# -ne 4 ]; then
  echo "usage: bash scripts/set_pins.sh BUTTON RED GREEN BLUE"
  echo "   eg: bash scripts/set_pins.sh 0:100 0:98 0:40 0:39"
  exit 1
fi

sudo test -f "$ENV_FILE" || {
  echo -e "${RED}$ENV_FILE not found — run: bash scripts/setup_field_unit.sh${NC}"
  exit 1
}

set_key() {
  # Replace the line if the key is there, append it if it is not. A plain sed
  # would silently do nothing on a config written before these keys existed.
  if sudo grep -q "^$1=" "$ENV_FILE"; then
    sudo sed -i "s|^$1=.*|$1=$2|" "$ENV_FILE"
  else
    echo "$1=$2" | sudo tee -a "$ENV_FILE" >/dev/null
  fi
  echo "  $1=$2"
}

echo "Setting pins in $ENV_FILE:"
set_key BUTTON_GPIO "$1"
set_key LED_RED     "$2"
set_key LED_GREEN   "$3"
set_key LED_BLUE    "$4"

sudo chmod 600 "$ENV_FILE"

echo
echo "Restarting the field unit..."
sudo systemctl restart mapper-field
sleep 6
if systemctl is-active --quiet mapper-field; then
  echo -e "${GREEN}  ✓ running${NC}"
else
  echo -e "${RED}  ✗ not running — check: systemctl status mapper-field${NC}"
  exit 1
fi

cat <<'EOF'

Now test the hardware:

  - The status LED should be lit: solid green when idle and ready to record,
    solid red when it is not (no USB stick, or no LiDAR).
  - Hold the button ~4 seconds  -> starts/stops logging.
  - Hold the button ~9 seconds  -> shuts the Pi down.

If the LED lights but the colours are wrong, reorder the last three arguments
and run this again - nothing to rebuild.

If the button does nothing, find its real line with:
    bash scripts/list_gpio.sh watch
EOF
