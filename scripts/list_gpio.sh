#!/usr/bin/env bash
# =============================================================================
# list_gpio.sh — show the GPIO lines this board actually has, and help you find
# which one your button / LED is wired to.
#
#   bash scripts/list_gpio.sh              # just list what exists
#   bash scripts/list_gpio.sh watch        # list, then watch for a line to change
#
# Why this is needed: "GPIO 26" is a Raspberry Pi number. It means nothing on an
# Orange Pi, which is why the button and LED stopped working when the unit moved
# boards. The kernel addresses pins as CHIP:LINE instead, which works everywhere
# — and that is what the dashboard now wants for a non-Pi board.
# =============================================================================
set -u

GREEN='\033[1;32m'; YELLOW='\033[1;33m'; BLUE='\033[1;34m'; NC='\033[0m'
step() { echo -e "\n${BLUE}==== $* ====${NC}"; }
ok()   { echo -e "${GREEN}  ✓ $*${NC}"; }
warn() { echo -e "${YELLOW}  ! $*${NC}"; }

step "Board"
if [ -r /proc/device-tree/model ]; then
  echo "  $(tr -d '\0' < /proc/device-tree/model)"
else
  echo "  (unknown)"
fi

step "GPIO chips the kernel can see"
if ! ls /dev/gpiochip* >/dev/null 2>&1; then
  warn "No /dev/gpiochip* at all — this kernel exposes no GPIO."
  echo "  The button and LED cannot work on this board."
  exit 1
fi
ls -1 /dev/gpiochip* | sed 's/^/  /'

if ! command -v gpioinfo >/dev/null 2>&1; then
  echo
  warn "gpioinfo not installed — install it for names and current use:"
  echo "      sudo apt install -y gpiod"
fi

step "Lines"
if command -v gpioinfo >/dev/null 2>&1; then
  # Only show lines that are free; a line already owned by the kernel (power
  # rails, MMC, etc.) is not one you can wire a button to.
  gpioinfo 2>/dev/null | awk '
    /^gpiochip/ {chip=$1; print "\n  " $0; next}
    /unused/ && /input/ {print "   " $0}
  ' | head -80
  echo
  echo "  (only free input-capable lines shown; full list: gpioinfo)"
else
  python3 - <<'PY' 2>/dev/null || echo "  install gpiod or python3-lgpio to list lines"
try:
    import lgpio, glob, os
    for dev in sorted(glob.glob('/dev/gpiochip*')):
        n = int(dev.replace('/dev/gpiochip', ''))
        try:
            h = lgpio.gpiochip_open(n)
            info = lgpio.gpio_get_chip_info(h)
            print('  gpiochip%d: %s lines' % (n, info[1]))
            lgpio.gpiochip_close(h)
        except Exception as e:
            print('  gpiochip%d: could not open (%s)' % (n, e))
except ImportError:
    raise SystemExit(1)
PY
fi

step "How to use this"
cat <<'EOF'
  Give the dashboard the pin as CHIP:LINE. For example, if your button is on
  line 26 of gpiochip0:

      --button-gpio 0:26      --led-red 0:16  --led-green 0:20  --led-blue 0:21

  A plain number (26) still means a Raspberry Pi BCM pin, so existing Pi setups
  are unaffected.
EOF

# ---- optional: identify a line by pressing the button ------------------------
if [ "${1:-}" = "watch" ]; then
  step "Finding your button"
  echo "  I'll watch every free line for 20 seconds."
  echo "  PRESS AND HOLD your button a few times now."
  echo
  python3 - <<'PY'
import glob, time
try:
    import lgpio
except ImportError:
    print("  python3-lgpio not installed:  sudo apt install -y python3-lgpio")
    raise SystemExit(1)

watch = []
for dev in sorted(glob.glob('/dev/gpiochip*')):
    n = int(dev.replace('/dev/gpiochip', ''))
    try:
        h = lgpio.gpiochip_open(n)
    except Exception:
        continue
    try:
        nlines = lgpio.gpio_get_chip_info(h)[1]
    except Exception:
        lgpio.gpiochip_close(h); continue
    for line in range(min(nlines, 64)):
        try:
            lgpio.gpio_claim_input(h, line, lgpio.SET_PULL_UP)
            watch.append((n, h, line, lgpio.gpio_read(h, line)))
        except Exception:
            pass          # already owned by the kernel - skip it

print("  watching %d free lines..." % len(watch))
seen = {}
end = time.time() + 20
while time.time() < end:
    for i, (n, h, line, base) in enumerate(watch):
        try:
            v = lgpio.gpio_read(h, line)
        except Exception:
            continue
        if v != base:
            key = (n, line)
            if key not in seen:
                seen[key] = True
                print("  CHANGED:  %d:%d   <-- this looks like your button" % (n, line))
    time.sleep(0.02)

if not seen:
    print("  Nothing changed. Either the button isn't wired to a free line,")
    print("  or it needs an external pull-up resistor.")
else:
    print()
    print("  Use the number above, e.g.  --button-gpio %d:%d" % list(seen)[0])
PY
fi
