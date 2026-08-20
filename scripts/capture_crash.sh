#!/usr/bin/env bash
# =============================================================================
# capture_crash.sh — record everything while you deliberately break the unit.
#
#   bash scripts/capture_crash.sh
#   ... now go and change the setting that crashes it ...
#   ... wait until it has misbehaved, then press Ctrl-C ...
#
# Writes ONE file you can send me: /tmp/mapper_crash_<timestamp>.txt
#
# Why a script rather than "send me the journal": by the time you notice
# something has gone wrong, the interesting part has usually scrolled past, and
# `journalctl --since` needs you to already know when it happened. This starts
# recording BEFORE the event, so the moment itself is always in the file.
#
# It samples three things every 2 s, because the question is which one moves
# first:
#   - the process list (are children piling up? is anything stuck in state D?)
#   - who currently holds the GPS serial port
#   - memory, so a slow leak would be visible as a trend rather than a guess
# and follows the service journal live alongside them.
#
# Nothing here changes the unit. It only watches.
# =============================================================================
set -u

GREEN='\033[1;32m'; YELLOW='\033[1;33m'; BLUE='\033[1;34m'; NC='\033[0m'
ok()   { echo -e "${GREEN}  ✓ $*${NC}"; }
# fuser needs root to see other users' processes. If sudo has lapsed, say that
# plainly rather than letting sudo's error land in the middle of the evidence.
holders() {
  if sudo -n true 2>/dev/null; then
    sudo -n fuser -v "$1" 2>&1 || echo "  (nothing has $1 open)"
  else
    echo "  (skipped - sudo not available at this moment)"
  fi
}
warn() { echo -e "${YELLOW}  ! $*${NC}"; }
step() { echo -e "\n${BLUE}==== $* ====${NC}"; }

STAMP=$(date +%Y-%m-%d_%H-%M-%S)
OUT="/tmp/mapper_crash_${STAMP}.txt"
GPS_PORT="${1:-/dev/gps}"

# journalctl -f and fuser want privileges; ask once, up front, rather than
# stalling for a password halfway through the event we are trying to catch.
if ! sudo -n true 2>/dev/null; then
  echo "This needs sudo (to follow the service journal). Asking once now:"
  sudo -v || { echo "no sudo - aborting"; exit 1; }
fi

{
  echo "=============================================================="
  echo "mapper crash capture — started $(date -Is)"
  echo "=============================================================="
  echo
  echo "---- uname ----"; uname -a
  echo "---- uptime ----"; uptime
  echo "---- git HEAD ----"
  git -C "$(cd "$(dirname "$0")/.." && pwd)" log --oneline -1 2>/dev/null
  echo "---- service ----"
  systemctl is-active mapper-field 2>/dev/null
  echo
  echo "BASELINE (before you touch anything)"
  echo "---- mapper_web processes ----"
  ps -o pid,ppid,stat,wchan:20,etime,rss,cmd -C mapper_web 2>/dev/null
  echo "---- who holds $GPS_PORT ----"
  holders "$GPS_PORT"
  echo "---- memory ----"
  free -m
  echo
} > "$OUT"

step "Recording"
echo "  file: $OUT"
echo "  GPS port watched: $GPS_PORT"
echo
ok "Recording has started."
echo
echo "  NOW go and do the thing that breaks it (change the return type)."
echo "  Leave this running while you do it, and for ~30 s afterwards."
echo
warn "Press Ctrl-C when you are done. Then send me $OUT"
echo

# Follow the journal from this moment on, into the same file.
sudo -n journalctl -u mapper-field -f --since "now" >> "$OUT" 2>&1 &
JPID=$!

cleanup() {
  trap - INT TERM          # a second Ctrl-C must not re-enter this
  kill "$JPID" 2>/dev/null
  kill "${SLEEP_PID:-}" 2>/dev/null
  {
    echo
    echo "=============================================================="
    echo "FINAL STATE — $(date -Is)"
    echo "=============================================================="
    echo "---- mapper_web processes ----"
    ps -o pid,ppid,stat,wchan:20,etime,rss,cmd -C mapper_web 2>/dev/null
    echo "---- all processes in the service cgroup ----"
    systemctl status mapper-field --no-pager -n 0 2>/dev/null | sed -n '1,40p'
    echo "---- who holds $GPS_PORT ----"
    holders "$GPS_PORT"
    echo "---- memory ----"
    free -m
    echo
    echo "capture ended $(date -Is)"
  } >> "$OUT"
  echo
  ok "Saved: $OUT"
  echo
  echo "  Quick look at what changed — process count over the run:"
  grep -c "^SAMPLE" "$OUT" 2>/dev/null | sed 's/^/    samples taken: /'
  echo "  Send me that file."
  exit 0
}
trap cleanup INT TERM

# Sample loop. Deliberately terse per sample so the file stays readable.
while true; do
  {
    echo "SAMPLE $(date +%H:%M:%S)"
    # -C matches by process name; the count is the number that matters.
    N=$(ps -o pid= -C mapper_web 2>/dev/null | grep -c '[0-9]')
    D=$(ps -o stat= -C mapper_web 2>/dev/null | grep -c '^D')
    MEM=$(free -m | awk '/^Mem:/ {print $3"MB used, "$7"MB available"}')
    echo "  mapper_web processes: $N   (stuck in D: $D)   $MEM"
    if [ "$N" -gt 1 ]; then
      ps -o pid,ppid,stat,wchan:20,etime,rss,cmd -C mapper_web 2>/dev/null | sed 's/^/    /'
    fi
  } >> "$OUT"
  # `sleep 2` as a plain foreground command would make bash defer the Ctrl-C
  # trap until it finished, and on a bad day that means the final snapshot -
  # the most valuable part of the file - never gets written at all. Backgrounded
  # and waited on, the trap fires the instant the key is pressed.
  sleep 2 & SLEEP_PID=$!
  wait "$SLEEP_PID" 2>/dev/null
done
