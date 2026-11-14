#!/bin/bash
# timesync.sh: keep the clocks of the robot machines within milliseconds of each other (B-06). Two PCs that share a task
# (cluster heartbeats, LLM failover, rosbag replays) need this; a drifting laptop makes timestamps and TF lookups lie.
#
#   tools/net/timesync.sh server 192.168.1.0/24        # on the machine that is the time source (e.g. the robot laptop)
#   tools/net/timesync.sh client 192.168.1.10          # on every other machine: follow that server
#   tools/net/timesync.sh status                       # offset and sources (needs chrony)
# `server` and `client` PRINT the chrony config and the commands; they change the system only with --apply (uses sudo).
# The server also serves time when it has no internet (`local stratum 10`), so the pair stays consistent offline.
set -euo pipefail
MODE="${1:-}"; ARG="${2:-}"; APPLY=0; for a in "$@"; do [[ "$a" == "--apply" ]] && APPLY=1; done
CONF=/etc/chrony/conf.d/robocup.conf
case "$MODE" in
  server)
    [[ -n "$ARG" && "$ARG" != --apply ]] || { echo "usage: $0 server <subnet, e.g. 192.168.1.0/24> [--apply]" >&2; exit 2; }
    CFG=$'# robocup: serve time to the robot LAN\nallow '"$ARG"$'\nlocal stratum 10\n' ;;
  client)
    [[ -n "$ARG" && "$ARG" != --apply ]] || { echo "usage: $0 client <server ip> [--apply]" >&2; exit 2; }
    CFG=$'# robocup: follow the robot time server\nserver '"$ARG"$' iburst prefer\nmakestep 0.5 3\n' ;;
  status)
    command -v chronyc >/dev/null || { echo "chrony is not installed (sudo apt install chrony)"; exit 1; }
    chronyc tracking; chronyc sources; exit 0 ;;
  *) sed -n 2,10p "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
echo "--- $CONF"; printf '%s' "$CFG"; echo "---"
CMDS=("sudo apt install -y chrony" "printf '%s' \"\$CFG\" | sudo tee $CONF" "sudo systemctl restart chrony" "chronyc tracking")
if [[ $APPLY -eq 1 ]]; then
  command -v chronyc >/dev/null || sudo apt install -y chrony
  sudo mkdir -p /etc/chrony/conf.d
  printf '%s' "$CFG" | sudo tee "$CONF" >/dev/null
  grep -q "confdir /etc/chrony/conf.d" /etc/chrony/chrony.conf 2>/dev/null || \
    echo "note: add 'confdir /etc/chrony/conf.d' (or the file's content) to /etc/chrony/chrony.conf if your chrony does not read conf.d"
  sudo systemctl restart chrony
  sleep 3; chronyc tracking | head -4
else
  echo "not applied. To apply: re-run with --apply, or by hand:"; printf '  %s\n' "sudo apt install -y chrony" "sudo tee $CONF   # paste the lines above" "sudo systemctl restart chrony" "tools/net/timesync.sh status"
fi
