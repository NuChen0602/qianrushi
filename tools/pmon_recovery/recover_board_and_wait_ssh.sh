#!/usr/bin/env bash
set -euo pipefail
dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
known_ip=192.168.43.192
timeout=120
wait_only=0
while (($#)); do
  case "$1" in
    --wait-only) wait_only=1 ;;
    --known-ip) known_ip=${2:?--known-ip requires an address}; shift ;;
    --timeout) timeout=${2:?--timeout requires seconds}; shift ;;
    *) echo "Usage: $0 [--wait-only] [--known-ip IP] [--timeout SECONDS]" >&2; exit 64 ;;
  esac
  shift
done
case "$timeout" in *[!0-9]*|'') echo 'timeout must be a non-negative integer' >&2; exit 64;; esac
probe_ssh() {
  ssh -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=5 "root@$1" \
    'echo BOARD_ONLINE; uname -a; cat /proc/uptime'
}
approve_or_report() {
  "$dir/discover_board_ip.sh" --known-ip "$known_ip" || true
  echo 'No SSH connection was made to an unapproved host key.' >&2
}
if nc -z -w 1 "$known_ip" 22 >/dev/null 2>&1; then
  if probe_ssh "$known_ip"; then exit 0; fi
  approve_or_report
  exit 3
fi
if ((wait_only)); then
  echo "Waiting up to ${timeout}s for ${known_ip}:22; serial is not accessed." >&2
  elapsed=0
  while ((elapsed < timeout)); do
    if nc -z -w 1 "$known_ip" 22 >/dev/null 2>&1; then
      if probe_ssh "$known_ip"; then exit 0; fi
      approve_or_report; exit 3
    fi
    sleep 1; ((elapsed += 1))
  done
  approve_or_report
  exit 1
fi
log=$(python3 "$dir/board_recover_serial.py" --dry-run) || exit $?
echo "Serial dry-run log: $log" >&2
state=$(grep -E '^state=SERIAL_STATE_' "$log" | tail -n 1 | sed 's/ .*//; s/^state=//')
case "$state" in
  SERIAL_STATE_PMON)
    echo "PMON observed. User must manually confirm and execute exactly once:" >&2
    echo "sudo python3 $dir/board_recover_serial.py --send-cr" >&2; exit 20 ;;
  SERIAL_STATE_SILENT)
    echo "No serial bytes observed. Given the on-site finding, user may manually confirm exactly once:" >&2
    echo "sudo python3 $dir/board_recover_serial.py --send-cr --force-single-cr-on-silent" >&2; exit 21 ;;
  SERIAL_STATE_LINUX_BOOTING)
    echo "Linux boot markers observed; run $0 --wait-only --known-ip $known_ip --timeout $timeout" >&2; exit 22 ;;
  *) echo "Serial output is unknown; no byte was written. Review $log" >&2; exit 23 ;;
esac
