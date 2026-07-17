#!/usr/bin/env bash
# Read-only TCP/22 discovery. It never changes known_hosts or opens SSH sessions.
set -euo pipefail
known_ip=192.168.43.192
accept=0
while (($#)); do
  case "$1" in
    --known-ip) known_ip=${2:?--known-ip requires an address}; shift ;;
    --accept-hostkey) accept=1 ;;
    *) echo "Usage: $0 [--known-ip IP] [--accept-hostkey]" >&2; exit 64 ;;
  esac
  shift
done
port_open() { nc -z -w 1 "$1" 22 >/dev/null 2>&1; }
results=$(mktemp); trap 'rm -f "$results"' EXIT
port_open "$known_ip" && printf '%s\n' "$known_ip" >>"$results"
if ! port_open "$known_ip"; then
  while read -r ip _; do case "$ip" in 192.168.43.*) printf '%s\n' "$ip" >>"$results";; esac; done < <(ip neigh 2>/dev/null || true)
  for n in $(seq 1 254); do
    ip="192.168.43.$n"
    (port_open "$ip" && printf '%s\n' "$ip" >>"$results") &
    while (( $(jobs -pr | wc -l) >= 16 )); do wait -n || true; done
  done
  wait || true
fi
found=0
while read -r ip; do
  found=1
  keys=$(ssh-keyscan -T 3 "$ip" 2>/dev/null || true)
  [[ -n $keys ]] || { echo "Candidate $ip has port 22 but no keyscan result" >&2; continue; }
  candidate_fp=$(printf '%s\n' "$keys" | ssh-keygen -lf - 2>/dev/null | awk '{print $2, $4}' | sort -u)
  known_keys=$(ssh-keygen -F "$ip" 2>/dev/null | awk '!/^#/' || true)
  known_fp=$(printf '%s\n' "$known_keys" | ssh-keygen -lf - 2>/dev/null | awk '{print $2, $4}' | sort -u)
  echo "Candidate $ip SSH host-key algorithm/fingerprint:" >&2
  printf '%s\n' "$candidate_fp" >&2
  approved=no
  [[ -n $candidate_fp && -n $known_fp && $candidate_fp == "$known_fp" ]] && approved=yes
  if ((accept)); then
    echo "--accept-hostkey does not write known_hosts or open SSH; fingerprint remains user-review only." >&2
  fi
  printf 'BOARD_IP=%s\nHOSTKEY_APPROVED=%s\n' "$ip" "$approved"
done < <(sort -Vu "$results")
((found)) || { echo 'No TCP/22 candidate found.' >&2; exit 1; }
