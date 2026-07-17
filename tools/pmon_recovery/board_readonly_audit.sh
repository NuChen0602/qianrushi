#!/usr/bin/env bash
# Strictly read-only remote audit. Full dmesg is saved locally, never on the board.
set -uo pipefail
ip=${1:?Usage: $0 BOARD_IP}
dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
root=$(CDPATH= cd -- "$dir/../.." && pwd)
out="$root/docs/pmon_autoboot_repair_round2"
mkdir -p "$out"
stamp=$(date +%Y%m%d_%H%M%S)
summary="$out/board_audit_${ip}_${stamp}.log"
opts=(-o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=5)
ssh "${opts[@]}" "root@$ip" 'sh -s' <<'REMOTE' | tee "$summary"
run() { echo "===== $1 ====="; shift; "$@" 2>&1 || true; }
run uname uname -a
run proc_version cat /proc/version
run cmdline cat /proc/cmdline
run consoles cat /proc/consoles
run uptime cat /proc/uptime
run cpuinfo cat /proc/cpuinfo
run serial_driver cat /proc/tty/driver/serial
run tty_holders fuser -v /dev/ttyS0 /dev/ttyS1
run ps ps w
run mounts mount
run proc_mounts cat /proc/mounts
run boot_list ls -lah /boot
echo '===== boot_cfg_listing_glob ====='; ls -ln /boot/boot.cfg* 2>&1 || true
run boot_cfg_numbered nl -ba /boot/boot.cfg
echo '===== boot_cfg_hashes ====='; sha256sum /boot/boot.cfg /boot/boot.cfg.* 2>/dev/null || true
for f in /boot/boot.cfg /boot/boot.cfg.*; do [ -f "$f" ] || continue; echo "===== $f ====="; sed -n '1,160p' "$f"; done
for f in /proc/device-tree/chosen/bootargs /proc/device-tree/chosen/stdout-path /proc/device-tree/aliases/serial0 /proc/device-tree/aliases/serial1; do [ -e "$f" ] || continue; echo "===== $f ====="; od -An -tx1 -c "$f" 2>&1 || true; done
run ip_addr ip addr
run ip_route ip route
run wireless cat /proc/net/wireless
echo '===== wifi_processes ====='; ps w | grep -E '[w]pa_supplicant|[u]dhcpc' || true
echo '===== selected_dmesg ====='; dmesg 2>/dev/null | grep -Ei 'pmon|bios|dmi|wlan|wifi|firmware|sdio|ttyS|console|bootconsole' || true
REMOTE
ssh "${opts[@]}" "root@$ip" 'dmesg' >"$out/dmesg_${ip}_${stamp}.log" 2>&1 || true
echo "summary=$summary"
echo "dmesg=$out/dmesg_${ip}_${stamp}.log"
