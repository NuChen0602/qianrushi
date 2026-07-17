#!/bin/sh
# REFERENCE_ONLY_NOT_EXECUTED in round 2. Do not copy to or run on the board.
set -eu
backup=${1:?Usage: $0 /boot/boot.cfg.before_showmenu0_TIMESTAMP}
cfg=/boot/boot.cfg
case "$backup" in /boot/boot.cfg.before_showmenu0_*) ;; *) echo 'refuse: valid backup path required'; exit 1;; esac
[ -f "$backup" ] || { echo 'refuse: backup does not exist'; exit 1; }
expected=$(sha256sum "$backup" | awk '{print $1}')
tmp="/boot/.boot.cfg.rollback.$$"
cp -p "$backup" "$tmp"; mv "$tmp" "$cfg"; sync
actual=$(sha256sum "$cfg" | awk '{print $1}')
[ "$expected" = "$actual" ] || { echo 'rollback hash mismatch'; exit 1; }
printf 'sha256=%s\n' "$actual"; cat "$cfg"
