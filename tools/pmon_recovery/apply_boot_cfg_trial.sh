#!/bin/sh
# REFERENCE_ONLY_NOT_EXECUTED in round 2. Do not copy to or run on the board.
set -eu
cfg=/boot/boot.cfg
stamp=$(date +%Y%m%d_%H%M%S)
backup="/boot/boot.cfg.before_showmenu0_$stamp"
tmp="/boot/.boot.cfg.$stamp.tmp"
[ "$(grep -Ec '^timeout[[:space:]]+3[[:space:]]*$' "$cfg")" = 1 ] || { echo 'refuse: timeout must be 3'; exit 1; }
[ "$(grep -Ec '^default[[:space:]]+0[[:space:]]*$' "$cfg")" = 1 ] || { echo 'refuse: default must be 0'; exit 1; }
[ "$(grep -Ec '^showmenu[[:space:]]+1[[:space:]]*$' "$cfg")" = 1 ] || { echo 'refuse: showmenu must be 1'; exit 1; }
first=$(awk '/^title[[:space:]]/{if (seen++) exit} /^title[[:space:]]/{seen=1} seen{print}' "$cfg")
printf '%s\n' "$first" | grep -Eq '^[[:space:]]*kernel[[:space:]]+\(emmc0,0\)/boot/vmlinuz[[:space:]]*$' || { echo 'refuse: first kernel is not standard vmlinuz'; exit 1; }
printf '%s\n' "$first" | grep -E '^[[:space:]]*args[[:space:]]+' | grep -Eq 'console=ttyS0([ ,]|$)' && { echo 'refuse: default args contain console=ttyS0'; exit 1; }
cp -p "$cfg" "$backup"
before=$(sha256sum "$cfg" | awk '{print $1}')
sed 's/^showmenu[[:space:]]\{1,\}1[[:space:]]*$/showmenu 0/' "$cfg" >"$tmp"
mv "$tmp" "$cfg"; sync
after=$(sha256sum "$cfg" | awk '{print $1}')
printf 'backup=%s\nbefore_sha256=%s\nafter_sha256=%s\n' "$backup" "$before" "$after"
cat "$cfg"
