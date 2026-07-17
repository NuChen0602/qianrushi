#!/bin/sh
# Read-only network diagnosis for the LS2K0301 Buildroot image.

echo "=== system ==="
date
uname -a
uptime
echo "cmdline: $(cat /proc/cmdline 2>/dev/null)"

echo "=== interfaces ==="
if command -v ip >/dev/null 2>&1; then
  ip -brief link 2>/dev/null || ip link
  ip -brief address 2>/dev/null || ip address
  ip route
else
  ifconfig -a
  route -n
fi

echo "=== wireless ==="
if command -v iw >/dev/null 2>&1; then iw dev; fi
if command -v rfkill >/dev/null 2>&1; then rfkill list; fi
if command -v wpa_cli >/dev/null 2>&1; then
  wpa_cli -i "${ROBOT_WIFI_IFACE:-wlan0}" status 2>/dev/null || true
fi

echo "=== wpa configuration (password redacted) ==="
if [ -r /etc/wpa_supplicant.conf ]; then
  sed 's/^[[:space:]]*psk=.*/    psk=***REDACTED***/' /etc/wpa_supplicant.conf
else
  echo "MISSING: /etc/wpa_supplicant.conf"
fi

echo "=== boot scripts ==="
ls -l /etc/init.d/S*wifi* /etc/init.d/S*wpa* /etc/init.d/S*dropbear* /etc/init.d/S*ssh* 2>/dev/null || true
for script in /etc/init.d/S*wifi* /etc/init.d/S*wpa*; do
  if [ -f "${script}" ]; then
    echo "--- ${script} ---"
    sed -n '1,160p' "${script}"
  fi
done

echo "=== processes and listeners ==="
ps | grep -E 'wpa_supplicant|udhcpc|dropbear|sshd' | grep -v grep || true
if command -v ss >/dev/null 2>&1; then
  ss -ltnp
elif command -v netstat >/dev/null 2>&1; then
  netstat -ltnp
fi

echo "=== recent kernel messages ==="
dmesg 2>/dev/null | grep -Ei 'wlan|wifi|80211|firmware|usb|dropbear' | tail -80 || true

has_ip=0
if command -v ip >/dev/null 2>&1; then
  ip -4 address show 2>/dev/null | grep -q 'inet ' && has_ip=1
else
  ifconfig 2>/dev/null | grep -q 'inet addr:' && has_ip=1
fi
if [ "${has_ip}" -eq 1 ]; then
  echo "RESULT: at least one IPv4 address is configured"
  exit 0
fi
echo "RESULT: no IPv4 address; automatic Wi-Fi/DHCP startup is not working"
exit 1
