#!/bin/sh
# Install a non-blocking, retrying Wi-Fi startup service on the LS2K0301 Buildroot image.
set -eu

if [ "$(id -u)" -ne 0 ]; then
  echo "run this installer as root"
  exit 1
fi
if [ ! -s /etc/wpa_supplicant.conf ] ||
   ! grep -q '^[[:space:]]*ssid=' /etc/wpa_supplicant.conf; then
  echo "/etc/wpa_supplicant.conf is missing or has no ssid"
  echo "configure the hotspot SSID/password first; this installer never prints or replaces the password"
  exit 1
fi
if ! command -v wpa_supplicant >/dev/null 2>&1; then
  echo "wpa_supplicant is not installed in the board image"
  exit 1
fi
if ! command -v udhcpc >/dev/null 2>&1; then
  echo "udhcpc is not installed in the board image"
  exit 1
fi

mkdir -p /usr/local/sbin /var/log /var/run
cat > /usr/local/sbin/robot_network_up.sh <<'WORKER'
#!/bin/sh
CONF=/etc/wpa_supplicant.conf
PIDFILE=/var/run/robot-network-up.pid
LOG=/var/log/robot-network.log

if [ -r "${PIDFILE}" ]; then
  old_pid="$(cat "${PIDFILE}" 2>/dev/null || true)"
  if [ -n "${old_pid}" ] && kill -0 "${old_pid}" 2>/dev/null; then exit 0; fi
fi
echo $$ > "${PIDFILE}"
trap 'rm -f "${PIDFILE}"' EXIT INT TERM

find_iface() {
  if [ -n "${ROBOT_WIFI_IFACE:-}" ] && [ -d "/sys/class/net/${ROBOT_WIFI_IFACE}" ]; then
    echo "${ROBOT_WIFI_IFACE}"
    return
  fi
  for path in /sys/class/net/wlan* /sys/class/net/wlp*; do
    if [ -d "${path}" ]; then basename "${path}"; return; fi
  done
}

load_wifi_drivers() {
  module_dir="/usr/lib/modules/$(uname -r)"
  [ -d /sys/class/net/wlan0 ] && return
  [ -f "${module_dir}/aic8800_bsp.ko" ] && insmod "${module_dir}/aic8800_bsp.ko" 2>/dev/null || true
  [ -f "${module_dir}/aic8800_fdrv.ko" ] && insmod "${module_dir}/aic8800_fdrv.ko" 2>/dev/null || true
}

load_wifi_drivers
while true; do
  iface="$(find_iface)"
  if [ -z "${iface}" ]; then
    echo "$(date '+%F %T') waiting for Wi-Fi interface" >> "${LOG}"
    sleep 5
    continue
  fi
  echo "$(date '+%F %T') connecting interface=${iface}" >> "${LOG}"
  if command -v ip >/dev/null 2>&1; then ip link set "${iface}" up || true; else ifconfig "${iface}" up || true; fi
  killall wpa_supplicant 2>/dev/null || true
  wpa_supplicant -B -i "${iface}" -c "${CONF}" -D nl80211 >> "${LOG}" 2>&1 || true
  sleep 4
  udhcpc -i "${iface}" -n -q -t 5 -T 3 >> "${LOG}" 2>&1 || true
  if command -v ip >/dev/null 2>&1; then
    ip -4 address show dev "${iface}" 2>/dev/null | grep -q 'inet ' && exit 0
  else
    ifconfig "${iface}" 2>/dev/null | grep -q 'inet addr:' && exit 0
  fi
  echo "$(date '+%F %T') no address; retrying in 10 seconds" >> "${LOG}"
  killall wpa_supplicant 2>/dev/null || true
  sleep 10
done
WORKER
chmod 0755 /usr/local/sbin/robot_network_up.sh

cat > /etc/init.d/S96_robot_network <<'SERVICE'
#!/bin/sh
case "${1:-start}" in
  start)
    /usr/local/sbin/robot_network_up.sh >/dev/null 2>&1 &
    ;;
  stop)
    if [ -r /var/run/robot-network-up.pid ]; then
      kill "$(cat /var/run/robot-network-up.pid)" 2>/dev/null || true
      rm -f /var/run/robot-network-up.pid
    fi
    killall udhcpc 2>/dev/null || true
    killall wpa_supplicant 2>/dev/null || true
    ;;
  restart)
    "$0" stop
    "$0" start
    ;;
  *) echo "Usage: $0 {start|stop|restart}"; exit 1 ;;
esac
SERVICE
chmod 0755 /etc/init.d/S96_robot_network

stamp="$(date +%Y%m%d%H%M%S)"
for old in /etc/init.d/S99wifi_start /etc/init.d/S96_wifi_start; do
  if [ -e "${old}" ]; then
    mv "${old}" "${old}.disabled-${stamp}"
    echo "disabled blocking/legacy startup script: ${old}"
  fi
done

/etc/init.d/S96_robot_network restart
sync
echo "automatic Wi-Fi retry service installed"
echo "log: /var/log/robot-network.log"
echo "wait up to 30 seconds, then run: ip address; ps | grep -E 'wpa|udhcpc|dropbear'"
