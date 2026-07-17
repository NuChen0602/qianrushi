现在codex额度52%，用到25%以下就别用了


连接wifi的步骤：
usb接上电脑，直接在终端运行以下脚本：
cat > ~/pmon_fallback_enter.sh <<'EOF'
#!/usr/bin/env bash
set -e

SERIAL="/dev/ttyACM0"
IFACE="wlp0s20f3"
BOARD_MAC="14:0a:02:12:da:3a"
SSH_USER="root"

echo "[1/6] 清理串口占用..."
sudo fuser -k "$SERIAL" 2>/dev/null || true
sudo systemctl stop ModemManager 2>/dev/null || true

echo "[2/6] 启动 PMON fallback 自动 Enter 脚本..."
sudo python3 - <<PY &
import os, sys, time, termios, select

dev = "$SERIAL"
fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

attrs = termios.tcgetattr(fd)
attrs[0] = 0
attrs[1] = 0
attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
attrs[3] = 0
attrs[4] = termios.B115200
attrs[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, attrs)

print("\\n[串口] 已打开。现在按一下板子的 RESET。\\n", flush=True)

buf = b""
sent_c = False
sent_enter = False
start = time.time()
last_enter = 0

while time.time() - start < 120:
    try:
        r, _, _ = select.select([fd], [], [], 0.03)

        if r:
            data = os.read(fd, 4096)
            if data:
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
                buf = (buf + data)[-6000:]

                if (not sent_c) and (b"Press 'c' to command-line" in buf or b"Boot Menu List" in buf):
                    os.write(fd, b"c")
                    print("\\n\\n[串口] 已发送 c，等待 AUTO Press <Enter>...\\n", flush=True)
                    sent_c = True

                if (not sent_enter) and (b"Press <Enter>" in buf or b"AUTO" in buf):
                    print("\\n\\n[串口] 看到 AUTO/Press Enter，开始连续发送 Enter...\\n", flush=True)
                    for _ in range(30):
                        os.write(fd, b"\\r")
                        time.sleep(0.05)
                    sent_enter = True

        if sent_c and not sent_enter and time.time() - last_enter > 0.2:
            os.write(fd, b"\\r")
            last_enter = time.time()

    except BlockingIOError:
        pass
    except OSError:
        break

os.close(fd)
PY

BOOT_PID=$!

echo "[3/6] 请现在按一下板子的 RESET。"
echo "[4/6] 等待板子启动并连接 Wi-Fi..."

BOARD_IP=""
for i in $(seq 1 100); do
    BOARD_IP=$(sudo arp-scan --interface="$IFACE" --localnet 2>/dev/null | awk -v mac="$BOARD_MAC" 'tolower($2)==mac {print $1; exit}')
    if [ -n "$BOARD_IP" ]; then
        echo
        echo "[5/6] 找到板子: $BOARD_IP  $BOARD_MAC"
        break
    fi
    sleep 2
done

kill "$BOOT_PID" 2>/dev/null || true

if [ -z "$BOARD_IP" ]; then
    echo
    echo "没有扫到板子。当前热点设备："
    sudo arp-scan --interface="$IFACE" --localnet || true
    echo
    echo "这次没有成功进入 Linux。"
    exit 1
fi

echo "[6/6] 开始 SSH..."
ssh -o StrictHostKeyChecking=accept-new "$SSH_USER@$BOARD_IP"
EOF

chmod +x ~/pmon_fallback_enter.sh
~/pmon_fallback_enter.sh


程序启动步骤：
运行/home/chen/Library_Patrol_Project/library_patrol_final_demo/scripts/start_full_demo.sh

完整链路已经测试