#!/bin/bash
set -eo pipefail

BOARD_IP="${BOARD_IP:-192.168.123.70}"
BOARD_DIR="/home/root/robot_2k0301"
SPEED="${1:-40}"

echo "键盘遥控："
echo "  按住 W / S：前进 / 后退"
echo "  A / D：左转 / 右转（每次 2 度）"
echo "  C：舵机回正"
echo "  + / -：速度加 / 减 5"
echo "  空格或 X：立即停车"
echo "  Q：停车并退出"
echo

exec ssh -t "root@${BOARD_IP}" \
    "cd '${BOARD_DIR}' && exec ./robot_board_app --test teleop --speed '${SPEED}' --stop-distance 500 --slow-distance 800"
