#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 root@BOARD_IP"
  exit 2
fi
TARGET="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ARCHIVE="${PROJECT_ROOT}/build/ls2k0301/robot_2k0301-board.tar.gz"

"${SCRIPT_DIR}/package_ls2k0301.sh"
scp "${ARCHIVE}" "${TARGET}:/tmp/robot_2k0301-board.tar.gz"
ssh "${TARGET}" 'mkdir -p /root/robot_2k0301 && gzip -dc /tmp/robot_2k0301-board.tar.gz | tar -xf - -C /root && chmod +x /root/robot_2k0301/robot_board_* /root/robot_2k0301/scripts/*.sh'
echo "deployed to ${TARGET}:/root/robot_2k0301"
echo "next: ssh ${TARGET} 'cd /root/robot_2k0301 && ./scripts/board_preflight.sh'"
