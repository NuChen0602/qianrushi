#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TARGET_DIR="${PROJECT_ROOT}/third_party/LS2K0301_Library"

if ! command -v git >/dev/null 2>&1; then
  echo "git command not found. Please install git first."
  exit 1
fi

if [ -d "${TARGET_DIR}/.git" ]; then
  echo "LS2K0301_Library already exists, pulling latest changes..."
  git -C "${TARGET_DIR}" pull --ff-only
else
  git clone https://gitee.com/seekfree/LS2K0301_Library.git "${TARGET_DIR}"
fi

echo "LS2K0301_Library is ready at ${TARGET_DIR}"

