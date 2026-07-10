#!/bin/bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_ROOT="${PROJECT_ROOT}/.local_tools/ros-humble-local"
ROS_PREFIX="${INSTALL_ROOT}/opt/ros/humble"
INDEX_URL="https://mirrors.ustc.edu.cn/ros2/ubuntu/dists/jammy/main/binary-amd64/Packages.gz"
BASE_URL="https://mirrors.ustc.edu.cn/ros2/ubuntu"
PACKAGES=(ros-humble-nav2-amcl ros-humble-nav2-map-server)

mkdir -p "${INSTALL_ROOT}"
index_file="$(mktemp)"
trap 'rm -f "${index_file}"' EXIT
curl -fsSL "${INDEX_URL}" -o "${index_file}"

for package in "${PACKAGES[@]}"; do
    filename="$(gzip -dc "${index_file}" | awk -v package="${package}" '
        BEGIN { RS=""; FS="\n" }
        $1 == "Package: " package {
            for (i = 1; i <= NF; ++i) {
                if ($i ~ /^Filename: /) {
                    sub(/^Filename: /, "", $i)
                    print $i
                }
            }
        }
    ')"
    if [[ -z "${filename}" ]]; then
        echo "无法在 ROS 2 镜像中找到 ${package}" >&2
        exit 1
    fi
    deb_file="$(mktemp --suffix=.deb)"
    echo "下载 ${package}"
    curl -fsSL "${BASE_URL}/${filename}" -o "${deb_file}"
    dpkg-deb -x "${deb_file}" "${INSTALL_ROOT}"
    rm -f "${deb_file}"
done

echo "定位依赖已安装到 ${ROS_PREFIX}"
