#!/usr/bin/env bash
set -eo pipefail

ROS_APT_SOURCE_VERSION="1.1.0"
ROS_APT_SOURCE_SHA256="1600cb8cc28258a39bffc1736a75bcbf52d1f2db371a4d020c1b187d2a5a083b"

usage() {
  cat <<'EOF'
用法：00_install_ros2_humble.sh [--help]

在 aarch64 Ubuntu 22.04 上安装 ROS 2 Humble。
ros-apt-source 固定为 1.1.0，并在 dpkg 安装前校验 SHA-256。
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo "错误：未知参数：$*" >&2
  usage >&2
  exit 2
fi

ARCH="$(uname -m)"
. /etc/os-release
if [[ "$ARCH" != "aarch64" ]]; then
  echo "错误：当前架构为 $ARCH，预期为 aarch64。" >&2
  exit 1
fi
if [[ "$VERSION_ID" != "22.04" || "$VERSION_CODENAME" != "jammy" ]]; then
  echo "错误：本脚本只支持 Ubuntu 22.04 Jammy，当前为 $PRETTY_NAME。" >&2
  exit 1
fi

sudo apt update
sudo apt install -y software-properties-common curl ca-certificates
sudo add-apt-repository -y universe

DEB_NAME="ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb"
DEB_PATH="/tmp/$DEB_NAME"
DEB_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/$DEB_NAME"

curl --fail --location --retry 3 --output "$DEB_PATH" "$DEB_URL"
echo "$ROS_APT_SOURCE_SHA256  $DEB_PATH" | sha256sum --check -
dpkg-deb --info "$DEB_PATH" >/dev/null
sudo dpkg -i "$DEB_PATH"

sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools

echo
echo "ROS 2 Humble 安装完成。后续命令："
echo "  source /opt/ros/humble/setup.bash"
echo "  bash nano_deploy/01_install_dependencies.sh"
