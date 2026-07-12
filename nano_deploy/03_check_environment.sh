#!/usr/bin/env bash
# ROS 2 Humble 的环境脚本会读取未预先定义的变量，因此这里不启用 nounset。
set -eo pipefail

SCRIPT_DIR="$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WORKSPACE_ROOT="$(dirname "$(dirname "$PROJECT_ROOT")")"

source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"

echo "系统架构：$(uname -m)"
echo "ROS 发行版：$ROS_DISTRO"
echo "AeroHalo360 安装位置：$(ros2 pkg prefix aero_halo_360)"
python3 -c "import pymavlink; print('pymavlink：可用')"

echo
echo "可见串口设备："
find /dev -maxdepth 1 -type c \( -name 'ttyTHS*' -o -name 'ttyUSB*' -o -name 'ttyACM*' \) -print 2>/dev/null || true

echo
echo "环境检查完成。Livox 驱动启动后，请继续检查："
echo "  ros2 topic type /livox/lidar"
echo "  ros2 topic hz /livox/lidar"
