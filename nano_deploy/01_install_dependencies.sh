#!/usr/bin/env bash
# ROS 2 Humble 的环境脚本会读取未预先定义的变量，因此这里不启用 nounset。
set -eo pipefail

SCRIPT_DIR="$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WORKSPACE_ROOT="$(dirname "$(dirname "$PROJECT_ROOT")")"
ROS_DISTRO=humble
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
ROSDEP_MODE=best-effort

usage() {
  cat <<'EOF'
用法：01_install_dependencies.sh [选项]

选项：
  --strict-rosdep  rosdep update 失败时立即以非零状态退出
  --skip-rosdep    不访问 rosdep 网络，只安装并验证显式依赖
  --offline        --skip-rosdep 的别名
  -h, --help       显示帮助

默认模式会先完成 APT 与 pymavlink，再尝试 rosdep update。rosdep 网络失败时，
脚本打印原因和恢复命令；只有实际必需依赖验证通过才会成功退出。
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --strict-rosdep) ROSDEP_MODE=strict ;;
    --skip-rosdep|--offline) ROSDEP_MODE=skip ;;
    -h|--help) usage; exit 0 ;;
    *) echo "错误：未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

source "$SCRIPT_DIR/lib/common.sh"
source_ros_environment "$ROS_SETUP"

sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  ros-humble-diagnostic-msgs \
  ros-humble-geometry-msgs \
  ros-humble-rclcpp \
  ros-humble-rclpy \
  ros-humble-rosidl-default-generators \
  ros-humble-sensor-msgs \
  ros-humble-sensor-msgs-py \
  ros-humble-std-msgs \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-visualization-msgs

# Ubuntu 22.04 允许用户级 pip 安装；不写入 ROS 或系统 Python 目录。
python3 -m pip install --user --upgrade pymavlink
python3 -c 'import pymavlink; print("pymavlink 导入验证通过")'

if [[ "$ROSDEP_MODE" != "skip" ]]; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  show_rosdep_sources
  if ! rosdep update; then
    print_rosdep_recovery "$WORKSPACE_ROOT"
    if [[ "$ROSDEP_MODE" == "strict" ]]; then
      exit 30
    fi
  fi
else
  echo "已按要求跳过 rosdep 网络更新。"
fi

verify_runtime_dependencies

echo
echo "AeroHalo360 编译依赖已安装并验证。"
echo "若使用 UART/USB 串口，请执行：sudo usermod -aG dialout $USER"
echo "执行后注销并重新登录，组权限才会生效。"
