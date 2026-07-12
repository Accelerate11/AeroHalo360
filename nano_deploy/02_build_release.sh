#!/usr/bin/env bash
# ROS 2 Humble 的环境脚本会读取未预先定义的变量，因此这里不启用 nounset。
set -eo pipefail

SCRIPT_DIR="$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WORKSPACE_ROOT="$(dirname "$(dirname "$PROJECT_ROOT")")"
ROS_SETUP="/opt/ros/humble/setup.bash"
ROSDEP_MODE=best-effort

usage() {
  cat <<'EOF'
用法：02_build_release.sh [选项]

选项：
  --strict-rosdep  rosdep install/check 失败时立即退出
  --skip-rosdep    离线构建；跳过 rosdep，只验证显式依赖
  --offline        --skip-rosdep 的别名
  -h, --help       显示帮助
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

if [[ ! -f "$PROJECT_ROOT/aero_halo_360/package.xml" ]]; then
  echo "错误：部署包目录结构不正确：$PROJECT_ROOT" >&2
  exit 10
fi

cd "$WORKSPACE_ROOT"

if [[ "$ROSDEP_MODE" != "skip" ]]; then
  show_rosdep_sources
  if ! rosdep install \
    --from-paths src \
    --ignore-src \
    -r -y \
    --rosdistro humble \
    --skip-keys "ament_cmake_gtest ament_cmake_test"
  then
    print_rosdep_recovery "$WORKSPACE_ROOT"
    if [[ "$ROSDEP_MODE" == "strict" ]]; then
      exit 31
    fi
  fi
else
  echo "已按要求跳过 rosdep install，开始显式依赖验证。"
fi

verify_runtime_dependencies

# 生产构建关闭测试，并使用串行执行器降低 Nano 的峰值内存。
colcon build \
  --packages-select aero_halo_360 \
  --executor sequential \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF

source "$WORKSPACE_ROOT/install/setup.bash"
ros2 pkg prefix aero_halo_360

echo
echo "Release 编译完成。新终端中请执行："
echo "  source /opt/ros/humble/setup.bash"
echo "  source $WORKSPACE_ROOT/install/setup.bash"
