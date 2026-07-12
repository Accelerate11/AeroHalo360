#!/usr/bin/env bash

# ROS 2 Humble 的 setup.bash 不是 nounset 安全的，调用方必须保持 set -u 关闭。
source_ros_environment() {
  local ros_setup="$1"
  if [[ ! -f "$ros_setup" ]]; then
    echo "错误：未找到 ROS 2 环境脚本：$ros_setup" >&2
    return 20
  fi
  # ROS 安装路径由调用方校验后传入。
  # shellcheck disable=SC1090
  source "$ros_setup"
}

show_rosdep_sources() {
  local source_dir="/etc/ros/rosdep/sources.list.d"
  if [[ ! -d "$source_dir" ]]; then
    echo "提示：rosdep 源目录不存在：$source_dir" >&2
    return 0
  fi

  local suspicious
  suspicious="$(grep -RInE 'ghproxy|raw\.githubusercontent\.com|githubusercontent' "$source_dir" 2>/dev/null || true)"
  if [[ -n "$suspicious" ]]; then
    echo "检测到可能受网络/DNS影响的 rosdep 源（只报告，不会修改）：" >&2
    echo "$suspicious" >&2
  fi
}

print_rosdep_recovery() {
  local workspace_root="$1"
  cat >&2 <<EOF
rosdep 未完成。请检查上方网络错误和 /etc/ros/rosdep/sources.list.d。
网络恢复后可执行：
  rosdep update
  cd "$workspace_root"
  rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
如显式依赖已安装且需要离线构建，可执行：
  bash nano_deploy/02_build_release.sh --skip-rosdep
EOF
}

verify_runtime_dependencies() {
  local missing=0
  local package
  for package in \
    diagnostic_msgs geometry_msgs rclcpp rclpy rosidl_default_generators \
    sensor_msgs sensor_msgs_py std_msgs tf2 tf2_ros visualization_msgs
  do
    if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
      echo "错误：缺少必需 ROS 2 包：$package" >&2
      missing=1
    fi
  done

  if ! python3 -c 'import pymavlink' >/dev/null 2>&1; then
    echo "错误：Python 无法导入 pymavlink。" >&2
    missing=1
  fi

  if [[ "$missing" -ne 0 ]]; then
    return 21
  fi
  echo "依赖验证通过：ROS 2 包与 pymavlink 均可用。"
}
