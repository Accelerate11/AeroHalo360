#!/usr/bin/env bash
# 只读健康检查：不重启进程、不修改参数。
set -eo pipefail

FAILURES=0

check_node() {
  local node="$1"
  if ros2 node list 2>/dev/null | grep -qx "$node"; then
    echo "通过：节点存在 $node"
  else
    echo "失败：节点不存在 $node" >&2
    FAILURES=$((FAILURES + 1))
  fi
}

check_single_publisher() {
  local topic="$1"
  local info
  info="$(ros2 topic info "$topic" 2>/dev/null || true)"
  if echo "$info" | grep -Eq 'Publisher count: 1$'; then
    echo "通过：$topic 恰有一个发布者"
  else
    echo "失败：$topic 发布者数量异常" >&2
    echo "$info" >&2
    FAILURES=$((FAILURES + 1))
  fi
}

check_node /cloud_processor_node
check_single_publisher /aero_halo_360/sector_distances

if [[ "$START_MAVLINK" == true ]]; then
  check_node /mavlink_obstacle_sender
fi

if [[ "$STATIC_TF_ENABLED" == true ]]; then
  check_node /aero_halo_360_static_lidar_tf
fi

timeout 3 ros2 topic echo /aero_halo_360/diagnostics --once >/dev/null 2>&1 || {
  echo "失败：无法读取 cloud diagnostics" >&2
  FAILURES=$((FAILURES + 1))
}

if [[ "$START_MAVLINK" == true ]]; then
  timeout 3 ros2 topic echo /aero_halo_360/mavlink_diagnostics --once >/dev/null 2>&1 || {
    echo "失败：无法读取 MAVLink diagnostics" >&2
    FAILURES=$((FAILURES + 1))
  }
fi

if [[ "$FAILURES" -ne 0 ]]; then
  echo "AeroHalo360 健康检查失败：$FAILURES 项" >&2
  exit 1
fi

echo "AeroHalo360 健康检查通过"
