#!/usr/bin/env bash
# 本脚本默认只读，不修改 IP、飞控参数、rosdep 源，也不终止任何进程。
set -eo pipefail

SCRIPT_DIR="$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
DEFAULT_WORKSPACE="$(dirname "$(dirname "$PROJECT_ROOT")")"

INTERFACE=""
HOST_IP=""
LIDAR_IP=""
LIDAR_TOPIC="/livox/lidar"
SERIAL_DEVICE=""
LIVOX_WORKSPACE=""
AERO_WORKSPACE="$DEFAULT_WORKSPACE"
MAVLINK_CONNECTION=""
TARGET_SYSTEM=1
TARGET_COMPONENT=1
WAIT_HEARTBEAT=false
CHECK_AERO=false
SAMPLE_SECONDS=5
FAILURES=0

usage() {
  cat <<'EOF'
用法：04_preflight.sh [选项]

基础选项：
  --interface NAME          要检查的雷达网卡
  --host-ip IPV4            该网卡应具有的主机 IPv4
  --lidar-ip IPV4           只读 ping/邻居检查的雷达地址
  --lidar-topic TOPIC       PointCloud2 话题，默认 /livox/lidar
  --livox-workspace PATH    Livox 驱动工作空间，用于加载 install/setup.bash
  --aero-workspace PATH     AeroHalo360 工作空间
  --sample-seconds N        话题频率采样秒数，默认 5

飞控选项：
  --serial-device PATH      稳定串口路径，推荐 /dev/serial/by-id/...
  --mavlink-connection STR  pymavlink 连接字符串
  --wait-heartbeat          等待并验证目标飞控 heartbeat
  --target-system ID        目标飞控 sysid，默认 1
  --target-component ID     目标飞控 compid，默认 1

AeroHalo360：
  --check-aero              检查扇区频率和两个 diagnostics
  -h, --help                显示帮助

所有检查均为只读。未提供的硬件参数不会被猜测或写入系统。
EOF
}

fail() {
  echo "失败：$*" >&2
  FAILURES=$((FAILURES + 1))
}

pass() {
  echo "通过：$*"
}

require_value() {
  if [[ "$#" -lt 2 || -z "$2" ]]; then
    echo "错误：$1 缺少参数值" >&2
    exit 2
  fi
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --interface) require_value "$@"; INTERFACE="$2"; shift ;;
    --host-ip) require_value "$@"; HOST_IP="$2"; shift ;;
    --lidar-ip) require_value "$@"; LIDAR_IP="$2"; shift ;;
    --lidar-topic) require_value "$@"; LIDAR_TOPIC="$2"; shift ;;
    --serial-device) require_value "$@"; SERIAL_DEVICE="$2"; shift ;;
    --livox-workspace) require_value "$@"; LIVOX_WORKSPACE="$2"; shift ;;
    --aero-workspace) require_value "$@"; AERO_WORKSPACE="$2"; shift ;;
    --mavlink-connection) require_value "$@"; MAVLINK_CONNECTION="$2"; shift ;;
    --target-system) require_value "$@"; TARGET_SYSTEM="$2"; shift ;;
    --target-component) require_value "$@"; TARGET_COMPONENT="$2"; shift ;;
    --sample-seconds) require_value "$@"; SAMPLE_SECONDS="$2"; shift ;;
    --wait-heartbeat) WAIT_HEARTBEAT=true ;;
    --check-aero) CHECK_AERO=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "错误：未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if ! [[ "$SAMPLE_SECONDS" =~ ^[1-9][0-9]*$ ]]; then
  echo "错误：--sample-seconds 必须是正整数" >&2
  exit 2
fi

ARCH="$(uname -m)"
[[ "$ARCH" == "aarch64" ]] && pass "CPU 架构为 aarch64" || fail "CPU 架构为 $ARCH，预期 aarch64"

if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  [[ "$VERSION_ID" == "22.04" ]] && pass "Ubuntu 22.04" || fail "Ubuntu 版本为 $VERSION_ID"
else
  fail "无法读取 /etc/os-release"
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  fail "缺少 /opt/ros/humble/setup.bash"
else
  source /opt/ros/humble/setup.bash
  [[ "$ROS_DISTRO" == "humble" ]] && pass "ROS_DISTRO=humble" || fail "ROS_DISTRO=$ROS_DISTRO"
fi

if [[ -n "$LIVOX_WORKSPACE" ]]; then
  if [[ -f "$LIVOX_WORKSPACE/install/setup.bash" ]]; then
    source "$LIVOX_WORKSPACE/install/setup.bash"
    pass "已加载 Livox 工作空间"
  else
    fail "Livox 工作空间缺少 install/setup.bash：$LIVOX_WORKSPACE"
  fi
fi

if [[ -f "$AERO_WORKSPACE/install/setup.bash" ]]; then
  source "$AERO_WORKSPACE/install/setup.bash"
fi

ros2 pkg prefix aero_halo_360 >/dev/null 2>&1 &&
  pass "aero_halo_360 包可见" || fail "aero_halo_360 包不可见"
python3 -c 'import pymavlink' >/dev/null 2>&1 &&
  pass "pymavlink 可导入" || fail "pymavlink 无法导入"
ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1 &&
  pass "livox_ros_driver2 包可见" || fail "livox_ros_driver2 包不可见"

if [[ -n "$INTERFACE" ]]; then
  if ip link show dev "$INTERFACE" >/dev/null 2>&1; then
    ip link show dev "$INTERFACE" | grep -q 'state UP' &&
      pass "网卡 $INTERFACE 为 UP" || fail "网卡 $INTERFACE 不是 UP"
    if [[ -n "$HOST_IP" ]]; then
      ip -4 addr show dev "$INTERFACE" | grep -q "inet $HOST_IP/" &&
        pass "$INTERFACE 具有 $HOST_IP" || fail "$INTERFACE 不具有 $HOST_IP"
    fi
  else
    fail "网卡不存在：$INTERFACE"
  fi
elif [[ -n "$HOST_IP" ]]; then
  fail "提供 --host-ip 时也必须提供 --interface"
fi

if [[ -n "$LIDAR_IP" ]]; then
  ping -c 2 -W 1 "$LIDAR_IP" >/dev/null 2>&1 &&
    pass "雷达地址 $LIDAR_IP 可达" || fail "雷达地址 $LIDAR_IP ping 失败"
  echo "邻居表（只读）："
  ip neigh show "$LIDAR_IP" || true
fi

TOPIC_TYPE="$(ros2 topic type "$LIDAR_TOPIC" 2>/dev/null || true)"
if [[ "$TOPIC_TYPE" == "sensor_msgs/msg/PointCloud2" ]]; then
  pass "$LIDAR_TOPIC 类型为 PointCloud2"
else
  [[ -z "$TOPIC_TYPE" ]] && TOPIC_TYPE="不可见"
  fail "$LIDAR_TOPIC 类型为 $TOPIC_TYPE"
fi

TOPIC_INFO="$(ros2 topic info "$LIDAR_TOPIC" 2>/dev/null || true)"
echo "$TOPIC_INFO"
echo "$TOPIC_INFO" | grep -Eq 'Publisher count: [1-9]' &&
  pass "$LIDAR_TOPIC 存在发布者" || fail "$LIDAR_TOPIC 没有发布者"

HZ_OUTPUT="$(timeout "$SAMPLE_SECONDS" ros2 topic hz "$LIDAR_TOPIC" 2>&1 || true)"
echo "$HZ_OUTPUT"
echo "$HZ_OUTPUT" | grep -q 'average rate:' &&
  pass "$LIDAR_TOPIC 能统计到频率" || fail "$LIDAR_TOPIC 未统计到有效频率"

if [[ -n "$SERIAL_DEVICE" ]]; then
  if [[ -e "$SERIAL_DEVICE" ]]; then
    pass "串口存在：$SERIAL_DEVICE -> $(readlink -f "$SERIAL_DEVICE")"
    [[ -r "$SERIAL_DEVICE" && -w "$SERIAL_DEVICE" ]] &&
      pass "当前用户可读写串口" || fail "当前用户不可读写串口"
  else
    fail "串口不存在：$SERIAL_DEVICE"
  fi
  id -nG | tr ' ' '\n' | grep -qx dialout &&
    pass "当前用户属于 dialout" || fail "当前用户不属于 dialout"
  echo "串口占用信息（只读，空输出表示未发现占用）："
  fuser -v "$SERIAL_DEVICE" 2>&1 || true
fi

if [[ "$WAIT_HEARTBEAT" == true ]]; then
  if [[ -z "$MAVLINK_CONNECTION" ]]; then
    fail "--wait-heartbeat 需要 --mavlink-connection"
  elif ros2 pkg prefix aero_halo_360 >/dev/null 2>&1; then
    ros2 run aero_halo_360 check_mavlink.py \
      --connection "$MAVLINK_CONNECTION" \
      --timeout "$SAMPLE_SECONDS" \
      --target-system "$TARGET_SYSTEM" \
      --target-component "$TARGET_COMPONENT" \
      --heartbeat-only &&
      pass "目标飞控 heartbeat 验证通过" || fail "目标飞控 heartbeat 验证失败"
  fi
fi

if [[ "$CHECK_AERO" == true ]]; then
  for topic in \
    /aero_halo_360/diagnostics \
    /aero_halo_360/mavlink_diagnostics
  do
    ros2 topic echo "$topic" --once >/tmp/aero_halo_360_preflight_diag.txt 2>&1 &&
      pass "$topic 可读取" || fail "$topic 不可读取"
  done

  SECTOR_INFO="$(ros2 topic info /aero_halo_360/sector_distances 2>/dev/null || true)"
  echo "$SECTOR_INFO"
  echo "$SECTOR_INFO" | grep -Eq 'Publisher count: 1$' &&
    pass "扇区话题恰有 1 个发布者" || fail "扇区发布者数量不是 1"

  SECTOR_HZ="$(timeout "$SAMPLE_SECONDS" ros2 topic hz /aero_halo_360/sector_distances 2>&1 || true)"
  echo "$SECTOR_HZ"
  echo "$SECTOR_HZ" | grep -q 'average rate:' &&
    pass "扇区话题频率可测" || fail "扇区话题无有效频率"

  DUPLICATES="$(ros2 node list 2>/dev/null | sort | uniq -d)"
  if [[ -n "$DUPLICATES" ]]; then
    echo "$DUPLICATES" >&2
    fail "发现重复节点名"
  else
    pass "未发现重复节点名"
  fi
fi

if [[ "$FAILURES" -ne 0 ]]; then
  echo "预检完成：$FAILURES 项失败。" >&2
  exit 1
fi

echo "预检完成：所有已请求检查均通过。"
