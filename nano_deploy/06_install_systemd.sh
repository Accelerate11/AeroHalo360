#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEFAULT_WORKSPACE="$(cd "$REPO_ROOT/../.." && pwd)"

AERO_USER="$USER"
AERO_WORKSPACE="$DEFAULT_WORKSPACE"
LIVOX_WORKSPACE="$HOME/ws_livox"
MAVLINK_CONNECTION=""
MAVLINK_BAUD="921600"
LIVOX_LAUNCH_FILE="msg_MID360_launch.py"
ENABLE_SERVICES=false
ENABLE_STATIC_TF=false

usage() {
  cat <<'EOF'
用法：06_install_systemd.sh --mavlink-device PATH [选项]

选项：
  --user NAME                 systemd 运行用户，默认当前用户
  --workspace PATH            AeroHalo360 工作空间
  --livox-workspace PATH      Livox ROS 2 工作空间
  --livox-launch FILE         Livox launch 文件名
  --mavlink-device PATH       稳定串口路径或显式 UDP 连接，必填
  --mavlink-baud RATE         波特率，默认 921600
  --enable                    安装后启用并立即启动 Livox/cloud/sender/health
  --enable-static-tf          同时启用静态 TF 服务；仅限已标定参数
  --help                      显示帮助

脚本生成 /etc/aero-halo-360/env 和 systemd unit，不写飞控参数或网络配置。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --user) AERO_USER="$2"; shift 2 ;;
    --workspace) AERO_WORKSPACE="$2"; shift 2 ;;
    --livox-workspace) LIVOX_WORKSPACE="$2"; shift 2 ;;
    --livox-launch) LIVOX_LAUNCH_FILE="$2"; shift 2 ;;
    --mavlink-device) MAVLINK_CONNECTION="$2"; shift 2 ;;
    --mavlink-baud) MAVLINK_BAUD="$2"; shift 2 ;;
    --enable) ENABLE_SERVICES=true; shift ;;
    --enable-static-tf) ENABLE_STATIC_TF=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "错误：未知参数 $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! "$AERO_USER" =~ ^[a-z_][a-z0-9_-]*$ ]]; then
  echo "错误：用户名称格式无效：$AERO_USER" >&2
  exit 2
fi
if ! id "$AERO_USER" >/dev/null 2>&1; then
  echo "错误：用户不存在：$AERO_USER" >&2
  exit 2
fi
if [[ -z "$MAVLINK_CONNECTION" ]]; then
  echo "错误：必须显式提供 --mavlink-device。" >&2
  exit 2
fi
if [[ ! "$MAVLINK_BAUD" =~ ^[0-9]+$ || "$MAVLINK_BAUD" -le 0 ]]; then
  echo "错误：--mavlink-baud 必须是正整数。" >&2
  exit 2
fi

AERO_WORKSPACE="$(realpath "$AERO_WORKSPACE")"
LIVOX_WORKSPACE="$(realpath "$LIVOX_WORKSPACE")"
AERO_SOURCE="$AERO_WORKSPACE/src/aero-halo-360"
BASE_CONFIG="$AERO_WORKSPACE/install/aero_halo_360/share/aero_halo_360/config/default.yaml"
PROFILE_CONFIG="$AERO_WORKSPACE/install/aero_halo_360/share/aero_halo_360/config/flight_low_speed.yaml"

for required in   "$AERO_WORKSPACE/install/setup.bash"   "$AERO_SOURCE/nano_deploy/05_health_check.sh"   "$BASE_CONFIG"   "$PROFILE_CONFIG"   "$LIVOX_WORKSPACE/install/setup.bash"; do
  if [[ ! -e "$required" ]]; then
    echo "错误：缺少部署文件：$required" >&2
    exit 1
  fi
done

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT
cat >"$TMP_DIR/env" <<EOF
ROS_SETUP=/opt/ros/humble/setup.bash
AERO_WORKSPACE=$AERO_WORKSPACE
AERO_SOURCE=$AERO_SOURCE
LIVOX_WORKSPACE=$LIVOX_WORKSPACE
LIVOX_LAUNCH_FILE=$LIVOX_LAUNCH_FILE
BASE_CONFIG=$BASE_CONFIG
PROFILE_CONFIG=$PROFILE_CONFIG
MAVLINK_CONNECTION=$MAVLINK_CONNECTION
MAVLINK_BAUD=$MAVLINK_BAUD
LIDAR_PARENT_FRAME=base_link
LIDAR_CHILD_FRAME=livox_frame
LIDAR_X=0.0
LIDAR_Y=0.0
LIDAR_Z=0.0
LIDAR_ROLL=0.0
LIDAR_PITCH=0.0
LIDAR_YAW=0.0
EOF

for template in "$REPO_ROOT"/systemd/*.service.in; do
  output="$TMP_DIR/$(basename "${template%.in}")"
  sed "s/@AERO_USER@/$AERO_USER/g" "$template" >"$output"
done
cp "$REPO_ROOT/systemd/aero-halo-360-health.timer" "$TMP_DIR/"

sudo install -d -m 0755 /etc/aero-halo-360
sudo install -m 0640 "$TMP_DIR/env" /etc/aero-halo-360/env
sudo install -m 0644 "$TMP_DIR"/*.service /etc/systemd/system/
sudo install -m 0644 "$TMP_DIR/aero-halo-360-health.timer" /etc/systemd/system/
sudo systemctl daemon-reload

if "$ENABLE_SERVICES"; then
  sudo systemctl enable --now     livox-mid360.service     aero-halo-360-cloud.service     aero-halo-360-sender.service     aero-halo-360-health.timer
fi
if "$ENABLE_STATIC_TF"; then
  echo "警告：静态 TF 默认值仍为零；请先编辑 /etc/aero-halo-360/env 写入实测标定值。"
  sudo systemctl enable --now aero-halo-360-tf.service
fi

echo "systemd 文件安装完成。"
echo "环境文件：/etc/aero-halo-360/env"
echo "日志命令：sudo journalctl -u aero-halo-360-cloud -u aero-halo-360-sender -f"
