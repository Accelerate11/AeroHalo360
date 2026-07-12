#!/usr/bin/env bash
set -eo pipefail

VERSION="v0.2.1-alpha"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OUTPUT_DIR="$REPO_ROOT/release"
RELEASE_DATE="$(date +%Y%m%d)"

usage() {
  cat <<'EOF'
用法：07_package_release.sh [--output-dir DIR] [--date YYYYMMDD] [--help]

重新生成 PACKAGE_MANIFEST.txt，并创建 v0.2.1-alpha 源码 tar.gz 与 SHA-256 sidecar。
归档不包含 .git、build、install、log、rosbag、缓存、密钥或本机发布目录。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --date) RELEASE_DATE="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "错误：未知参数 $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ ! "$RELEASE_DATE" =~ ^[0-9]{8}$ ]]; then
  echo "错误：--date 必须为 YYYYMMDD。" >&2
  exit 2
fi

mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(realpath "$OUTPUT_DIR")"
ARCHIVE_NAME="AeroHalo360_nano_${VERSION}_${RELEASE_DATE}.tar.gz"
ARCHIVE_PATH="$OUTPUT_DIR/$ARCHIVE_NAME"
SHA_PATH="$OUTPUT_DIR/${ARCHIVE_NAME%.tar.gz}.sha256"

mapfile -t FILES < <(
  cd "$REPO_ROOT"
  find . -type f     ! -path './.git/*'     ! -path './release/*'     ! -path '*/build/*'     ! -path '*/install/*'     ! -path '*/log/*'     ! -path '*/__pycache__/*'     ! -name '*.pyc'     ! -name '*.bag'     ! -name '*.db3'     ! -name '*.mcap'     ! -name '*.key'     ! -name '*.pem'     -printf '%P\n' | LC_ALL=C sort
)

{
  echo "AeroHalo360 Nano 源码发布清单"
  echo
  echo "版本：$VERSION"
  echo "打包日期：$RELEASE_DATE"
  echo "目标环境：Jetson Orin Nano / aarch64 / Ubuntu 22.04 / ROS 2 Humble"
  echo "文件总数：${#FILES[@]}"
  echo
  echo "以下路径全部包含在归档中；不包含 build/install/log、rosbag、缓存、密钥和个人配置。"
  printf '%s\n' "${FILES[@]}"
} >"$REPO_ROOT/PACKAGE_MANIFEST.txt"

PARENT_DIR="$(dirname "$REPO_ROOT")"
ROOT_NAME="$(basename "$REPO_ROOT")"
tar   --sort=name   --mtime='UTC 2026-01-01'   --owner=0   --group=0   --numeric-owner   --exclude-vcs   --exclude="$ROOT_NAME/release"   --exclude='*/__pycache__'   --exclude='*.pyc'   --exclude='*.bag'   --exclude='*.db3'   --exclude='*.mcap'   --exclude='*.key'   --exclude='*.pem'   -czf "$ARCHIVE_PATH"   -C "$PARENT_DIR" "$ROOT_NAME"

(
  cd "$OUTPUT_DIR"
  sha256sum "$ARCHIVE_NAME" >"$(basename "$SHA_PATH")"
)
tar -tzf "$ARCHIVE_PATH" >/dev/null
(
  cd "$OUTPUT_DIR"
  sha256sum --check "$(basename "$SHA_PATH")"
)
echo "发布包：$ARCHIVE_PATH"
echo "校验文件：$SHA_PATH"
