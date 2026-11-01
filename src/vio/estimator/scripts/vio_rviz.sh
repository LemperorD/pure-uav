#!/usr/bin/env bash
# 启动 rviz2 + TUM→ROS2 桥接。
#
#   ./scripts/vio_rviz.sh out/session18_traj.tum            # live：边跑边看
#   ./scripts/vio_rviz.sh out/session18_traj.tum replay     # 回放整个文件
#   ./scripts/vio_rviz.sh out/session18_traj.tum replay --speed 4 --loop
#
# rviz 必须在宿主机桌面（DISPLAY 可用）里跑；用系统 ROS2 Humble 的 Python 3.10。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"

if [ -z "${DISPLAY:-}" ]; then
  echo "❌ DISPLAY 为空：rviz2 需要桌面环境（在笔记本本机终端里跑）" >&2
  exit 2
fi
TUM="${1:-}"
if [ -z "$TUM" ]; then
  echo "用法: $0 <tum文件> [live|replay] [其余参数...]" >&2
  exit 2
fi
shift
MODE="live"
if [ "${1:-}" = "live" ] || [ "${1:-}" = "replay" ]; then
  MODE="$1"; shift
fi

# 避开 conda 的库/Python 路径污染（ROS2 Humble 用的是系统 Python 3.10）
unset PYTHONPATH LD_LIBRARY_PATH 2>/dev/null || true
source /opt/ros/humble/setup.bash
export ROS_HOME="${ROS_HOME:-$ROOT/out/ros_home}"
mkdir -p "$ROS_HOME"

rviz2 -d "$ROOT/rviz/vio.rviz" &
RVIZ_PID=$!
cleanup() { kill "$RVIZ_PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

sleep 1
/usr/bin/python3 "$ROOT/tools/tum_to_rviz.py" --tum "$TUM" --mode "$MODE" "$@"
