#!/usr/bin/env bash
# 离线原地绕圈测试：采集目录 → VIO 数据集 → 跑 VIO → 漂移报告。
#
#   ./scripts/test_inplace_offline.sh session18
#   ./scripts/test_inplace_offline.sh ~/calib/camera_imu_calibration/data/session18
#
# 需要先录好 data/sessionXX（camera_imu_calibration 的采集目录）。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
CALIB="$HOME/calib/camera_imu_calibration"

ARG="${1:-}"
if [ -z "$ARG" ]; then
  echo "用法: $0 <session名 或 采集目录>" >&2
  exit 2
fi
if [ -d "$ARG" ]; then
  SESSION_DIR="$(cd "$ARG" && pwd)"
  NAME="$(basename "$SESSION_DIR")"
else
  NAME="$ARG"
  SESSION_DIR="$CALIB/data/$NAME"
fi
if [ ! -f "$SESSION_DIR/images.csv" ] || [ ! -f "$SESSION_DIR/imu.csv" ]; then
  echo "❌ $SESSION_DIR 里没有 images.csv / imu.csv" >&2
  exit 2
fi

DS="$HOME/calib/datasets/${NAME}_vio"
TUM="out/${NAME}_traj.tum"

echo "== 1/3 转数据集 =="
python3 "$ROOT/tools/session_to_dataset.py" --session "$SESSION_DIR" --out "$DS"

echo
echo "== 2/3 跑 VIO =="
"$ROOT/build/bin/vio_offline" \
  --dataset "$DS" \
  --config-dir "$ROOT/config/gemini_1920x1080" \
  --csv-out "out/${NAME}_pose.csv" \
  --tum-out "$TUM" \
  --raw-tum-out "out/${NAME}_raw.tum" 2>&1 | tail -12

echo
echo "== 3/3 漂移报告 =="
python3 "$ROOT/tools/drift_report.py" "$TUM" --plot "out/${NAME}_drift.png"

echo
echo "在 RViz 里回放："
echo "  $ROOT/scripts/vio_rviz.sh $TUM replay --speed 3 --loop"
