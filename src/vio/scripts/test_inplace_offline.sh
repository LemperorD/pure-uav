#!/usr/bin/env bash
# 离线原地绕圈测试：采集目录 → VIO 数据集 → 跑 VIO → 漂移报告。
#
#   ./scripts/test_inplace_offline.sh session18
#   ./scripts/test_inplace_offline.sh /path/to/session18
#
# 需要先录好 data/vio/sessionXX 或 calibration_for_vio/extrinsic/data/sessionXX。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
REPO_ROOT="$(cd "$ROOT/../.." && pwd)"
CALIB_REPO="${VIO_CALIBRATION_REPO:-$(dirname "$REPO_ROOT")/calibration_for_vio}"

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
  if [ -d "$ROOT/data/vio/$NAME" ]; then
    SESSION_DIR="$ROOT/data/vio/$NAME"
  else
    SESSION_DIR="$CALIB_REPO/extrinsic/data/$NAME"
  fi
fi
if [ ! -f "$SESSION_DIR/images.csv" ] || [ ! -f "$SESSION_DIR/imu.csv" ]; then
  echo "❌ $SESSION_DIR 里没有 images.csv / imu.csv" >&2
  exit 2
fi

OUT_DIR="$ROOT/out"
mkdir -p "$OUT_DIR"
DS="$OUT_DIR/datasets/${NAME}_vio"
TUM="$OUT_DIR/${NAME}_traj.tum"
BIN=""
for cand in "$ROOT/build/bin/vio_offline" "$REPO_ROOT/bin/vio_offline" "$REPO_ROOT/build/vio/bin/vio_offline"; do
  if [ -x "$cand" ]; then
    BIN="$cand"
    break
  fi
done
if [ -z "$BIN" ]; then
  echo "❌ 找不到 vio_offline，请先编译" >&2
  exit 2
fi

echo "== 1/3 转数据集 =="
python3 "$ROOT/tools/session_to_dataset.py" --session "$SESSION_DIR" --out "$DS"

echo
echo "== 2/3 跑 VIO =="
"$BIN" \
  --dataset "$DS" \
  --config-dir "$ROOT/config/gemini_1920x1080" \
  --csv-out "$OUT_DIR/${NAME}_pose.csv" \
  --tum-out "$TUM" \
  --raw-tum-out "$OUT_DIR/${NAME}_raw.tum" 2>&1 | tail -12

echo
echo "== 3/3 漂移报告 =="
python3 "$ROOT/tools/drift_report.py" "$TUM" --plot "$OUT_DIR/${NAME}_drift.png"

echo
echo "在 RViz 里回放："
echo "  $ROOT/scripts/vio_rviz.sh $TUM replay --speed 3 --loop"
