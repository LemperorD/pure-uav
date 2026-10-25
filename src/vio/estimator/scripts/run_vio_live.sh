#!/usr/bin/env bash
# 实时 VIO 一键启动。参数细节记不住没关系，用这个脚本。
#
#   ./scripts/run_vio_live.sh                  手持演示：动态初始化 + 开窗口 + 不发飞控
#   ./scripts/run_vio_live.sh --static         同上，但静态初始化（先静止 ≥2 秒，再动一下触发）
#   ./scripts/run_vio_live.sh --duration 60    60 秒后自动退出（默认一直跑到按 q）
#   ./scripts/run_vio_live.sh --fly            正式飞行：按 100°/s 自动测光 + 真的发飞控
#   ./scripts/run_vio_live.sh --out DIR        指定输出目录（默认 out/live_<时间戳>）
#   ./scripts/run_vio_live.sh -- --no-send     "--" 之后的参数原样传给 vio_live
#
# 每次都会记录：pose.csv（位姿+零偏）、traj.tum（全部估计位姿）、
#               traj_safe.tum（只含健康门限通过的 = 实际发出去的，给 RViz 看这个）、imu.csv。
# 跑完用：python3 scripts/analyze_live.py <输出目录>
#
# 窗口里按 q 或 Esc 退出。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# 二进制位置随工程布局不同：
#   开发副本 ~/calib/vio_online   -> build/bin/vio_live
#   pure-uav-vio 仓库 src/vio/estimator -> <repo>/bin/vio_live
# 同名二进制可能在多处（开发副本的 build/bin、仓库根的 bin、老构建残留），
# 一律取**修改时间最新**的那个，避免跑到旧构建上。
BIN=""
BIN_MTIME=0
for cand in "${ROOT}/build/bin/vio_live" "${ROOT}/../../../bin/vio_live" "${ROOT}/../../../../bin/vio_live"; do
  [[ -x "$cand" ]] || continue
  mt=$(stat -c %Y "$cand" 2>/dev/null || echo 0)
  if (( mt > BIN_MTIME )); then BIN_MTIME=$mt; BIN="$cand"; fi
done
[[ -n "$BIN" ]] || { echo "[错误] 找不到 vio_live，先编译" >&2; exit 1; }
BIN="$(cd "$(dirname "$BIN")" && pwd)/vio_live"
echo "[run] 二进制: $BIN  ($(date -d @"$BIN_MTIME" '+%m-%d %H:%M'))"
MODE=demo
DURATION=0
STATIC=0
OUTDIR=""
EXTRA=()

usage() { sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --static)   STATIC=1; shift ;;
    --fly)      MODE=fly; shift ;;
    --demo)     MODE=demo; shift ;;
    --duration) DURATION="${2:?--duration 需要一个秒数}"; shift 2 ;;
    --out)      OUTDIR="${2:?--out 需要一个目录}"; shift 2 ;;
    -h|--help)  usage ;;
    --)         shift; EXTRA=("$@"); break ;;
    *)          echo "未知参数: $1（要透传给 vio_live 的参数请放在 -- 之后）" >&2; exit 2 ;;
  esac
done

if [[ ! -x "$BIN" ]]; then
  cat >&2 <<EOF
找不到 $BIN —— 先编译：
  cd "$ROOT"
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build -j\$(nproc) --target vio_live
EOF
  exit 1
fi

[[ -z "$OUTDIR" ]] && OUTDIR="out/live_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTDIR"

ARGS=(
  --imu-log      "$OUTDIR/imu.csv"
  --csv-out      "$OUTDIR/pose.csv"
  --tum-out      "$OUTDIR/traj.tum"
  --safe-tum-out "$OUTDIR/traj_safe.tum"
  --view-shot-prefix "$OUTDIR/shot"
)

if [[ "$MODE" == "fly" ]]; then
  # 正式飞行：按 100°/s 反推曝光上限（模糊预算 3 px），不加 --allow-blur —— 光不够就
  # 直接拒绝启动，这是它该做的事。失效保护默认开，LOST 期间不发位姿。
  ARGS+=( --tune-expected-rate 100 )
  echo "[run] 模式=飞行（会真的往飞控发 ODOMETRY）"
else
  # 手持演示：按 15°/s 估曝光；这个房间的光本来就不满足我们自己的"能飞"标准，
  # 用 --allow-blur 显式放行那道检查（只是想看实时行为，不是结论）。
  ARGS+=( --tune-expected-rate 15 --allow-blur --view --no-send )
  echo "[run] 模式=手持演示（不发飞控）"
fi

if [[ "$STATIC" == "1" ]]; then
  ARGS+=( --static-init )
  echo "[run] 初始化=静态：先静止 ≥2 秒，然后动一下触发"
else
  echo "[run] 初始化=动态"
fi

[[ "$DURATION" != "0" ]] && ARGS+=( --duration "$DURATION" )

echo "[run] 输出目录: $OUTDIR"
echo "[run] 命令: $BIN ${ARGS[*]} ${EXTRA[*]}"
echo

#=============================================================================
# 【2026-10-23】运行条件留痕。教训：ver1 与 ver2 只差"走得多快"，但 ver2 当时
# 没 tee 控制台，事后要花时间从 SDK 日志里挖当时的曝光/增益。
# 现在把参数、二进制时间戳、配置指纹一起写进 run_meta.txt，
# 并**强制** tee 到 $OUTDIR/console.log —— 不再依赖手打重定向。
#=============================================================================
CFG_DIR="config/gemini_1920x1080"
REC_DIR=""
prev=""
for tok in "${EXTRA[@]}"; do
  # 【2026-10-23】--config-dir 也要跟：fast1 那次记录成 1920x1080，差点让我
  # 误判它为"内参没生效导致 3 倍尺度"。meta 必须记真实用的那份配置。
  [[ "$prev" == "--record" ]] && REC_DIR="$tok"
  [[ "$prev" == "--config-dir" ]] && CFG_DIR="$tok"
  prev="$tok"
done
CONFIG_YAML="$ROOT/$CFG_DIR/estimator_config.yaml"
[[ -f "$CONFIG_YAML" ]] || CONFIG_YAML="$ROOT/config/gemini_1920x1080/estimator_config.yaml"
META="$OUTDIR/run_meta.txt"
{
  echo "started: $(date -Is)"
  echo "argv: $BIN ${ARGS[*]} ${EXTRA[*]}"
  echo "mode: $MODE"
  echo "binary: $BIN"
  echo "binary_mtime: $(date -d @"$BIN_MTIME" -Is)"
  echo "binary_md5: $(md5sum "$BIN" | awk '{print $1}')"
  echo "config: $CONFIG_YAML"
  if [[ -f "$CONFIG_YAML" ]]; then
    echo "config_md5: $(md5sum "$CONFIG_YAML" | awk '{print $1}')"
    echo "config_mtime: $(date -r "$CONFIG_YAML" -Is)"
  fi
  echo "record_dir: ${REC_DIR:--}"
  echo "host: $(hostname)  user: ${USER}"
} > "$META"
if [[ -n "$REC_DIR" ]]; then cp "$META" "$REC_DIR/run_meta.txt" 2>/dev/null || true; fi

set +e
"$BIN" "${ARGS[@]}" "${EXTRA[@]}" 2>&1 | tee "$OUTDIR/console.log" \
  | grep -vE '^\[[0-9]{2}/[0-9]{2} [0-9:.]+\]'
RC=${PIPESTATUS[0]}
set -e
# 跑完把实际锁定的曝光/增益/运动上限补进 run_meta.txt（这些只有跑起来才知道）
{
  echo "ended: $(date -Is)"
  echo "rc: $RC"
  grep -aoE '锁定：曝光 [0-9.]+ ms 增益 [0-9]+ → 灰度 [0-9.]+' "$OUTDIR/console.log" | tail -1 || true
  grep -aoE '运动上限 [0-9.]+°/s' "$OUTDIR/console.log" | tail -1 || true
  grep -aoE '平移上限 ≈ [0-9.]+×Z m/s' "$OUTDIR/console.log" | tail -1 || true
  echo "deperr_collapse_hits: $(grep -ac '深度一致性崩塌' "$OUTDIR/console.log" || true)"
} >> "$META"
if [[ -n "$REC_DIR" ]]; then cp "$META" "$REC_DIR/run_meta.txt" 2>/dev/null || true; fi

echo
if [[ -s "$OUTDIR/pose.csv" ]]; then
  echo "[run] 完事。分析这次的数据："
  echo "      python3 scripts/analyze_live.py $OUTDIR"
else
  echo "[run] 没生成位姿文件（没初始化成功？）检查上面的 [Orbbec]/[VIO] 日志" >&2
fi
exit "$RC"
