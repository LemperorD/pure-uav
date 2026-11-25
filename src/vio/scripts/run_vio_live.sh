#!/usr/bin/env bash
# 实时 VIO 一键启动。参数细节记不住没关系，用这个脚本。
#
#   ./scripts/run_vio_live.sh                  手持演示：动态初始化 + 开窗口 + 不发飞控
#   ./scripts/run_vio_live.sh --static         同上，但静态初始化（先静止 ≥2 秒，再动一下触发）
#   ./scripts/run_vio_live.sh --duration 60    60 秒后自动退出（默认一直跑到按 q）
#   ./scripts/run_vio_live.sh --fly            正式运行：向飞控发送位姿（默认自动启停 camera_server）
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
REPO_ROOT="$(cd "$ROOT/../.." && pwd)"
cd "$ROOT"

MODE=demo
DURATION=0
STATIC=0
OUTDIR=""
PIN_CPUS=1
EXTRA=()

usage() { sed -n '2,14p' "$ROOT/scripts/run_vio_live.sh" | sed 's/^# \{0,1\}//'; exit 0; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --static)   STATIC=1; shift ;;
    --fly)      MODE=fly; shift ;;
    --demo)     MODE=demo; shift ;;
    --no-pin)   PIN_CPUS=0; shift ;;
    --duration) DURATION="${2:?--duration 需要一个秒数}"; shift 2 ;;
    --out)      OUTDIR="${2:?--out 需要一个目录}"; shift 2 ;;
    -h|--help)  usage ;;
    --)         shift; EXTRA=("$@"); break ;;
    *)          echo "未知参数: $1（要透传给 vio_live 的参数请放在 -- 之后）" >&2; exit 2 ;;
  esac
done

# 在整仓构建和独立构建的产物中选最新的二进制。
BIN=""
BIN_MTIME=0
for cand in "$REPO_ROOT/bin/vio_live" "$REPO_ROOT/build/vio/bin/vio_live" "$ROOT/build/bin/vio_live"; do
  [[ -x "$cand" ]] || continue
  mt=$(stat -c %Y "$cand" 2>/dev/null || echo 0)
  if (( mt > BIN_MTIME )); then BIN_MTIME=$mt; BIN="$cand"; fi
done
if [[ -z "$BIN" ]]; then
  cat >&2 <<EOF
找不到 vio_live —— 先在仓库根目录编译：
  cd "$REPO_ROOT"
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build -j\$(nproc) --target vio_live
EOF
  exit 1
fi
echo "[run] 二进制: $BIN  ($(date -d @"$BIN_MTIME" '+%m-%d %H:%M'))"

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
  # 曝光和测光统一由 camera_server 配置；消费者只控制位姿发送。
  # 失效保护默认开，LOST 期间不发位姿。
  echo "[run] 模式=飞行（会真的往飞控发 ODOMETRY）"
else
  # 手持演示：只读相机服务并显示，不发送位姿。
  ARGS+=( --view --no-send )
  echo "[run] 模式=手持演示（不发飞控）"
fi

if [[ "$STATIC" == "1" ]]; then
  ARGS+=( --static-init )
  echo "[run] 初始化=静态：先静止 ≥2 秒，然后动一下触发"
else
  echo "[run] 初始化=动态"
fi

[[ "$DURATION" != "0" ]] && ARGS+=( --duration "$DURATION" )

echo "[run] 默认自动启停 camera_server（复用外部服务请加 -- --external-camera）；曝光、增益、测光和深度流由 config/camera.yaml 管理"
echo "[run] 输出目录: $OUTDIR"

# -----------------------------------------------------------------------------
# 多核分簇绑核与定频（尤其针对 Orange Pi 等 8 核 ARM 大小簇/双簇板卡）：
#   1) 非阻塞尝试把 scaling_governor 切到 performance（无免密权限则仅提示，绝不阻塞启动）；
#   2) 自动按 cpuinfo_max_freq 识别高频簇给 PURE_VIO_CPUS、其余簇给 PURE_CAM_CPUS；
#      对称多核 ARM（>=6 核）则将前 1/3~1/2 核给 camera_server，后半簇给 vio_live。
#      外部显式传入 PURE_VIO_CPUS / PURE_CAM_CPUS 或加 --no-pin 时尊重外部设置。
# -----------------------------------------------------------------------------
if [[ "$PIN_CPUS" == "1" && "$(uname -s)" == "Linux" ]]; then
  gov_files=(/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor)
  if [[ -e "${gov_files[0]}" ]]; then
    need_perf=0
    for gf in "${gov_files[@]}"; do
      [[ "$(cat "$gf" 2>/dev/null || true)" != "performance" ]] && need_perf=1 && break
    done
    if [[ "$need_perf" == "1" ]]; then
      if [[ -w "${gov_files[0]}" ]]; then
        for gf in "${gov_files[@]}"; do echo performance > "$gf" 2>/dev/null || true; done
        echo "[run] CPU 调频: 已锁定 performance 定频模式"
      elif command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; then
        for gf in "${gov_files[@]}"; do echo performance | sudo -n tee "$gf" >/dev/null 2>&1 || true; done
        echo "[run] CPU 调频: 已通过免密 sudo 锁定 performance 定频模式"
      else
        echo "[run] 提示: 当前 CPU 未锁定 performance（板端可执行: echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor）"
      fi
    fi
  fi

  if [[ -z "${PURE_VIO_CPUS:-}" && -z "${PURE_CAM_CPUS:-}" ]]; then
    readarray -t cpu_dirs < <(printf '%s\n' /sys/devices/system/cpu/cpu[0-9]* | sort -V)
    online_cpus=()
    max_freq=0
    min_freq=0
    for cd in "${cpu_dirs[@]}"; do
      [[ -d "$cd" ]] || continue
      cid="${cd##*cpu}"
      if [[ -f "$cd/online" && "$(cat "$cd/online" 2>/dev/null || echo 1)" != "1" ]]; then
        continue
      fi
      online_cpus+=("$cid")
      if [[ -f "$cd/cpufreq/cpuinfo_max_freq" ]]; then
        f="$(cat "$cd/cpufreq/cpuinfo_max_freq" 2>/dev/null || echo 0)"
        if [[ "$f" =~ ^[0-9]+$ ]] && (( f > 0 )); then
          (( f > max_freq )) && max_freq=$f
          (( min_freq == 0 || f < min_freq )) && min_freq=$f
        fi
      fi
    done

    n_online=${#online_cpus[@]}
    if (( n_online >= 4 )); then
      vio_list=()
      cam_list=()
      if (( max_freq > 0 && min_freq > 0 && min_freq * 100 < max_freq * 90 )); then
        # 异构大小核/高低频双簇：>=90% 最高频的核心划给 VIO，低频簇划给 camera_server
        thresh=$(( max_freq * 90 / 100 ))
        for cid in "${online_cpus[@]}"; do
          f="$(cat "/sys/devices/system/cpu/cpu${cid}/cpufreq/cpuinfo_max_freq" 2>/dev/null || echo 0)"
          if [[ "$f" =~ ^[0-9]+$ ]] && (( f >= thresh )); then
            vio_list+=("$cid")
          else
            cam_list+=("$cid")
          fi
        done
      elif [[ "$(uname -m)" =~ ^(aarch64|arm64|armv.*)$ ]] && (( n_online >= 6 )); then
        # 对称多核 ARM（例如双 4 核簇同频）：前半核簇给相机解码与系统中断，后半核簇专供 VIO
        split=$(( n_online == 8 ? 4 : 2 ))
        cam_list=("${online_cpus[@]:0:split}")
        vio_list=("${online_cpus[@]:split}")
      fi

      if (( ${#vio_list[@]} >= 2 && ${#cam_list[@]} >= 1 )); then
        export PURE_VIO_CPUS
        export PURE_CAM_CPUS
        PURE_VIO_CPUS="$(IFS=,; echo "${vio_list[*]}")"
        PURE_CAM_CPUS="$(IFS=,; echo "${cam_list[*]}")"
        echo "[run] 自动分簇绑核: VIO=${PURE_VIO_CPUS}  camera_server=${PURE_CAM_CPUS}（关闭请加 --no-pin）"
      fi
    fi
  else
    echo "[run] 使用外部指定绑核: PURE_VIO_CPUS=${PURE_VIO_CPUS:-未设}  PURE_CAM_CPUS=${PURE_CAM_CPUS:-未设}"
  fi
fi

echo "[run] 命令: $BIN ${ARGS[*]} ${EXTRA[*]}"
echo

#=============================================================================
# 【2026-10-23】运行条件留痕。教训：ver1 与 ver2 只差"走得多快"，但 ver2 当时
# 没 tee 控制台，事后要花时间从 SDK 日志里挖当时的曝光/增益。
# 现在把参数、二进制时间戳、配置指纹一起写进 run_meta.txt，
# 并**强制** tee 到 $OUTDIR/console.log —— 不再依赖手打重定向。
#=============================================================================
CONFIG_YAML="$REPO_ROOT/config/camera.yaml"
REC_DIR=""
prev=""
for tok in "${EXTRA[@]}"; do
  [[ "$prev" == "--record" ]] && REC_DIR="$tok"
  [[ "$prev" == "--camera-config" ]] && CONFIG_YAML="$tok"
  prev="$tok"
done
if [[ ! -f "$CONFIG_YAML" && "$CONFIG_YAML" != /* ]]; then
  CONFIG_YAML="$REPO_ROOT/$CONFIG_YAML"
fi
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
# 记录退出状态；相机实际曝光/增益请查 camera_server 日志。
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
  echo "[run] 没生成位姿文件（没初始化成功？）检查上面的 [CAM]/[VIO] 日志和 camera_server 状态" >&2
fi
exit "$RC"
