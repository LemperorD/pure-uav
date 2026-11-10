#!/bin/bash
# =============================================================================
#  机载统一入口：相机只初始化一次，VIO 与检测作为独立进程从共享内存取帧。
#
#  启停顺序：清理残留通道 → camera_server（独占 Orbbec）→ vio_live / detect_rt
#  退出时逆序停止。检测进程崩溃只重启检测，不停止定位。
#
#  用法：
#    ./scripts/start_uav.sh --no-send
#    ./scripts/start_uav.sh --detection --stream
#    ./scripts/start_uav.sh --detection --serial /dev/ttyACM0 --conf 0.3
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

CAMERA_CONFIG="config/camera.yaml"
VIO_CHANNEL="/pure_uav_camera_vio"
COLOR_CHANNEL="/pure_uav_camera_color"
ENABLE_DETECTION=0
ENABLE_STREAM=0
NO_SEND=0
STREAM_DIR="/dev/shm/pure-uav-detect"
STREAM_FPS=20
VIO_ARGS=()
DET_ARGS=()
SERVER_PID=0
VIO_PID=0
DET_PID=0
DET_RESTARTS=0

VIO_VALUE_OPTS="--serial --baud --config-dir --csv-out --tum-out --safe-tum-out --imu-log --imu-hz --cam-ts --record --duration --health-min-features --width --height --fps --shm-timeout-ms"
DET_VALUE_OPTS="--model --conf --iou --frames --dump-dir --shm-timeout-ms"

usage() {
    cat <<EOF
用法: $0 [选项]

  --detection            同时启动 detect_rt（默认只跑 VIO）
  --stream               检测侧写带框 JPG，供 mjpg-streamer 推流
  --no-send              VIO 只计算不向飞控发送 ODOMETRY
  --config FILE          相机服务配置（默认 config/camera.yaml）
  --vio-channel NAME     VIO 共享内存通道名
  --color-channel NAME   彩色共享内存通道名
  --stream-dir DIR       带框 JPG 目录（默认 /dev/shm/pure-uav-detect）
  --stream-fps N         推流帧率（默认 20）
  -h, --help             显示帮助

VIO 参数（透传 vio_live）：--serial --baud --config-dir --csv-out --tum-out
  --safe-tum-out --imu-log --imu-hz --cam-ts --record --duration
  --health-min-features --width --height --fps --shm-timeout-ms，
  以及 --view --quiet --no-online-timeoffset --no-depth --no-tune。
检测参数（透传 detect_rt）：--model --conf --iou --frames --dump-dir
  --shm-timeout-ms，以及 --json。
EOF
}

shm_file() {
    echo "/dev/shm/${1#/}"
}

route_value_opt() {
    local opt="$1"
    local val="$2"
    local routed=0
    if [[ " ${VIO_VALUE_OPTS} " == *" ${opt} "* ]]; then
        VIO_ARGS+=("${opt}" "${val}")
        routed=1
    fi
    if [[ " ${DET_VALUE_OPTS} " == *" ${opt} "* ]]; then
        DET_ARGS+=("${opt}" "${val}")
        routed=1
    fi
    if [[ "${routed}" -eq 0 ]]; then
        echo "[error] 未知参数: ${opt}" >&2
        exit 2
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --detection) ENABLE_DETECTION=1; shift ;;
        --stream) ENABLE_STREAM=1; shift ;;
        --no-send) NO_SEND=1; shift ;;
        --config) CAMERA_CONFIG="$2"; shift 2 ;;
        --vio-channel) VIO_CHANNEL="$2"; shift 2 ;;
        --color-channel) COLOR_CHANNEL="$2"; shift 2 ;;
        --stream-dir) STREAM_DIR="$2"; shift 2 ;;
        --stream-fps) STREAM_FPS="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        --view|--quiet|--no-online-timeoffset|--no-depth|--no-tune)
            VIO_ARGS+=("$1"); shift ;;
        --json) DET_ARGS+=("$1"); shift ;;
        --*)
            if [[ $# -lt 2 ]]; then
                echo "[error] ${1} 缺少参数值" >&2
                exit 2
            fi
            route_value_opt "$1" "$2"
            shift 2
            ;;
        *)
            echo "[error] 未知参数: $1" >&2
            usage
            exit 2
            ;;
    esac
done

cleanup() {
    trap - EXIT INT TERM
    local pid
    for pid in "${DET_PID}" "${VIO_PID}" "${SERVER_PID}"; do
        if [[ "${pid}" -gt 0 ]] && kill -0 "${pid}" 2>/dev/null; then
            kill "${pid}" 2>/dev/null || true
        fi
    done
    for pid in "${DET_PID}" "${VIO_PID}" "${SERVER_PID}"; do
        if [[ "${pid}" -gt 0 ]]; then
            wait "${pid}" 2>/dev/null || true
        fi
    done
    rm -f "$(shm_file "${VIO_CHANNEL}")" "$(shm_file "${COLOR_CHANNEL}")"
    echo "[info] 已停止相机服务与消费者，共享内存通道已清理"
}
trap cleanup EXIT INT TERM

for bin in bin/camera_server bin/vio_live; do
    if [[ ! -x "${bin}" ]]; then
        echo "[error] 找不到 ${bin}，请先执行 ./scripts/autoBuild.sh" >&2
        exit 1
    fi
done
if [[ "${ENABLE_DETECTION}" -eq 1 && ! -x bin/detect_rt ]]; then
    echo "[error] 找不到 bin/detect_rt，请先执行 ./scripts/autoBuild.sh" >&2
    exit 1
fi

# 上次异常退出可能留下共享内存对象；相机服务也会自行 unlink，这里兜底。
rm -f "$(shm_file "${VIO_CHANNEL}")" "$(shm_file "${COLOR_CHANNEL}")"

echo "[info] 启动 camera_server（唯一相机采集进程）..."
env -u LD_LIBRARY_PATH -u LD_PRELOAD \
    bin/camera_server --config "${CAMERA_CONFIG}" \
    --vio-channel "${VIO_CHANNEL}" --color-channel "${COLOR_CHANNEL}" &
SERVER_PID=$!
sleep 0.5
if ! kill -0 "${SERVER_PID}" 2>/dev/null; then
    echo "[error] camera_server 启动失败（配置错误或设备被其他进程占用），见上方日志" >&2
    wait "${SERVER_PID}" 2>/dev/null || true
    SERVER_PID=0
    exit 1
fi

VIO_CMD=(bin/vio_live --shm "${VIO_CHANNEL}" --shm-timeout-ms 10000 "${VIO_ARGS[@]}")
if [[ "${NO_SEND}" -eq 1 ]]; then
    VIO_CMD+=(--no-send)
fi
echo "[info] 启动 vio_live（共享内存消费者）..."
env -u LD_LIBRARY_PATH -u LD_PRELOAD "${VIO_CMD[@]}" &
VIO_PID=$!

DET_CMD=(bin/detect_rt --shm "${COLOR_CHANNEL}" --shm-timeout-ms 10000 "${DET_ARGS[@]}")
if [[ "${ENABLE_STREAM}" -eq 1 ]]; then
    mkdir -p "${STREAM_DIR}"
    rm -f "${STREAM_DIR}/frame.jpg" "${STREAM_DIR}/frame_tmp.bin"
    DET_CMD+=(--stream-dir "${STREAM_DIR}" --stream-fps "${STREAM_FPS}")
fi
start_detection() {
    env -u LD_LIBRARY_PATH -u LD_PRELOAD "${DET_CMD[@]}" &
    DET_PID=$!
}
if [[ "${ENABLE_DETECTION}" -eq 1 ]]; then
    echo "[info] 启动 detect_rt（共享内存消费者）..."
    start_detection
fi

echo
echo "======================================================"
echo "  camera_server : ${VIO_CHANNEL} / ${COLOR_CHANNEL}"
echo "  vio_live      : PID ${VIO_PID}"
if [[ "${ENABLE_DETECTION}" -eq 1 ]]; then
    echo "  detect_rt     : PID ${DET_PID}"
else
    echo "  detect_rt     : 未启用（加 --detection 开启）"
fi
if [[ "${ENABLE_STREAM}" -eq 1 ]]; then
    echo "  画面推流      : 另开终端执行 ./scripts/start_detection_stream.sh"
fi
echo "  按 Ctrl-C 逆序停止全部进程"
echo "======================================================"
echo

while true; do
    sleep 0.5
    if ! kill -0 "${SERVER_PID}" 2>/dev/null; then
        echo "[error] camera_server 已退出，停止全部进程（重启请重新执行本脚本）" >&2
        break
    fi
    if ! kill -0 "${VIO_PID}" 2>/dev/null; then
        echo "[error] vio_live 已退出，停止全部进程" >&2
        break
    fi
    if [[ "${ENABLE_DETECTION}" -eq 1 && "${DET_PID}" -gt 0 ]] && ! kill -0 "${DET_PID}" 2>/dev/null; then
        wait "${DET_PID}" 2>/dev/null || true
        DET_PID=0
        if [[ "${DET_RESTARTS}" -lt 3 ]]; then
            DET_RESTARTS=$((DET_RESTARTS + 1))
            echo "[warn] detect_rt 退出（NPU 异常只会影响检测），第 ${DET_RESTARTS}/3 次重启；VIO 继续运行" >&2
            sleep 2
            start_detection
        else
            echo "[warn] detect_rt 反复退出，停止重启；VIO 与相机服务继续运行" >&2
        fi
    fi
done
