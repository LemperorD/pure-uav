#!/bin/bash
# =============================================================================
#  启动「YOLO 带框画面 → mjpg-streamer → http://<板子IP>:8080/」
#
#  原理：
#    detect_rt 通过 Orbbec SDK 独占相机，把带框图原子写成 <STREAM_DIR>/frame.jpg；
#    mjpg-streamer 的 input_file 插件监听该目录，再由 output_http 在 8080 输出。
#
#  ⚠ 相机只能被一个进程采集：运行本脚本前必须先停掉原来的 input_uvc 版
#    mjpg-streamer。可加 --replace 让脚本自动杀掉占用端口的进程（需要 sudo）。
#
#  用法：
#    ./scripts/start_detection_stream.sh [--port 8080] [--dir /dev/shm/pure-uav-detect]
#                                        [--replace] [detect_rt 的其它参数...]
#
#  示例：
#    ./scripts/start_detection_stream.sh --replace
#    ./scripts/start_detection_stream.sh --replace --conf 0.3 --camera 640x480@60
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MJPG_ROOT="${MJPG_ROOT:-/home/orangepi/Documents/mjpg-streamer/mjpg-streamer-experimental}"
STREAM_DIR="${STREAM_DIR:-/dev/shm/pure-uav-detect}"
PORT="${PORT:-8080}"
STREAM_FPS="${STREAM_FPS:-20}"

REPLACE=0
DETECT_ARGS=()

usage() {
    cat <<EOF
用法: $0 [--port N] [--dir DIR] [--replace] [detect_rt 参数...]

  --port N      mjpg-streamer HTTP 端口（默认 8080）
  --dir DIR     带框 JPG 交换目录（默认 /dev/shm/pure-uav-detect）
  --replace     先杀掉占用该端口的旧 mjpg-streamer（需要 sudo）
  -h, --help    显示帮助

其余参数原样传给 bin/detect_rt，例如 --conf 0.3 --camera 640x480@60 --frames 100。
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --port) PORT="$2"; shift 2 ;;
        --dir) STREAM_DIR="$2"; shift 2 ;;
        --replace) REPLACE=1; shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; DETECT_ARGS=("$@"); break ;;
        *) DETECT_ARGS+=("$1"); shift ;;
    esac
done

DETECT_BIN="${ROOT_DIR}/bin/detect_rt"
if [[ ! -x "${DETECT_BIN}" ]]; then
    echo "[error] 找不到 ${DETECT_BIN}，请先执行 ./scripts/autoBuild.sh" >&2
    exit 1
fi
if [[ ! -x "${MJPG_ROOT}/mjpg_streamer" ]]; then
    echo "[error] 找不到 ${MJPG_ROOT}/mjpg_streamer" >&2
    exit 1
fi
for f in input_file.so output_http.so; do
    if [[ ! -f "${MJPG_ROOT}/${f}" ]]; then
        echo "[error] 缺少 mjpg-streamer 插件: ${MJPG_ROOT}/${f}" >&2
        exit 1
    fi
done

port_in_use() {
    ss -ltn 2>/dev/null | awk '{print $4}' | grep -q ":${PORT}$"
}

if port_in_use; then
    if [[ "${REPLACE}" -eq 1 ]]; then
        echo "[info] 停止占用 ${PORT} 端口的旧进程（sudo fuser -k ${PORT}/tcp）..."
        sudo fuser -k "${PORT}/tcp" || true
        for _ in $(seq 1 20); do
            port_in_use || break
            sleep 0.2
        done
    else
        echo "[error] 端口 ${PORT} 已被占用（通常是原来的 mjpg-streamer）。" >&2
        echo "        请先在原终端 Ctrl-C 停掉，或重新执行并加 --replace。" >&2
        exit 1
    fi
fi
if port_in_use; then
    echo "[error] 端口 ${PORT} 仍被占用，退出。" >&2
    exit 1
fi

mkdir -p "${STREAM_DIR}"
rm -f "${STREAM_DIR}/frame.jpg" "${STREAM_DIR}/frame_tmp.jpg"

MJPG_PID=""
cleanup() {
    if [[ -n "${MJPG_PID}" ]]; then
        kill "${MJPG_PID}" 2>/dev/null || true
        wait "${MJPG_PID}" 2>/dev/null || true
        MJPG_PID=""
    fi
}
trap cleanup EXIT INT TERM

echo "[info] 启动 mjpg-streamer (input_file → output_http ${PORT})..."
env -u LD_LIBRARY_PATH -u LD_PRELOAD \
    "${MJPG_ROOT}/mjpg_streamer" \
    -i "${MJPG_ROOT}/input_file.so -f ${STREAM_DIR}/ -r -n frame.jpg" \
    -o "${MJPG_ROOT}/output_http.so -p ${PORT} -w ${MJPG_ROOT}/www" &
MJPG_PID=$!

for _ in $(seq 1 25); do
    port_in_use && break
    sleep 0.2
done
if ! port_in_use; then
    echo "[error] mjpg-streamer 未能监听 ${PORT}，请查看上面的日志。" >&2
    exit 1
fi

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
[[ -z "${IP}" ]] && IP="127.0.0.1"
echo
echo "======================================================"
echo "  带框画面: http://${IP}:${PORT}/"
echo "  视频流  : http://${IP}:${PORT}/?action=stream"
echo "  按 Ctrl-C 同时停止 detect_rt 与 mjpg-streamer"
echo "======================================================"
echo

env -u LD_LIBRARY_PATH -u LD_PRELOAD "${DETECT_BIN}" \
    "${DETECT_ARGS[@]}" \
    --stream-dir "${STREAM_DIR}" \
    --stream-fps "${STREAM_FPS}"
