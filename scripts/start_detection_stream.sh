#!/bin/bash
# =============================================================================
#  启动「YOLO 带框画面 → mjpg-streamer → http://<板子IP>:8080/」
#
#  原理：
#    camera_server 独占相机，detect_rt 从共享内存彩色通道取帧，把带框图原子写成
#    <STREAM_DIR>/frame.jpg；mjpg-streamer 的 input_file 插件监听该目录，
#    再由 output_http 在 8080 输出。
#
#  ⚠ 本脚本只负责 mjpg-streamer。相机服务与 VIO 由外部托管启动，
#    detect_rt 由 bin/main 心跳后拉起（或单独运行），需带
#    --stream-dir / --stream-fps 写出带框图。
#    可加 --replace 让脚本自动杀掉占用端口的进程（需要 sudo）。
#
#  用法：
#    ./scripts/start_detection_stream.sh [--port 8080] [--dir /dev/shm/pure-uav-detect]
#                                        [--replace]
#
#  示例：
#    ./bin/main --detect-arg --stream-dir --detect-arg /dev/shm/pure-uav-detect \
#      --detect-arg --stream-fps --detect-arg 20          # 另一个终端
#    ./scripts/start_detection_stream.sh --replace
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MJPG_ROOT="${MJPG_ROOT:-/home/orangepi/Documents/mjpg-streamer/mjpg-streamer-experimental}"
STREAM_DIR="${STREAM_DIR:-/dev/shm/pure-uav-detect}"
PORT="${PORT:-8080}"
STREAM_FPS="${STREAM_FPS:-20}"

REPLACE=0

usage() {
    cat <<EOF
用法: $0 [--port N] [--dir DIR] [--replace]

  --port N      mjpg-streamer HTTP 端口（默认 8080）
  --dir DIR     带框 JPG 交换目录（默认 /dev/shm/pure-uav-detect）
  --replace     先杀掉占用该端口的旧 mjpg-streamer（需要 sudo）
  -h, --help    显示帮助

相机服务与 VIO 由外部托管启动；detect_rt 由 bin/main 拉起（或单独运行）并写 frame.jpg。
检测算法参数（模型/阈值/类别/限频/帧龄）唯一真源为 config/detector-rt.yaml。
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --port) PORT="$2"; shift 2 ;;
        --dir) STREAM_DIR="$2"; shift 2 ;;
        --replace) REPLACE=1; shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; break ;;
        *)
            echo "[error] 未知参数: $1（检测算法参数请改 config/detector-rt.yaml）" >&2
            usage
            exit 2
            ;;
    esac
done

if [[ ! -x "${ROOT_DIR}/bin/camera_server" || ! -x "${ROOT_DIR}/bin/detect_rt" ]]; then
    echo "[error] 缺少 camera_server / detect_rt，请先执行 ./scripts/autoBuild.sh" >&2
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

if ! awk -v f="${STREAM_FPS}" 'BEGIN { exit !(f >= 1 && f <= 60) }'; then
    echo "[error] STREAM_FPS 必须在 [1,60] 内，当前: ${STREAM_FPS}" >&2
    exit 1
fi

mkdir -p "${STREAM_DIR}"
rm -f "${STREAM_DIR}/frame.jpg" "${STREAM_DIR}/frame_tmp.jpg" "${STREAM_DIR}/frame_tmp.bin"

# input_file 的 --existing 模式按固定延迟轮询 frame.jpg。
# 不能用 inotify + --remove 模式：写入速率一高，inotify 事件会堆积，
# 插件每次 read 只处理第一个事件，实际输出会掉到 1 FPS 左右。
DELAY="$(awk -v f="${STREAM_FPS}" 'BEGIN { printf "%.3f", 1.0 / f }')"

MJPG_PID=""
cleanup() {
    if [[ -n "${MJPG_PID}" ]]; then
        kill "${MJPG_PID}" 2>/dev/null || true
        wait "${MJPG_PID}" 2>/dev/null || true
        MJPG_PID=""
    fi
}
trap cleanup EXIT INT TERM

# 相机服务与 VIO 由外部托管，detect_rt 由 bin/main 拉起；这里只等首帧就绪，
# 因为 input_file --existing 模式启动时扫描一次目录，目录为空会直接退出。
echo "[info] 等待 detect_rt 首帧（需先启动 bin/main 或 detect_rt，带 --stream-dir ${STREAM_DIR}）..."
for _ in $(seq 1 100); do
    [[ -f "${STREAM_DIR}/frame.jpg" ]] && break
    sleep 0.1
done
if [[ ! -f "${STREAM_DIR}/frame.jpg" ]]; then
    echo "[error] 等待首帧超时。请先在另一个终端启动 bin/main（或单独运行 detect_rt），带 --stream-dir ${STREAM_DIR}" >&2
    exit 1
fi

echo "[info] 启动 mjpg-streamer (input_file --existing, ${DELAY}s/帧 → output_http ${PORT})..."
env -u LD_LIBRARY_PATH -u LD_PRELOAD \
    "${MJPG_ROOT}/mjpg_streamer" \
    -i "${MJPG_ROOT}/input_file.so -f ${STREAM_DIR}/ -e -d ${DELAY}" \
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
echo "  按 Ctrl-C 停止 mjpg-streamer（相机服务/VIO 外部托管，检测由 bin/main 管理）"
echo "======================================================"
echo

wait "${MJPG_PID}"
