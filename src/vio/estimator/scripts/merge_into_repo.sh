#!/usr/bin/env bash
#
# 把开发副本 ~/calib/vio_online 一键同步进 pure-uav-vio 仓库（src/vio/estimator）。
#
# 用法：
#   ./scripts/merge_into_repo.sh                       # 默认仓库 ~/桌面/pure-uav-vio
#   ./scripts/merge_into_repo.sh --repo /path/to/repo
#   ./scripts/merge_into_repo.sh --dry-run
#
# 做的事：
#   1. 同步本工程到 <repo>/src/vio/estimator（排除 build/out/datasets）
#   2. 在 <repo>/src/CMakeLists.txt 末尾追加 add_subdirectory(vio/estimator)（幂等）
#   3. 检查 Orbbec SDK / MAVLink 头文件，打印后续构建命令

set -euo pipefail

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="${HOME}/桌面/pure-uav-vio"
DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --repo)
            REPO="${2:?--repo 需要路径}"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        -h|--help)
            sed -n '2,20p' "$0"
            exit 0
            ;;
        *)
            echo "未知参数: $1" >&2
            exit 2
            ;;
    esac
done

if [[ ! -f "${REPO}/src/CMakeLists.txt" ]]; then
    echo "[错误] ${REPO} 看起来不是 pure-uav 仓库（缺 src/CMakeLists.txt）" >&2
    exit 1
fi

DEST="${REPO}/src/vio/estimator"
echo "源目录 : ${SRC_DIR}"
echo "目标   : ${DEST}"

RSYNC_OPTS=(
    -a
    --exclude 'build/'
    --exclude 'build_*/'
    --exclude 'out/'
    --exclude 'datasets/'
    --exclude '__pycache__/'
    --exclude '*.log'
    --exclude '*.bak'
    --exclude '.git/'
    --exclude '.idea/'
    --exclude '/Log/'
    --exclude '/--config-dir/'
    # 排查期的实验配置（config/_dbg_* / _sweep_* / _frz_* ...）不进仓库，
    # 仓库里只保留生产配置：gemini_1920x1080 / gemini_1280x720 / session05_1920x1080
    --exclude '/config/_*'
)

if [[ ${DRY_RUN} -eq 1 ]]; then
    echo "[dry-run] rsync 预览："
    rsync "${RSYNC_OPTS[@]}" --dry-run --itemize-changes "${SRC_DIR}/" "${DEST}/" | head -40
else
    mkdir -p "${DEST}"
    rsync "${RSYNC_OPTS[@]}" "${SRC_DIR}/" "${DEST}/"
    echo "[OK] 已同步到 ${DEST}"
fi

# 在仓库 src/CMakeLists.txt 里挂上子目录
SRC_CMAKE="${REPO}/src/CMakeLists.txt"
if grep -q "vio/estimator" "${SRC_CMAKE}"; then
    echo "[OK] src/CMakeLists.txt 已包含 vio/estimator"
else
    if [[ ${DRY_RUN} -eq 1 ]]; then
        echo "[dry-run] 会在 ${SRC_CMAKE} 末尾追加：add_subdirectory(vio/estimator)"
    else
        {
            echo ""
            echo "# VIO estimator（OpenVINS，ROS-free；由 ~/calib/vio_online 同步）"
            echo "add_subdirectory(vio/estimator)"
        } >> "${SRC_CMAKE}"
        echo "[OK] 已追加 add_subdirectory(vio/estimator) 到 ${SRC_CMAKE}"
    fi
fi

# 依赖检查
echo ""
echo "依赖检查："
ORBBEC_OK=0
MVLINK_OK=0
if [[ -f "${REPO}/src/driver/orbbec/include/libobsensor/ObSensor.hpp" ]]; then
    echo "  [OK] Orbbec SDK: ${REPO}/src/driver/orbbec"
    ORBBEC_OK=1
else
    echo "  [!!] 没找到 Orbbec SDK（${REPO}/src/driver/orbbec），实时程序会跳过"
fi

if [[ -f "${REPO}/src/driver/communicate/third_party/mavlink/common/mavlink.h" ]]; then
    echo "  [OK] MAVLink 头文件: ${REPO}/src/driver/communicate/third_party/mavlink"
    MVLINK_OK=1
elif [[ -f "${HOME}/桌面/pure-uav/pure-uav-communication/src/driver/communicate/third_party/mavlink/common/mavlink.h" ]]; then
    echo "  [OK] MAVLink 头文件（回退到 communication 分支）"
    MVLINK_OK=1
else
    echo "  [!!] 没找到 MAVLink 头文件，实时程序会跳过；可设 -DPURE_UAV_MAVLINK_DIR=..."
fi

echo ""
if [[ ${ORBBEC_OK} -eq 1 && ${MVLINK_OK} -eq 1 ]]; then
    echo "下一步（含实时程序）："
else
    echo "下一步（只会编译离线程序；实时依赖见上）："
fi
cat <<EOF

    cd ${REPO}
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j\$(nproc)

    # 离线回归
    ${REPO}/bin/vio_offline --dataset ~/calib/datasets/session05 \\
        --config-dir ${DEST}/config/session05_1920x1080

    # 实时（需要硬件；首次建议 --no-send --duration 30）
    ${REPO}/bin/vio_live --serial /dev/ttyACM0 --baud 921600 \\
        --config-dir ${DEST}/config/gemini_1920x1080

提示：
  - 配置文件路径在运行时相对当前目录解析，建议用绝对路径。
  - 正式单进程集成时，给 communicate 模块打 IMU 回调 patch：
        cd ~/桌面/pure-uav/pure-uav-communication
        patch -p1 < ${DEST}/patches/communicator_imu_callback.patch
EOF
