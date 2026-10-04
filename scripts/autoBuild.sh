#!/bin/bash

set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
BIN_DIR="${ROOT_DIR}/bin"

# 默认配置
CLEAN_BUILD=false
BUILD_TYPE="Release"

# 参数解析
while getopts "cdh" opt; do
case ${opt} in
c)
CLEAN_BUILD=true
;;
d)
BUILD_TYPE="Debug"
;;
h)
echo "Usage: $0 [options]"
echo
echo "Options:"
echo "  -c    Clean build and binary directories"
echo "  -d    Build in Debug mode"
echo "  -h    Show this help message"
echo
echo "Examples:"
echo "  $0          Release incremental build"
echo "  $0 -c       Release clean build"
echo "  $0 -d       Debug incremental build"
echo "  $0 -cd      Debug clean build"
exit 0
;;
?)
echo "Error: Invalid option -${OPTARG}"
echo "Use '$0 -h' for help."
exit 1
;;
esac
done

# 打印配置
echo "========================================"
echo "           pure-uav Build"
echo "========================================"
echo "Build type : ${BUILD_TYPE}"
echo "Clean      : ${CLEAN_BUILD}"
echo "Root       : ${ROOT_DIR}"
echo "Build dir  : ${BUILD_DIR}"
echo "========================================"

# Clean
if [ "${CLEAN_BUILD}" = true ]; then

    sudo -v

    echo "[Clean] Removing build directory..."
    if [ -d "${BUILD_DIR}" ]; then
        find "${BUILD_DIR}" -mindepth 1 \
            ! -name ".gitkeep" \
            -exec rm -rf {} +
    fi

    echo "[Clean] Removing binary directory..."
    if [ -d "${BIN_DIR}" ]; then
        find "${BIN_DIR}" -mindepth 1 \
            ! -name ".gitkeep" \
            -exec rm -rf {} +
    fi

    echo "[Clean] Done."

fi

# CMake Configure
echo
echo "[CMake] Configuring..."
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

# Build
echo
echo "[Build] Building..."
cmake --build "${BUILD_DIR}" --parallel "$(nproc)"
ln -s build/compile_commands.json compile_commands.json

echo
echo "========================================"
echo "Build finished successfully!"
echo "========================================"
