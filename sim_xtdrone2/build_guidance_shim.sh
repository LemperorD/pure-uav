#!/usr/bin/env bash
# =============================================================================
#  编译 pure_guidance_shim → libpure_guidance.so（WSL / Linux 侧）
#
#  流程：
#    1) 若 /mnt/c 仓库可读 → 从中刷新本地 guidance_libs 源码副本
#       （单一真源始终在仓库里；本地副本只是编译工作区）；
#    2) g++ 编成 .so；
#    3) 跑离线自检 test_guidance_shim.py（不需要 ROS / 仿真）。
#
#  用法：
#      cd ~/rm_guidance_sim
#      bash build_guidance_shim.sh
#
#  ⚠ 若 phase_b 节点正在运行，请先停掉（运行中的进程会锁住 .so）。
# =============================================================================
set -e
cd "$(dirname "$0")"

REPO_LIBS="/mnt/c/Users/wangyipeng/Desktop/RM工作/RM无人机制导律/pure-uav-main/src/guidance/guidance_libs"

# 1) 刷新源码副本（/mnt/c 不可读时退回本地已有副本）
if [ -d "$REPO_LIBS" ]; then
    mkdir -p guidance_libs/include guidance_libs/src
    cp -f "$REPO_LIBS/include/"*.hpp guidance_libs/include/ 2>/dev/null || true
    cp -f "$REPO_LIBS/src/"*.cpp     guidance_libs/src/     2>/dev/null || true
    echo "已从 /mnt/c 仓库刷新 guidance_libs 源码"
else
    echo "（/mnt/c 仓库不可读，使用本地 guidance_libs 副本）"
fi

if [ ! -f guidance_libs/src/target.cpp ] || [ ! -f pure_guidance_shim.cpp ]; then
    echo "!! 缺源码（guidance_libs/ 或 pure_guidance_shim.cpp），请让助手同步" >&2
    exit 1
fi

# 2) 编译
g++ -O2 -std=c++17 -fPIC -shared \
    -I guidance_libs/include \
    pure_guidance_shim.cpp \
    guidance_libs/src/guidance.cpp \
    guidance_libs/src/pn_law.cpp \
    guidance_libs/src/sensor.cpp \
    guidance_libs/src/target.cpp \
    -o libpure_guidance.so
echo "编译完成："
ls -la libpure_guidance.so

# 3) 离线自检（应打印“全部通过 ✅”，退出码 0）
python3 test_guidance_shim.py
