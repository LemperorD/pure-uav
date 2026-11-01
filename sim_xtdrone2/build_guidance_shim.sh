#!/usr/bin/env bash
# =============================================================================
#  编译 pure_guidance_shim → libpure_guidance.so（WSL / Linux 侧）
#
#  流程：
#    1) 若 /mnt/d 仓库可读 → 从中刷新本地 guidance_libs 源码副本
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

# 仓库根：默认 D 盘路径；搬家/换机器时可用环境变量覆盖：
#   PURE_UAV_REPO=/path/to/pure-uav-main bash build_guidance_shim.sh
REPO_ROOT="${PURE_UAV_REPO:-/mnt/d/Users/Desktop/RM工作/RM无人机制导律/pure-uav-main}"
REPO_LIBS="$REPO_ROOT/src/guidance/guidance_libs"
# ★ 2026-10-20：`vec3.hpp` 已迁到 src/common/common_libs/include/（不再在 guidance_libs/include）
#   语义没变，但只从 guidance_libs 拷头文件就会漏掉它 ⇒ 这里显式多拷一处。
REPO_COMMON="$REPO_ROOT/src/common/common_libs"

# 1) 刷新源码副本（仓库不可读时退回本地已有副本，但会**明确告警**）
if [ -d "$REPO_LIBS" ]; then
    mkdir -p guidance_libs/include guidance_libs/src
    cp -f "$REPO_LIBS/include/"*.hpp guidance_libs/include/ 2>/dev/null || true
    cp -f "$REPO_LIBS/src/"*.cpp     guidance_libs/src/     2>/dev/null || true
    if [ -d "$REPO_COMMON/include" ]; then
        cp -f "$REPO_COMMON/include/vec3.hpp" guidance_libs/include/ 2>/dev/null || true
    fi
    echo "已从仓库刷新 guidance_libs 源码（含 common 的 vec3.hpp）：$REPO_LIBS"
else
    echo "!! 警告：找不到仓库路径：$REPO_LIBS" >&2
    echo "!! 若继续，会使用本地已有 guidance_libs 副本（可能过期，测试结果不可信）。" >&2
    echo "!! 确认要这样跑：ALLOW_LOCAL_COPY=1 bash build_guidance_shim.sh" >&2
    if [ "${ALLOW_LOCAL_COPY:-0}" != "1" ]; then
        exit 1
    fi
fi

if [ ! -f guidance_libs/src/target.cpp ] || [ ! -f pure_guidance_shim.cpp ] \
   || [ ! -f guidance_libs/src/quat.cpp ] || [ ! -f guidance_libs/include/vec3.hpp ]; then
    echo "!! 缺源码（guidance_libs/ 或 pure_guidance_shim.cpp 或 vec3.hpp），请让助手同步" >&2
    exit 1
fi

# 2) 编译
#    源文件列表 = guidance_core(guidance/pn_law/quat) + guidance_sim(sensor/target)
#    ⚠ 新增源文件时这里要同步（quat.cpp 于 2026-10-16 从 sensor.cpp 拆出）
g++ -O2 -std=c++17 -fPIC -shared \
    -I guidance_libs/include \
    pure_guidance_shim.cpp \
    guidance_libs/src/guidance.cpp \
    guidance_libs/src/pn_law.cpp \
    guidance_libs/src/quat.cpp \
    guidance_libs/src/sensor.cpp \
    guidance_libs/src/target.cpp \
    -o libpure_guidance.so
echo "编译完成："
ls -la libpure_guidance.so

# 3) 离线自检（应打印“全部通过 ✅”，退出码 0）
python3 test_guidance_shim.py
