#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Phase B 一键运行：XTDrone2 制导闭环验证（PPN + 真 DART 目标）
#   前置：
#     1) 仿真已在另一个终端跑起来（见 README.md）
#     2) 制导核已编译：bash build_guidance_shim.sh（只需要一次）
#   用法：bash run_phase_b.sh                        # 默认场景
#         bash run_phase_b.sh --no-cruise            # 悬停直接制导（失败案例）
#         bash run_phase_b.sh --N 4 --a-max 8        # 换制导参数
# ---------------------------------------------------------------------------
set -e
source /opt/ros/jazzy/setup.bash
source "$HOME/XTDrone2/install/setup.bash"
cd "$(dirname "$0")"
exec python3 phase_b_guidance.py "$@"
