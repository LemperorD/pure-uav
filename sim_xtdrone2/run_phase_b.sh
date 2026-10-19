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
# ★ 2026-10-18：节点默认已改 `auto`（弹丸自动对准塔）。本脚本是**阶段 B 历史演示**
#   （"制导闭环能不能命中"），不是守备拦截场景 ⇒ 这里把旧参数**显式固化**，
#   保证输出与历史记录可比。放在 "$@" 之前 ⇒ 你在命令行传的值仍然能覆盖。
#   想用自动瞄准：bash run_phase_b.sh --target-elev=auto --target-azim=auto
exec python3 phase_b_guidance.py \
    --target-elev=5 --target-azim=0 --boost-dir=-1,0,0 "$@"
