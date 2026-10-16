#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Phase A 一键运行：XTDrone2 指令回写通路验证
#   前置：仿真已在另一个终端跑起来（见 README.md）
#   用法：./run_phase_a.sh                 # 默认 ns=x500_0
#         ./run_phase_a.sh --a-mag 0.3     # 更温柔的脉冲
# ---------------------------------------------------------------------------
set -e
source /opt/ros/jazzy/setup.bash
source "$HOME/XTDrone2/install/setup.bash"
cd "$(dirname "$0")"
exec python3 phase_a_accel_path.py "$@"
