#!/usr/bin/env bash
# =============================================================================
#  起仿真前的预检 —— 把"能不能跑"一次性确认掉，省得到节点跑一半才发现缺东西。
#
#  检查项：
#    1) 环境：/opt/ros/jazzy、~/XTDrone2/install
#    2) 文件：节点 / 运行脚本 / 离线工具 是否齐全
#    3) 制导核：libpure_guidance.so 是否存在（缺了就顺手编译）
#    4) 待换版副本：提醒还没 apply_update 的 _vN / README.md.updated*
#    5) （--wait）等仿真起来：轮询 /<ns>/fmu/out/vehicle_status
#
#  用法（在 WSL 里）：
#      cd ~/rm_guidance_sim
#      bash preflight.sh                 # 只做自检
#      bash preflight.sh --wait          # 自检 + 等仿真就绪（最多 120 s）
#      NS=x500_0 bash preflight.sh --wait
# =============================================================================
set -u
cd "$(dirname "$0")"
NS="${NS:-x500_0}"
FAIL=0

ok()  { echo "  ✓ $*"; }
bad() { echo "  ✗ $*"; FAIL=1; }
info(){ echo "  · $*"; }

echo "=============================================================="
echo " XTDrone2 仿真预检   (ns=$NS, 目录 $(pwd))"
echo "=============================================================="

# ---------------------------------------------------------------- 1) 环境
echo "== 1) 环境 =="
if [ -f /opt/ros/jazzy/setup.bash ]; then
    ok "ROS2 Jazzy 存在"
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
else
    bad "找不到 /opt/ros/jazzy/setup.bash —— ROS2 没装或版本不对"
fi
if [ -f "$HOME/XTDrone2/install/setup.bash" ]; then
    ok "XTDrone2 已编译（$HOME/XTDrone2/install）"
    # shellcheck disable=SC1091
    source "$HOME/XTDrone2/install/setup.bash"
else
    bad "找不到 ~/XTDrone2/install/setup.bash —— 先 colcon build"
fi
if command -v ros2 >/dev/null 2>&1; then ok "ros2 命令可用"; else bad "ros2 命令不可用"; fi

# ---------------------------------------------------------------- 2) 文件
echo "== 2) 文件 =="
for f in phase_b_guidance.py run_phase_b.sh phase_a_accel_path.py run_phase_a.sh \
         sim_phase_b_offline.py scene_model.py viz_scene.py \
         test_phase_a_states.py test_phase_b_states.py \
         pure_guidance_shim.cpp build_guidance_shim.sh; do
    if [ -f "$f" ]; then ok "$f"; else bad "缺 $f"; fi
done

# ---------------------------------------------------------------- 3) 制导核
echo "== 3) 制导核（C++ → .so）=="
if [ -f ./libpure_guidance.so ]; then
    ok "libpure_guidance.so 已存在（$(stat -c%s ./libpure_guidance.so) B）"
    if [ ./libpure_guidance.so -ot ./pure_guidance_shim.cpp ]; then
        info "⚠ .so 比 shim 源码旧 —— 建议重新编译：bash build_guidance_shim.sh"
    fi
else
    info "未编译，现在编译……"
    if bash build_guidance_shim.sh; then ok "编译完成"; else bad "编译失败（见上）"; fi
fi

# ---------------------------------------------------------------- 4) 待换版副本
echo "== 4) 待换版副本 =="
STALE="$(ls -1 *_v[0-9]*.py README.md.updated* 2>/dev/null || true)"
if [ -n "$STALE" ]; then
    echo "$STALE" | sed 's/^/  ! 还没换版: /'
    info "→ 先跑：bash apply_update_v4.sh"
else
    ok "没有待换版副本"
fi

# ---------------------------------------------------------------- 5) 等仿真
if [ "${1:-}" = "--wait" ]; then
    echo "== 5) 等仿真就绪（topic /$NS/fmu/out/vehicle_status）=="
    info "如果还没起仿真，请另开终端跑："
    info "  source /opt/ros/jazzy/setup.bash && source ~/XTDrone2/install/setup.bash"
    info "  ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py"
    FOUND=0
    for i in $(seq 1 60); do
        if ros2 topic list 2>/dev/null | grep -qx "/$NS/fmu/out/vehicle_status"; then
            ok "仿真已就绪（等了 $((i * 2)) s）"
            FOUND=1
            break
        fi
        printf '\r  … 等待中 %ds ' $((i * 2))
        sleep 2
    done
    echo
    [ "$FOUND" -eq 1 ] || bad "等了 120 s 仍未见 topic —— 终端 1 的 launch 起来了吗？"
fi

echo "=============================================================="
if [ "$FAIL" -eq 0 ]; then
    echo "预检通过 ✅"
    echo
    echo "下一步："
    echo "  终端 2（另开）：bash run_phase_b.sh --namespace $NS"
    echo "  · 关注日志： [1/6] 已解锁 → [4/6] nav_state=14 → [6/6] 目标发射 → ★ 命中！"
    echo "  · 跑完自动 LAND + 停桨；一次 Ctrl-C 也会走同样收尾，两次 = 强制退出。"
else
    echo "预检有问题 ❌ —— 见上面带 ✗ 的项"
fi
echo "=============================================================="
exit "$FAIL"
