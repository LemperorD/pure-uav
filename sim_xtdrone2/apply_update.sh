#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# 把 WSL 运行目录里的"待启用副本"换成正式文件。
#
# 为什么需要这个脚本：
#   助手在 Windows 侧通过 UNC 路径（//wsl.localhost/...）写文件，但**运行中的
#   ROS2 节点会锁住 .py 句柄**，导致 Windows 侧覆盖报 `Permission denied`。
#   于是助手改用"写一个新文件名的副本"，由你在 WSL 里执行本脚本换上去。
#
# 支持的副本命名（两大类，都会自动识别）：
#   · <正式名>.new          —— 如 phase_b_guidance.py.new / README.md.new
#   · <词干>_vN.py          —— 如 phase_b_guidance_v2.py（取修改时间最新的一个）
#
# 用法（在 WSL 里）：
#   cd ~/rm_guidance_sim
#   bash apply_update.sh
# ---------------------------------------------------------------------------
set -e
cd "$(dirname "$0")"

echo "== 运行目录: $(pwd) =="

# 1) 清掉 __pycache__（否则可能加载到旧字节码）
rm -rf __pycache__ && echo "已清理 __pycache__"

# 2) 自我更新：本脚本若不是 apply_update.sh（如 apply_update_v4.sh），
#    就把自己也换到最新（被锁时静默失败，不影响主流程）
SELF="$(basename "$0")"
if [ "$SELF" != "apply_update.sh" ]; then
    if cp -f "$SELF" apply_update.sh 2>/dev/null; then
        echo "已自我更新: $SELF → apply_update.sh"
    else
        echo "（apply_update.sh 被占用，跳过自我更新；$SELF 保留）"
    fi
fi
chmod +x apply_update.sh 2>/dev/null || true

# 3) 通用换版：任何 "<name>.new" 都换到 "<name>"（round 1 —— 精确 .new 优先）
for f in *.new; do
    [ -e "$f" ] || continue
    tgt="${f%.new}"
    if [ -e "$tgt" ]; then
        echo "启用: $f  →  $tgt"
        mv -f "$f" "$tgt" || echo "  ⚠ 换版失败: $f"
    else
        echo "  （跳过 $f：目标 $tgt 不存在）"
    fi
done

# 3.5) README（约定用 "README.md.updated" / "README.md.updated_vN" 命名）
mapfile -t _rd < <(ls -1t README.md.updated* 2>/dev/null || true)
if [ "${#_rd[@]}" -gt 0 ]; then
    if cp -f "${_rd[0]}" README.md 2>/dev/null; then
        rm -f "${_rd[@]}" 2>/dev/null || true
        echo "已更新: ${_rd[0]} → README.md"
    else
        echo "  ⚠ README.md 被占用，保留 ${_rd[0]}"
    fi
fi
rm -f README.md.new 2>/dev/null || true

# 4) 版本号副本（<词干>_vN.py）：取修改时间最新的换到正式名，其余清理。
#    ⚠ 必须清理其余的，否则下次同步会误选到过期副本。
for base in \
    phase_a_accel_path.py \
    phase_b_guidance.py \
    scene_model.py \
    viz_scene.py \
    test_phase_a_states.py \
    test_phase_b_states.py \
    sim_phase_b_offline.py \
    pure_guidance_shim.cpp \
    preflight.sh \
    run_pilot_compare.sh \
    build_guidance_shim.sh ; do
    stem="${base%.*}"
    ext="${base##*.}"
    mapfile -t cands < <(ls -1t "${stem}_v"*."${ext}" 2>/dev/null || true)
    [ "${#cands[@]}" -gt 0 ] || continue
    echo "启用: ${cands[0]}  →  $base"
    mv -f "${cands[0]}" "$base"
    for f in "${cands[@]:1}"; do
        rm -f "$f" && echo "  已清理过期副本: $f" || echo "  ⚠ 清理失败（被锁，可忽略）: $f"
    done
done

# 5) 补回执行位（Windows 侧写入的文件没有执行位）
chmod +x phase_a_accel_path.py test_phase_a_states.py \
         phase_b_guidance.py  test_phase_b_states.py sim_phase_b_offline.py \
         scene_model.py viz_scene.py \
         run_phase_a.sh run_phase_b.sh run_pilot_compare.sh apply_update.sh \
         preflight.sh build_guidance_shim.sh \
         2>/dev/null || true

echo
echo "== 当前文件 =="
ls -la

echo
echo "== 离线自检（不需要 ROS / 仿真）=="
echo "-- 阶段 A 状态机 --"
python3 test_phase_a_states.py || echo "⚠ 阶段 A 自检未全过（见上）"

if [ -f test_phase_b_states.py ]; then
    echo
    echo "-- 阶段 B 制导闭环（需要 libpure_guidance.so；没有会提示先编译）--"
    if ls ./*pure_guidance.so >/dev/null 2>&1 || [ -f ./libpure_guidance.so ]; then
        python3 test_phase_b_states.py || echo "⚠ 阶段 B 自检未全过（见上）"
    else
        echo "（跳过：还没编译制导核 —— 先跑 bash build_guidance_shim.sh）"
    fi
fi

cat <<'EOF'

== 下一步 ==
先确认仿真在跑（另一个终端），然后：

    bash run_pilot_compare.sh                  # pilot 守备场景：PPN + APN 对比（★推荐）
    bash run_phase_b.sh --namespace x500_0     # 阶段 B：原演示场景（制导闭环）
    bash run_phase_a.sh --namespace x500_0     # 阶段 A：指令通路（如需复测）

★ run_pilot_compare.sh 跑第二个 law 之前**必须重启仿真**
  （跑完无人机停在拦截点而非原点，否则几何漂移；节点会自动 warn 提示）。

跑完会自动 LAND + 停桨并退出（按一次 Ctrl-C 也会走同样的收尾流程；
连按两次 Ctrl-C = 强制退出）。
EOF
