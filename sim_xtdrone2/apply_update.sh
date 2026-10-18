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

# 4) 版本号副本（<词干>_vN.<ext>）：取修改时间最新的换到正式名，其余清理。
#    ⚠ 必须清理其余的，否则下次同步会误选到过期副本。
#
#    ★ 为什么用**白名单**而不是全自动扫描（2026-10-16 实测教训）：
#      试过全自动扫描（靠 ${f%%_v*} / ${f##*.} 推词干和扩展名），dry-run 暴露两个真缺陷：
#        ① "同词干只处理一次"的判断会被**外层循环顺序**影响 ⇒ 可能换上**更旧**的副本；
#        ② `xxx_v8.sh.tmp` 这种多段命名会把扩展名推成 `tmp`，占掉别的文件的名额。
#      ⇒ 白名单（词干明确、组内按 mtime 取最新）是安全的；再补一个
#        "未列出的 _vN 副本" 检测，保证新增脚本**不会静默漏换**。
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
    run_phase_b.sh \
    gz_scene_sync.py \
    verify_gz_msgs.py \
    audit_landing.py \
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

# 4.1) 清理历史的 apply_update_vN.sh。
#      ⚠ 本脚本的自我更新只由**步骤 2**（用 $SELF = 正在运行的那个）负责，
#        **绝不能**放进上面的白名单 —— 否则会按 mtime 在 v2…v8 之间竞争，
#        有可能把 apply_update.sh 覆盖成**更旧**的版本（dry-run 实测复现过）。
for f in apply_update_v*.sh; do
    [ -e "$f" ] || continue
    [ "$f" = "$SELF" ] && continue      # 正在运行的那个先留着（下次同步再清）
    rm -f "$f" && echo "  已清理历史换版脚本: $f" || echo "  ⚠ 清理失败（可忽略）: $f"
done

# 4.2) ★ 兜底：找出"白名单没覆盖"的 _vN 副本并**醒目提醒**（不静默漏换）
#      匹配形如 <词干>_v<数字>.<扩展名> 的文件（正则锚定结尾 ⇒ 天然排除 .sh.tmp 这类）
_orphan=0
for f in $(ls -1 *_v[0-9]*.* 2>/dev/null || true); do
    [ -f "$f" ] || continue
    [[ "$f" =~ ^(.+)_v[0-9]+\.([A-Za-z0-9]+)$ ]] || continue
    _stem="${BASH_REMATCH[1]}"
    _ext="${BASH_REMATCH[2]}"
    case "$_ext" in py|sh|cpp|hpp|h|md|txt|json|yaml|yml) ;; *) continue ;; esac
    # 白名单里已处理过的词干 → 跳过（此时副本应已被 mv 走；若还在说明 mv 失败）
    _listed=0
    for base in phase_a_accel_path.py phase_b_guidance.py scene_model.py \
                viz_scene.py test_phase_a_states.py test_phase_b_states.py \
                sim_phase_b_offline.py pure_guidance_shim.cpp preflight.sh \
                run_pilot_compare.sh run_phase_b.sh gz_scene_sync.py verify_gz_msgs.py \
                audit_landing.py \
                build_guidance_shim.sh apply_update.sh ; do
        [ "$_stem.$_ext" = "$base" ] && _listed=1 && break
    done
    [ "$_listed" -eq 1 ] && continue
    if [ $_orphan -eq 0 ]; then
        echo
        echo "⚠⚠ 发现**未列入白名单**的待换版副本（本次不会自动换）："
        _orphan=1
    fi
    echo "     $f   →   目标应为 ${_stem}.${_ext}"
done
if [ $_orphan -eq 1 ]; then
    echo "     处理：把 \"${_stem}.${_ext}\" 加进本脚本第 4 步的白名单，或手动 mv。"
    echo "     （加白名单后重跑本脚本即可）"
    echo
fi

# 4.5) 顺手清掉助手误写的临时文件（避免长期堆积）
rm -f guidance_CMakeLists.txt.txt *.orig *.rej 2>/dev/null || true

# 5) 补回执行位（Windows 侧写入的文件没有执行位）
chmod +x phase_a_accel_path.py test_phase_a_states.py \
         phase_b_guidance.py  test_phase_b_states.py sim_phase_b_offline.py \
         scene_model.py viz_scene.py \
         run_phase_a.sh run_phase_b.sh run_pilot_compare.sh apply_update.sh \
         preflight.sh build_guidance_shim.sh \
         gz_scene_sync.py verify_gz_msgs.py \
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

==============================================================
  ⚠⚠ 本脚本**只做「换版 + 离线自检」——不会让无人机起飞**。
      上面那一大堆 PASS 都是**离线回归测试**（假 PX4 桩，纯数学），
      Gazebo / XTDrone2 里**不会有任何动作**，这是正常的。

      要真正飞，必须在**另一个终端**执行：
          bash run_pilot_compare.sh
     并且先确认仿真已在跑（见下）。
      判断有没有真飞：终端里应出现
          [INFO] [...][phase_b_guidance]:   → 发出 ARM
      只有 PASS 没有 [INFO] ⇒ 飞行节点根本没启动。
==============================================================

== 下一步 ==
先确认仿真在跑（终端 1），然后**另开一个终端**（终端 2）：

    终端 1：ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py
    终端 2：bash run_pilot_compare.sh          # pilot 守备场景：PPN + APN 对比（★推荐）
                                              # 默认在 Gazebo 里显示防御塔 + 弹丸 + 弹道
                                              # 加 --no-gz-viz 关掉画面

    其它工具（都不需要仿真在跑，除非另有说明）：
    python3 gz_scene_sync.py --demo            # 只验证 Gazebo 可视化链路（需要仿真正在跑）
    python3 gz_scene_sync.py --clear           # 清掉塔 / 弹丸 / 轨迹
    python3 test_phase_b_states.py             # 单独跑离线回归（就是上面那些 PASS）
    bash run_phase_b.sh --namespace x500_0     # 阶段 B：原演示场景（制导闭环）
    bash run_phase_a.sh --namespace x500_0     # 阶段 A：指令通路（如需复测）

★ 归位已默认开启（--return-to-start）：每个 law 跑完会把无人机**飞回起点**再降落，
  所以**下一个 law 不必重启仿真** —— 只要上一轮日志里有
      ✓ 已归位到起点：水平误差 x.xx m
  若看到「⚠ 归位超时」，那才需要重启仿真（节点也会提示）。

跑完会自动归位 + LAND + 停桨并退出（按一次 Ctrl-C 也会走同样的收尾流程；
连按两次 Ctrl-C = 强制退出）。
EOF
