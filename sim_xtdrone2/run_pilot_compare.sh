#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# pilot 守备场景 · PPN / APN 实飞对比
#
#   场景（对齐 scene_model.preset_pilot，"无人机已在守备巡航"的初步仿真）：
#     塔    : 原点，半径 1.2 m、高 3 m、安全垫 1.0 m
#     无人机: 8 m 高，朝 −x（西，迎击方向）**匀速 5 m/s** 巡航，
#             飞到 x = −4 m 时弹丸发射（--cruise-x，守备到位判据）
#     弹丸  : 从 (−14, 0, 10) 以 14 m/s、**俯角 6°** 朝 +x 飞向塔
#             （末段下坠、指向塔 —— 不拦就会打到塔）
#
#   前置条件：
#     1) 仿真已在另一个终端跑起来（见 README.md §1）
#     2) 制导核已编译：bash build_guidance_shim.sh（只需要一次）
#
#   ★ Gazebo 可视化默认开启（--gz-viz --gz-trail）：在 GUI 里能看见防御塔、
#      弹丸飞向塔、被拦截的瞬间、以及事后画出的弹道线。
#      可视化**接不上 Gazebo 会自动降级**（只警告不报错），不影响制导结果。
#      不想要画面时加 --no-gz-viz。
#
#   ★★ 默认在每个 law 之间**暂停**，等你重启仿真后再继续 ★★
#      原因：跑完一个 law 后节点会 LAND + DISARM，无人机停在**拦截点附近**而非原点。
#      紧接着跑下一个 law，无人机就从那里起飞 ⇒
#        ① 到 --cruise-x 的位移变短（甚至起点已越过阈值，一达标就发射）；
#        ② 弹丸发射点是**绝对坐标**，与无人机的相对几何随之改变 ⇒ 两个 law 不可比。
#      节点在 SETTLE 阶段也会自动检测并把这件事**显式 warn**出来（不静默）。
#      重启仿真 = 在跑 launch 的那个终端里 Ctrl-C，再重跑 launch，约 20 s。
#
#   用法：
#     bash run_pilot_compare.sh              # 默认 PPN(0) + APN(2)，带画面，中间暂停等重启
#     bash run_pilot_compare.sh 0            # 只跑 PPN
#     bash run_pilot_compare.sh 0 2 1        # 自定义序列（0=PPN 1=TPN 2=APN）
#     bash run_pilot_compare.sh --no-pause 0 2   # 不暂停（几何会漂移，仅用于快速冒烟）
#     bash run_pilot_compare.sh --no-gz-viz 0 2  # 关掉 Gazebo 可视化（纯控制台）
#
#   产物：phase_b_pilot_law<k>.csv（每个 law 一份）
# ---------------------------------------------------------------------------
set -e

# ---- 解析参数：可选 --no-pause / --no-gz-viz，其余当作 law 列表 ----
# （放在 source ROS 之前，这样 --help 在没有 ROS 的机器上也能用）
PAUSE=1
GZVIZ=1
LAWS=()
for a in "$@"; do
    case "$a" in
        --no-pause) PAUSE=0 ;;
        --no-gz-viz) GZVIZ=0 ;;
        -h|--help) sed -n '2,46p' "$0"; exit 0 ;;
        *) LAWS+=("$a") ;;
    esac
done
if [ ${#LAWS[@]} -eq 0 ]; then
    LAWS=(0 2)
fi

source /opt/ros/jazzy/setup.bash
source "$HOME/XTDrone2/install/setup.bash"
cd "$(dirname "$0")"

# pilot 守备场景参数（与 scene_model.preset_pilot 一致）
#
# ⚠ 一律用 `--opt=值` 的**等号**形式，不要写成 `--opt 值`：
#   argparse 会把「以 - 开头、但不是纯数字」的值（如 `-14,0,10`）误判成**选项名**，
#   报 `error: argument --target-p0: expected one argument`。
#   （纯负数如 `-6` / `-4` 是例外，argparse 有专门的负数规则，但统一用 = 更省心。）
PILOT_ARGS=(
    --target-height=8
    --target-p0=-14,0,10
    --target-speed=14
    --target-elev=-6
    --target-azim=0
    --v-boost-min=5
    --a-boost=6
    --boost-dir=-1,0,0
    --cruise-x=-4
)

# Gazebo 可视化（在 GUI 里看见弹丸飞向塔 / 被拦截）
#   失败自动降级（不接 Gazebo 也能跑，只是看不到画面），不影响制导结果
if [ "$GZVIZ" -eq 1 ]; then
    PILOT_ARGS+=(--gz-viz --gz-trail)
fi

law_name() {
    case "$1" in
        0) echo "PPN" ;;
        1) echo "TPN" ;;
        2) echo "APN" ;;
        *) echo "law$1" ;;
    esac
}

echo "=============================================================="
echo "  pilot 守备场景实飞对比"
echo "  场景：无人机 8 m 高 · 5 m/s 迎击 · 飞到 x=−4 发射"
echo "        弹丸 (−14,0,10) 14 m/s 俯角 6° → 塔（原点）"
echo "  序列：$(for l in "${LAWS[@]}"; do printf '%s ' "$(law_name "$l")"; done)"
echo "  间隔暂停：$([ $PAUSE -eq 1 ] && echo '开（推荐，需手动重启仿真）' || echo '关')"
echo "  Gazebo 可视化：$([ $GZVIZ -eq 1 ] && echo '开（塔 + 弹丸 + 轨迹）' || echo '关')"
echo "=============================================================="
echo

idx=0
for LAW in "${LAWS[@]}"; do
    NAME="$(law_name "$LAW")"
    CSV="phase_b_pilot_law${LAW}.csv"

    # ---- 第一个 law 之前：确认起点干净 ----
    if [ $idx -eq 0 ]; then
        echo "--------------------------------------------------------------"
        echo "  本 law 开始前，请确认无人机处在**原点附近**（刚起飞的状态）。"
        echo "  · 若刚重启过仿真 → 没问题，直接继续。"
        echo "  · 若是紧接着上一次跑完 → 起点已漂到拦截点，请先重启仿真。"
        echo "--------------------------------------------------------------"
        echo
    else
        # ---- 后续 law 之前：暂停，检查上一轮是否归位成功 ----
        echo "=============================================================="
        echo "  下一个 law（${NAME}）之前"
        echo "=============================================================="
        echo "  ★ 本节点收尾时会把无人机**飞回起点**再降落（--return-to-start，默认开）。"
        echo "    若上一轮日志里出现了这一行："
        echo "        ✓ 已归位到起点：水平误差 x.xx m、|v_h|=x.xx m/s"
        echo "    则**不必重启仿真**，直接按 Enter 继续即可（起飞前自检会替你把关）。"
        echo
        echo "    若看到的是『⚠ 归位超时』或『⚠⚠ 收尾机动中离地仅』："
        echo "    请重启仿真："
        echo "      1) 到跑 launch 的终端按 Ctrl-C 停掉仿真"
        echo "      2) ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py"
        echo "    等约 20 s（QGC 里能看到无人机重新出现在原点）"
        echo "--------------------------------------------------------------"
        if [ -t 0 ]; then
            read -r -p "  继续请按 Enter（Ctrl-C 中止）…" _ || true
        else
            echo "  （stdin 非终端，跳过暂停）"
        fi
        echo
    fi
    idx=$((idx + 1))

    echo "=============================================================="
    echo "  [law ${LAW}] ${NAME}  →  ${CSV}"
    echo "=============================================================="

    # 不用 set -e 中断：未命中/超时也可能返回非零，但那正是要对比的结果之一
    set +e
    python3 phase_b_guidance.py --law "$LAW" --csv "$CSV" "${PILOT_ARGS[@]}"
    RC=$?
    set -e
    echo
    echo "  （law ${LAW} ${NAME} 结束，退出码 ${RC}）"
    echo
done

echo "=============================================================="
echo "  pilot 对比完成"
echo "=============================================================="
for LAW in "${LAWS[@]}"; do
    NAME="$(law_name "$LAW")"
    CSV="phase_b_pilot_law${LAW}.csv"
    if [ -f "$CSV" ]; then
        # 顺便把"命中/最近距离"从 CSV 里摘出来，省得手工翻
        echo "  ${NAME}(law ${LAW}) → ${CSV}  （$(wc -l < "$CSV") 行）"
    else
        echo "  ${NAME}(law ${LAW}) → ${CSV}  ✗ 未生成"
    fi
done
echo
cat <<'EOF'
  ★ 回传方法（推荐，最省事）：
     每个 law 跑完，终端末尾会打印一块
         <<<<<<< BEGIN RM-PHASE-B-REPORT >>>>>>>
         ...
         <<<<<<< END RM-PHASE-B-REPORT >>>>>>>
     把**整块**（含起止标记）复制粘贴回来即可 —— 里面已经有：场景参数、
     发射几何、制导段逐 0.1 s 时间序列（位置/速度/姿态/指令/R_true）、
     命中统计、着陆质量。不用再翻日志、也不用传 CSV。

     若还想给完整 CSV（列更多：NED 位姿、目标真值、四元数…）：
         phase_b_pilot_law0.csv / phase_b_pilot_law2.csv
EOF
echo "=============================================================="
exit 0
