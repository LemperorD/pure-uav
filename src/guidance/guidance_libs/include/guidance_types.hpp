#pragma once

#include "vec3.hpp"

// 制导模块的数据结构定义。
// 字段严格对齐《输入数据字典.md》与《输出数据字典.md》，
// 这样仿真阶段和接真机阶段用的是同一套结构，只换数据来源。
//
// 【边界假设】制导假定上游数据处理已完成：
//   目标检测 → û / R；定位惯导 → q / v_M；视觉 → λ̇；
//   这些都已算好并填进 GuidanceState，制导只消费 GuidanceState。
//   "上游处理是否要单独写一个 cpp、归属谁" 见《待决问题.md》Q1。

namespace pure_uav {

// ============================================================
//  一、输入：视觉 + 惯导
// ============================================================

// 【上游参考，制导不直接消费】目标检测原始结果，字段对齐《输入数据字典.md》。
// 制导假定上游已把原始数据处理好并填进 GuidanceState；
// 本结构仅用于记录上游契约。是否需要一个独立的预处理模块见《待决问题.md》Q1。
struct Detection {
    double t = 0.0;       // 时间戳(s)，统一时钟源
    double u = 0.0;       // 检测框中心像素 x        [必须]
    double v = 0.0;       // 检测框中心像素 y        [必须]
    double w = 0.0;       // 框宽(像素)，测距用      [可选]
    double h = 0.0;       // 框高(像素)，测距用      [可选]
    double conf = 0.0;    // 置信度 0~1              [可选]
    double range = -1.0;  // 直接距离(m)，<0 表示无  [可选]
    int target_id = 0;    // 目标编号                [可选]
};

// 制导输入状态（视觉 + 惯导融合后），字段对齐《输入数据字典.md》
struct GuidanceState {
    double t = 0.0;        // 时间戳(s)
    Vec3   los_hat;        // 视线方向单位矢量 û
    // ⚠ λ̇ **不是制导的输入** —— `guidance.cpp` **从不读取本字段**。
    //   制导核自己算 ω：按 `GuidanceParams::los_rate_source` 二选一
    //     · FROM_RELATIVE_VELOCITY：ω = (r × v_rel)/R²  （仿真，需 v_T）
    //     · FROM_LOS_DIFFERENCE   ：ω = û × (dû/dt)     （真机，只需 û 历史）
    //   算出的 ω 回填到输出 `PureGuidanceCmd::lam_dot`（调试量）。
    //   本字段仅供**上游/日志对照**（例如视觉自己估了一路 λ̇ 想比对）——可选，留零即可。
    double lam_dot = 0.0;  // [可选·调试] 视线角速率 λ̇ (rad/s)；制导不消费
    double range = 0.0;    // 相对距离 R (m)
    bool   valid = false;  // 本帧数据是否有效

    // --- 惯导 ---
    // 姿态四元数，**分量顺序 [w, x, y, z]**（单位四元数 = [1,0,0,0]）。
    // ⚠ 顺序与 PX4 uORB / MAVLink（ATTITUDE_QUATERNION、ODOMETRY.q）一致，
    //   上游（飞控 / 通信）可以原样填入，不需要重排。
    double q[4] = {1.0, 0.0, 0.0, 0.0};
    Vec3   p_M;                          // 自身位置(可选)
    Vec3   v_M;                          // 自身速度
    Vec3   v_T;                          // 目标速度（仅"真值法"算 ω 需要；真机法可留零）
    Vec3   a_T_perp;                     // 目标横向加速度(APN 用；真机需估计)
};

// ============================================================
//  二、输出：制导指令
// ============================================================
//
// ★ 2026-10-20 改名（`GuidanceCmd/GuidanceMode` → `PureGuidanceCmd/PureGuidanceMode`）：
//   同仓库的通信模块（`src/driver/communicate/include/communicator.hpp`、
//   `communication` 分支的 `src/common/msg.hpp`）在**同一个 `pure_uav` 命名空间**里
//   也定义了 `GuidanceCmd` / `GuidanceMode`（内容是"期望位置/速度/加速度"的 setpoint，
//   与本模块的"导引律输出"不是同一个东西）。同名同命名空间 ⇒ 任何 TU 同时 include
//   两者都会报重定义。
//   ⇒ 本模块统一加 **`Pure` 前缀**（与 `libpure_guidance.so` 的命名一致），
//     机载主循环里两者可以并存：
//        · `PureGuidanceCmd`  ← 我们算出来的指令（ENU 加速度 + 调试量）
//        · `GuidanceCmd`      ← 通信模块的 setpoint（NED 位置/速度/加速度）
//     接线方式：`cmd_accel_to_ned()` → `GuidanceCmd{mode=ACCELERATION, afx,afy,afz}`。
//   ⚠ 将来若通信模块也改名，本前缀可以保留（不影响语义，只是多一层保险）。

// 制导模式
enum class PureGuidanceMode {
    PN,      // 比例导引
    APN,     // 增广比例导引
    HOLD,    // 末端冻结（t_go 过小，保持上一帧指令）
    SEARCH   // 目标丢失（输出零指令）
};

// 制导输出，字段对齐《输出数据字典.md》
struct PureGuidanceCmd {
    double t = 0.0;                 // 本指令时间戳(s)
    Vec3   a_cmd;                   // [核心] 期望加速度(m/s²，惯性系)
    double a_cmd_norm = 0.0;        // |a_cmd|
    bool   sat = false;             // 是否触发过载限幅
    bool   valid = false;           // 本周期输入是否有效
    double t_go = 0.0;              // 剩余飞行时间(s)
    PureGuidanceMode mode = PureGuidanceMode::SEARCH;

    // --- 调试量（回放/调参/写报告的证据链）---
    double lam_dot = 0.0;           // 视线角速率
    double Vc = 0.0;                // 接近速度
    double R = 0.0;                 // 相对距离
    Vec3   r_hat;                   // 视线方向单位矢量
    double N = 0.0;                 // 导航比
    Vec3   a_cmd_unsat;             // 限幅前的指令
};

// 制导记忆（跨周期保存的少量状态）。
// 由调用方（仿真主循环 / 机载节点）持有并每周期传回；制导本身无全局状态。
// 目前用于两处：
//   1) 差分法估计 ω 需要上一帧的 û 与时间戳；
//   2) 末端冻结需要上一帧的指令。
struct GuidanceMemory {
    bool   has_prev = false;  // 是否已有上一周期数据
    Vec3   los_hat_prev;      // 上一周期视线方向 û（差分法用）
    double t_prev = 0.0;      // 上一周期时间戳（差分法用）
    Vec3   a_cmd_prev;        // 上一周期指令（末端冻结用）
};

// ============================================================
//  三、参数
// ============================================================

enum class GuidanceLaw {
    PPN,  // 纯比例导引：a = N·(ω × v_M)
    TPN,  // 真比例导引：a = N·Vc·(ω × û)
    APN   // 增广比例导引：a = N·Vc·(ω × û) + (N/2)·a_T⊥
};

// 视线角速率 ω 的来源 —— 两套算法，用 GuidanceParams::los_rate_source 切换。
// 推导见《比例导引讲义.html》第 12 节。
enum class LosRateSource {
    // A）真值法（仿真用）：ω = (r × v_rel) / R²
    //     需要"距离 R + 目标速度 v_T"。精度高，但真机拿不到 v_T / R。
    FROM_RELATIVE_VELOCITY,

    // B）真机法（真机用）：ω = û × (dû/dt)，视线方向时序差分。
    //     只需要 û 的历史（存在 GuidanceMemory）+ 自身速度，不需要 v_T / R。
    //     注意：差分会放大噪声，真机通常需再滤波。
    FROM_LOS_DIFFERENCE
};

// 末端冻结的判据 —— 用 GuidanceParams::freeze_criterion 切换。
//
// 目的：末端 R→0 时，ω=(r×v_rel)/R²（或 û 差分）里的 1/R 会把噪声放大成大指令，
//       故"提前锁定"最后一帧指令（HOLD），防止制导发散。
//
// 两种基准在 Vc 近似常数时**完全等价**：R_freeze ≈ Vc · t_go_freeze。
// 但系统存在交会加速度时二者会分道扬镳（见《比例导引讲义》"加速度的影响"一节）：
//   · BY_TGO  ：以**剩余飞行时间**为基准。可修正量 ∝ ½·a·t_go²，
//               时间才是物理根本量 → 随接近速度自动缩放，作默认。
//   · BY_RANGE：以**剩余距离**为基准。R 可直接测（size 反推/深度），
//               真机无需估计 Vc，实现最简；但固定 R 阈值在 Vc 变化时会过早/过晚冻结。
//   · BY_TGO_OR_RANGE：两者取"或"，用 R 兜底 —— 防 Vc→0（尾追/远离）时
//               t_go = R/max(Vc,ε) 发散导致**永不冻结**。
enum class FreezeCriterion {
    BY_TGO,          // 剩余时间 t_go < t_go_freeze（默认）
    BY_RANGE,        // 剩余距离 R < R_freeze
    BY_TGO_OR_RANGE  // 任一满足即冻结（R 兜底）
};

struct GuidanceParams {
    GuidanceLaw law = GuidanceLaw::PPN; // 默认 PPN（不含距离项，对测距误差免疫）
    double N = 4.0;            // 导航比，工程取 3~5
    double a_max = 30.0;       // 过载上限(m/s²)

    // ↓↓↓ 末端冻结（判据三选一，见 FreezeCriterion）↓↓↓
    FreezeCriterion freeze_criterion = FreezeCriterion::BY_TGO; // 判据基准
    double t_go_freeze = 0.15; // BY_TGO 阈值(s)。**必须与交会速度匹配**：
                               //   Vc≈33 m/s 时取 0.3 s 会在 R≈10 m 就冻结，
                               //   那时 λ̇ 尚未收敛 → 等于提前锁定一条脱靶弹道。
    double R_freeze    = 5.0;  // BY_RANGE 阈值(m)。建议 ≈ Vc · t_go_freeze。

    // ↓↓↓ 两套 ω 算法的切换点 ↓↓↓
    // 仿真阶段用 A（真值，最准）；上真机改成 B（只靠视线方向）。其余代码不动。
    LosRateSource los_rate_source = LosRateSource::FROM_RELATIVE_VELOCITY;
};

} // namespace pure_uav
