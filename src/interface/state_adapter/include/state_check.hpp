#pragma once
// =============================================================================
//  输入自检：不变量（invariant）检查
//
//  【存在理由】"数据对齐"的事故几乎都是**静默**的：
//    · 四元数顺序反了 → 不报错，只是姿态悄悄变错；
//    · NED 当成 ENU 用 → 不报错，只是高度正负颠倒；
//    · 上游给了未归一化的四元数 → 不报错，只是姿态有缩放；
//    · 时间戳跨源相减 → 不报错，只是 dt 变成垃圾，λ̇ 差分爆炸。
//  所以必须有一条"收到一帧就体检"的路径，把静默错误变成**可见的告警**。
//
//  【用法 1：单测里钉不变量】《数据对齐-自决项详解.md》④
//      ok(invariants_hold(st), "…") —— 制导契约不许被后续改动破坏。
//
//  【用法 2：联调现场自检】收到对方一帧数据先跑一遍：
//      auto rep = check_guidance_state(st);
//      if (!rep.ok()) printf("%s\n", rep.summary().c_str());
//    ⇒ 能直接指出"是对方口径问题"还是"我们的问题"，把扯皮变成用数据说话。
//
//  【注意】本模块**只报告**，不修改数据、不做修正（修正是适配层的职责）。
//    这也意味着它可以用在"怀疑对方数据有问题"的场合而不会掩盖问题。
// =============================================================================

#include <string>

#include "guidance_types.hpp"

namespace pure_uav {
namespace adapter {

// ============================================================================
//  §0  结果类型
// ============================================================================

enum class Severity {
    kOk = 0,     // 满足不变量
    kWarn = 1,   // 可疑但物理上可能（例如 |q| 因外推略偏离 1）
    kError = 2,  // 违反不变量 —— 拿来进制导会出错
};

enum class CheckCode : int {
    kOk = 0,

    // --- 通用 ---
    kNotFinite,          // 出现 NaN / ±Inf
    kNotFilled,          // 关键字段仍为"未填充"状态（*_set 标志为 false）

    // --- 姿态 q ---
    kQuatNotUnit,        // |q| 偏离 1 超过容差

    // --- 视线 / 距离 ---
    kLosNotUnit,         // |los_hat| 偏离 1 超过容差（**上游没单位化**，最常见的脏数据）
    kLosZero,            // |los_hat| ≈ 0（无方向可言）
    kRangeNegative,      // range < 0
    kRangeZero,          // range ≈ 0（会导致 ω 分母 R² → 0）

    // --- 时间 ---
    kTimeNegative,       // 时间为负（可能"跨源相减"或时间戳回绕没处理）
    kTimeBackwards,      // 时间倒退
    kTimeJumpLarge,      // 单帧时间跳变过大（丢帧？时钟跳变？）

    // --- 指令 ---
    kCmdNormMismatch,    // |a_cmd| 与 a_cmd_norm 不一致
    kCmdSatInconsistent, // sat 标志与"是否真的超限"不一致
    kCmdOverLimit,       // 限幅后 |a_cmd| 仍超过 a_max
    kCmdUnsatBelowMax,   // sat=true 但限幅前就不超限（不该标记饱和）
    kCmdModeInvalid,     // mode 不是四个合法值之一
    kCmdTimeMismatch,    // cmd.t 与 state.t 不一致
};

// 单条检查结果
struct Issue {
    Severity   severity = Severity::kOk;
    CheckCode  code = CheckCode::kOk;
    std::string field;    // 字段名 / 量名
    std::string detail;   // 人类可读说明（含实测数）
};

// 一次体检的报告
struct Report {
    int n_ok = 0;
    int n_warn = 0;
    int n_error = 0;
    Issue issues[16];
    int n_issues = 0;

    // 有没有 error（warning 不算不通过：联调时它是提示，不是阻塞）
    bool ok() const noexcept { return n_error == 0; }

    // 一行摘要，例如："13 项通过 / 1 项告警 / 1 项错误"
    std::string summary() const;

    // 多行明细（每条 issue 一行；无 issue 时给"全部通过"）
    std::string detail() const;

    // 供测试/日志：是否存在某个 code 的 error
    bool has(CheckCode code) const noexcept;
};

// 把一次检查追加进 report（满足不变量则只记 n_ok）
void add_check(Report &rep, bool ok_cond, Severity sev_if_fail,
               CheckCode code, const char *field, const char *detail) noexcept;

// ============================================================================
//  §1  检查阈值（显式列出，便于按实测放宽/收紧）
// ============================================================================

inline constexpr double kQuatNormTol   = 1e-6;   // |q| 与 1 的最大允许偏差
                                                 // 外推/插值未归一化时会明显超过
inline constexpr double kLosNormTol    = 1e-6;   // |los_hat| 与 1 的最大允许偏差
inline constexpr double kRangeZeroTol  = 1e-9;   // range 视作 0 的阈值
inline constexpr double kTimeJumpMaxS  = 5.0;    // 单帧时间跳变上限(s)；超过多半是丢帧/时钟跳变
inline constexpr double kCmdTol        = 1e-9;   // 指令量的比较容差(m/s²)

// ============================================================================
//  §2  检查入口
// ============================================================================

// 体检一个 GuidanceState。
// `expected_los_units`：los_hat 是否为**单位矢量**。制导契约要求是单位矢量；
//   若你的上游给的是"未归一化方向"且打算让制导自己归一化，传 false 只查"非零"。
// `has_a_max` / `a_max_mps2`：给了才查"限幅后不得超限"。
Report check_guidance_state(const GuidanceState &st,
                            bool expected_los_units = true);

// 体检一个 GuidanceCmd（t 期望 == `expect_t`，一般传 state.t）。
Report check_guidance_cmd(const GuidanceCmd &cmd,
                          double a_max_mps2 = 0.0,
                          bool has_a_max = false,
                          double expect_t = 0.0,
                          bool has_expect_t = false);

// 一次性体检 state + cmd（联调现场最常用）。
Report check_frame(const GuidanceState &st, const GuidanceCmd &cmd,
                   double a_max_mps2 = 0.0, bool has_a_max = false);

// ============================================================================
//  §3  不变量总闸（单测用）
// ============================================================================

// 制导契约的**核心不变量**是否全部成立（无 error 即成立）：
//   · |q| = 1                · |los_hat| = 1            · range ≥ 0
//   · 所有量有限             · cmd 限幅自洽（给了 a_max 时）
// 与 check_frame 的区别：本函数**忽略 warning**，只看硬不变量。
bool invariants_hold(const GuidanceState &st, const GuidanceCmd &cmd,
                     double a_max_mps2 = 0.0, bool has_a_max = false);

}  // namespace adapter
}  // namespace pure_uav
