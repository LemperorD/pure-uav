#include "guidance_state_check.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "vec3.hpp"

namespace pure_uav::guidance::adapter {

// ============================================================================
//  §0  GuidanceReport
// ============================================================================

std::string GuidanceReport::summary() const {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%d 项通过 / %d 项告警 / %d 项错误", n_ok, n_warn, n_error);
    return std::string(buf);
}

std::string GuidanceReport::detail() const {
    if (n_issues == 0) {
        std::string s = "全部通过：";
        s += summary();
        return s;
    }
    std::string s;
    for (int i = 0; i < n_issues; ++i) {
        const GuidanceIssue& is = issues[i];
        s += (is.severity == GuidanceSeverity::kError) ? "[错误] " : "[告警] ";
        s += is.field;
        s += " —— ";
        s += is.detail;
        s += '\n';
    }
    s += summary();
    return s;
}

bool GuidanceReport::has(GuidanceCheckCode code) const noexcept {
    for (int i = 0; i < n_issues; ++i) {
        if (issues[i].code == code && issues[i].severity == GuidanceSeverity::kError) {
            return true;
        }
    }
    return false;
}

void add_check(GuidanceReport& rep, bool ok_cond, GuidanceSeverity sev_if_fail, GuidanceCheckCode code, const char* field, const char* detail) noexcept {
    if (ok_cond) {
        ++rep.n_ok;
        return;
    }
    if (sev_if_fail == GuidanceSeverity::kWarn) {
        ++rep.n_warn;
    } else {
        ++rep.n_error;
    }
    if (rep.n_issues < static_cast<int>(sizeof(rep.issues) / sizeof(rep.issues[0]))) {
        GuidanceIssue& is = rep.issues[rep.n_issues++];
        is.severity = sev_if_fail;
        is.code = code;
        is.field = field;
        is.detail = detail;
    }
}

// ============================================================================
//  内部工具
// ============================================================================
namespace {

std::string fmt(const char* label, double v) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s = %.12g", label, v);
    return std::string(buf);
}

inline bool finite3(const Vec3& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool finite4(const double q[4]) noexcept {
    return std::isfinite(q[0]) && std::isfinite(q[1]) && std::isfinite(q[2]) && std::isfinite(q[3]);
}

inline bool mode_valid(PureGuidanceMode m) noexcept {
    switch (m) {
        case PureGuidanceMode::PN:
        case PureGuidanceMode::APN:
        case PureGuidanceMode::HOLD:
        case PureGuidanceMode::SEARCH:
            return true;
    }
    return false;
}

// 姿态四元数模长（非有限时返回 NaN，调用方自行判）
inline double quat_norm(const double q[4]) noexcept {
    return std::sqrt((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
}

}  // namespace

// ============================================================================
//  §1  GuidanceState
// ============================================================================
//
// 注意：所有检查都用 add_check_lazy：detail 只在**失败时**才生成。
//   完整版一次约 960 ns，其中十几次 std::string 构造占绝大部分，而它们在通过路径上
//   全是白造的。换成惰性后通过路径零分配（见 guidance_state_check.hpp 的说明）。

GuidanceReport check_guidance_state(const GuidanceState& st, bool expected_los_units) {
    GuidanceReport rep;

    // ---- 通用有限性（不是不变量的话后面所有比较都没意义）----
    add_check_lazy(rep, std::isfinite(st.t), GuidanceSeverity::kError, GuidanceCheckCode::kNotFinite, "t", [&] { return fmt("时间戳非有限，t", st.t); });
    add_check_lazy(rep, finite3(st.los_hat), GuidanceSeverity::kError, GuidanceCheckCode::kNotFinite, "los_hat", [] { return std::string("视线方向出现 NaN / Inf"); });
    add_check_lazy(rep, std::isfinite(st.lam_dot), GuidanceSeverity::kError, GuidanceCheckCode::kNotFinite, "lam_dot", [] { return std::string("λ̇ 出现 NaN / Inf"); });
    add_check_lazy(rep, std::isfinite(st.range), GuidanceSeverity::kError, GuidanceCheckCode::kNotFinite, "range", [] { return std::string("距离出现 NaN / Inf"); });
    add_check_lazy(rep, finite3(st.p_M), GuidanceSeverity::kWarn, GuidanceCheckCode::kNotFinite, "p_M", [] { return std::string("自身位置出现 NaN / Inf"); });
    add_check_lazy(rep, finite3(st.v_M), GuidanceSeverity::kError, GuidanceCheckCode::kNotFinite, "v_M", [] { return std::string("自身速度出现 NaN / Inf"); });
    add_check_lazy(rep, finite3(st.v_T), GuidanceSeverity::kWarn, GuidanceCheckCode::kNotFinite, "v_T", [] { return std::string("目标速度出现 NaN / Inf"); });
    add_check_lazy(rep, finite3(st.a_T_perp), GuidanceSeverity::kWarn, GuidanceCheckCode::kNotFinite, "a_T_perp", [] { return std::string("目标加速度出现 NaN / Inf"); });
    add_check_lazy(rep, finite4(st.q), GuidanceSeverity::kError, GuidanceCheckCode::kNotFinite, "q", [] { return std::string("四元数出现 NaN / Inf"); });

    // ---- 姿态：|q| = 1 ----
    {
        const double n = quat_norm(st.q);
        if (std::isfinite(n)) {
            add_check_lazy(rep, std::fabs(n - 1.0) <= kQuatNormTol, GuidanceSeverity::kError, GuidanceCheckCode::kQuatNotUnit, "q", [n] { return fmt("|q| 偏离 1（上游可能没归一化）", n); });
        }
    }

    // ---- 视线：|los_hat| = 1（契约）或仅"非零" ----
    {
        const double ln = norm(st.los_hat);
        add_check_lazy(rep, ln > kRangeZeroTol, GuidanceSeverity::kError, GuidanceCheckCode::kLosZero, "los_hat", [] { return std::string("视线方向为零矢量，无方向可言"); });
        if (expected_los_units) {
            add_check_lazy(rep, std::fabs(ln - 1.0) <= kLosNormTol, GuidanceSeverity::kError, GuidanceCheckCode::kLosNotUnit, "los_hat", [ln] { return fmt("|los_hat| 偏离 1（上游没单位化）", ln); });
        }
    }

    // ---- 距离：range ≥ 0 ----
    add_check_lazy(rep, st.range >= 0.0, GuidanceSeverity::kError, GuidanceCheckCode::kRangeNegative, "range", [&] { return fmt("距离为负", st.range); });
    add_check_lazy(rep, st.range > kRangeZeroTol, GuidanceSeverity::kWarn, GuidanceCheckCode::kRangeZero, "range", [] { return std::string("距离≈0：ω 的 R² 分母会放大噪声（末端应已冻结）"); });

    // ---- 时间：不许为负 ----
    // 注意：只查"负"是能查的；"倒退"和"跳变"是跨帧性质，本函数查不了
    //   —— 序列层面的单调性由 GuidanceStateTimeTracker 负责（out_of_order()）。
    add_check_lazy(rep, st.t >= 0.0, GuidanceSeverity::kWarn, GuidanceCheckCode::kTimeNegative, "t", [&] { return fmt("时间为负（可能是跨源相减或回绕未处理）", st.t); });

    return rep;
}

// ============================================================================
//  §1.5  廉价体检（飞行回路专用，零分配）
// ============================================================================

bool quick_check(const GuidanceState& st, GuidanceCheckCode* first_bad) noexcept {
    const auto fail = [first_bad](GuidanceCheckCode code) noexcept {
        if (first_bad) {
            *first_bad = code;
        }
        return false;
    };

    // ① 有限性（与完整版的 error 级一致）
    if (!std::isfinite(st.t) || !std::isfinite(st.lam_dot) || !std::isfinite(st.range)) {
        return fail(GuidanceCheckCode::kNotFinite);
    }
    if (!finite3(st.los_hat) || !finite3(st.v_M) || !finite4(st.q)) {
        return fail(GuidanceCheckCode::kNotFinite);
    }

    // ② 视线：非零 + 单位化
    const double ln = norm(st.los_hat);
    if (!(ln > kRangeZeroTol)) {
        return fail(GuidanceCheckCode::kLosZero);  // 同时挡住 NaN
    }
    if (!(std::fabs(ln - 1.0) <= kLosNormTol)) {
        return fail(GuidanceCheckCode::kLosNotUnit);
    }

    // ③ 姿态：单位四元数
    const double qn = quat_norm(st.q);
    if (!(std::fabs(qn - 1.0) <= kQuatNormTol)) {
        return fail(GuidanceCheckCode::kQuatNotUnit);
    }

    // ④ 距离非负（range≈0 只是 warning，交给完整版/末端冻结）
    if (!(st.range >= 0.0)) {
        return fail(GuidanceCheckCode::kRangeNegative);
    }

    return true;
}

// ============================================================================
//  §2  PureGuidanceCmd
// ============================================================================

GuidanceReport check_guidance_cmd(const PureGuidanceCmd& cmd, double a_max_mps2, bool has_a_max, double expect_t, bool has_expect_t) {
    GuidanceReport rep;

    add_check_lazy(rep, std::isfinite(cmd.t), GuidanceSeverity::kError, GuidanceCheckCode::kNotFinite, "cmd.t", [&] { return fmt("指令时间戳非有限", cmd.t); });
    add_check_lazy(rep, finite3(cmd.a_cmd), GuidanceSeverity::kError, GuidanceCheckCode::kNotFinite, "a_cmd", [] { return std::string("指令加速度出现 NaN / Inf"); });
    add_check_lazy(rep, finite3(cmd.a_cmd_unsat), GuidanceSeverity::kWarn, GuidanceCheckCode::kNotFinite, "a_cmd_unsat", [] { return std::string("限幅前指令出现 NaN / Inf"); });
    add_check_lazy(rep, finite3(cmd.r_hat), GuidanceSeverity::kWarn, GuidanceCheckCode::kNotFinite, "r_hat", [] { return std::string("视线方向出现 NaN / Inf"); });
    add_check_lazy(rep, std::isfinite(cmd.a_cmd_norm), GuidanceSeverity::kError, GuidanceCheckCode::kNotFinite, "a_cmd_norm", [] { return std::string("|a_cmd| 非有限"); });
    add_check_lazy(rep, std::isfinite(cmd.t_go), GuidanceSeverity::kWarn, GuidanceCheckCode::kNotFinite, "t_go", [] { return std::string("t_go 非有限"); });

    // ---- |a_cmd| 与 a_cmd_norm 必须一致 ----
    {
        const double an = norm(cmd.a_cmd);
        add_check_lazy(rep, std::fabs(an - cmd.a_cmd_norm) <= kCmdTol, GuidanceSeverity::kError, GuidanceCheckCode::kCmdNormMismatch, "a_cmd_norm", [an, &cmd] { return fmt("|a_cmd| 与 a_cmd_norm 不一致", an - cmd.a_cmd_norm); });
    }

    // ---- 限幅自洽（给了 a_max 才查）----
    if (has_a_max && std::isfinite(a_max_mps2) && a_max_mps2 > 0.0) {
        const double an = norm(cmd.a_cmd);
        const double un = norm(cmd.a_cmd_unsat);

        add_check_lazy(rep, an <= a_max_mps2 + kCmdTol, GuidanceSeverity::kError, GuidanceCheckCode::kCmdOverLimit, "a_cmd", [an] { return fmt("限幅后 |a_cmd| 仍超过 a_max", an); });

        if (cmd.sat) {
            // sat=true ⇒ 限幅前必须真的超限（HOLD 模式会覆盖 a_cmd，故排除）
            if (cmd.mode != PureGuidanceMode::HOLD) {
                add_check_lazy(rep, un > a_max_mps2, GuidanceSeverity::kError, GuidanceCheckCode::kCmdUnsatBelowMax, "a_cmd_unsat", [un] { return fmt("sat=true 但限幅前未超过 a_max", un); });
                add_check_lazy(rep, std::fabs(an - a_max_mps2) <= kCmdTol, GuidanceSeverity::kWarn, GuidanceCheckCode::kCmdSatInconsistent, "a_cmd", [an] { return fmt("sat=true 时限幅后应恰好等于 a_max", an); });
            }
        } else if (cmd.mode != PureGuidanceMode::HOLD) {
            // sat=false ⇒ 限幅前也不该超限
            add_check_lazy(rep, un <= a_max_mps2 + kCmdTol, GuidanceSeverity::kWarn, GuidanceCheckCode::kCmdSatInconsistent, "a_cmd_unsat", [un] { return fmt("sat=false 但限幅前已超过 a_max", un); });
        }
    }

    // ---- mode 合法性 / SEARCH 契约 ----
    add_check_lazy(rep, mode_valid(cmd.mode), GuidanceSeverity::kError, GuidanceCheckCode::kCmdModeInvalid, "mode", [] { return std::string("mode 不是 PN / APN / HOLD / SEARCH 之一"); });
    if (cmd.mode == PureGuidanceMode::SEARCH) {
        add_check_lazy(rep, norm(cmd.a_cmd) <= kCmdTol, GuidanceSeverity::kError, GuidanceCheckCode::kCmdNormMismatch, "a_cmd", [] { return std::string("SEARCH 模式必须输出零指令"); });
        add_check_lazy(rep, !cmd.valid, GuidanceSeverity::kWarn, GuidanceCheckCode::kCmdModeInvalid, "valid", [] { return std::string("SEARCH 模式但 valid=true（状态自相矛盾）"); });
    }

    // ---- cmd.t 与 state.t 一致 ----
    if (has_expect_t) {
        add_check_lazy(rep, std::fabs(cmd.t - expect_t) <= kCmdTol, GuidanceSeverity::kWarn, GuidanceCheckCode::kCmdTimeMismatch, "cmd.t", [&cmd, expect_t] { return fmt("cmd.t 与 state.t 之差不一致", cmd.t - expect_t); });
    }

    return rep;
}

// ============================================================================
//  §3  合体入口
// ============================================================================

GuidanceReport check_frame(const GuidanceState& st, const PureGuidanceCmd& cmd, double a_max_mps2, bool has_a_max) {
    GuidanceReport rep = check_guidance_state(st);
    GuidanceReport rc = check_guidance_cmd(cmd, a_max_mps2, has_a_max, st.t, true);
    rep.n_ok += rc.n_ok;
    rep.n_warn += rc.n_warn;
    rep.n_error += rc.n_error;
    for (int i = 0; i < rc.n_issues; ++i) {
        if (rep.n_issues < static_cast<int>(sizeof(rep.issues) / sizeof(rep.issues[0]))) {
            rep.issues[rep.n_issues++] = rc.issues[i];
        }
    }
    return rep;
}

bool invariants_hold(const GuidanceState& st, const PureGuidanceCmd& cmd, double a_max_mps2, bool has_a_max) {
    return check_frame(st, cmd, a_max_mps2, has_a_max).ok();
}

}  // namespace pure_uav::guidance::adapter
