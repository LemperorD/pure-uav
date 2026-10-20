#include "state_check.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "vec3.hpp"

namespace pure_uav {
namespace adapter {

// ============================================================================
//  §0  Report
// ============================================================================

std::string Report::summary() const {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%d 项通过 / %d 项告警 / %d 项错误",
                  n_ok, n_warn, n_error);
    return std::string(buf);
}

std::string Report::detail() const {
    if (n_issues == 0) {
        std::string s = "全部通过：";
        s += summary();
        return s;
    }
    std::string s;
    for (int i = 0; i < n_issues; ++i) {
        const Issue &is = issues[i];
        s += (is.severity == Severity::kError) ? "[错误] " : "[告警] ";
        s += is.field;
        s += " —— ";
        s += is.detail;
        s += '\n';
    }
    s += summary();
    return s;
}

bool Report::has(CheckCode code) const noexcept {
    for (int i = 0; i < n_issues; ++i) {
        if (issues[i].code == code && issues[i].severity == Severity::kError) return true;
    }
    return false;
}

void add_check(Report &rep, bool ok_cond, Severity sev_if_fail,
               CheckCode code, const char *field, const char *detail) noexcept {
    if (ok_cond) {
        ++rep.n_ok;
        return;
    }
    if (sev_if_fail == Severity::kWarn) {
        ++rep.n_warn;
    } else {
        ++rep.n_error;
    }
    if (rep.n_issues < static_cast<int>(sizeof(rep.issues) / sizeof(rep.issues[0]))) {
        Issue &is = rep.issues[rep.n_issues++];
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

std::string fmt(const char *label, double v) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s = %.12g", label, v);
    return std::string(buf);
}

inline bool finite3(const Vec3 &v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool finite4(const double q[4]) noexcept {
    return std::isfinite(q[0]) && std::isfinite(q[1]) &&
           std::isfinite(q[2]) && std::isfinite(q[3]);
}

inline bool mode_valid(GuidanceMode m) noexcept {
    switch (m) {
        case GuidanceMode::PN:
        case GuidanceMode::APN:
        case GuidanceMode::HOLD:
        case GuidanceMode::SEARCH:
            return true;
    }
    return false;
}

}  // namespace

// ============================================================================
//  §1  GuidanceState
// ============================================================================

Report check_guidance_state(const GuidanceState &st, bool expected_los_units) {
    Report rep;

    // ---- 通用有限性（不是不变量的话后面所有比较都没意义）----
    add_check(rep, std::isfinite(st.t), Severity::kError, CheckCode::kNotFinite,
              "t", fmt("时间戳非有限，t", st.t).c_str());
    add_check(rep, finite3(st.los_hat), Severity::kError, CheckCode::kNotFinite,
              "los_hat", "视线方向出现 NaN / Inf");
    add_check(rep, std::isfinite(st.lam_dot), Severity::kError, CheckCode::kNotFinite,
              "lam_dot", "λ̇ 出现 NaN / Inf");
    add_check(rep, std::isfinite(st.range), Severity::kError, CheckCode::kNotFinite,
              "range", "距离出现 NaN / Inf");
    add_check(rep, finite3(st.p_M), Severity::kWarn, CheckCode::kNotFinite,
              "p_M", "自身位置出现 NaN / Inf");
    add_check(rep, finite3(st.v_M), Severity::kError, CheckCode::kNotFinite,
              "v_M", "自身速度出现 NaN / Inf");
    add_check(rep, finite3(st.v_T), Severity::kWarn, CheckCode::kNotFinite,
              "v_T", "目标速度出现 NaN / Inf");
    add_check(rep, finite3(st.a_T_perp), Severity::kWarn, CheckCode::kNotFinite,
              "a_T_perp", "目标加速度出现 NaN / Inf");
    add_check(rep, finite4(st.q), Severity::kError, CheckCode::kNotFinite,
              "q", "四元数出现 NaN / Inf");

    // ---- 姿态：|q| = 1 ----
    {
        const double n2 = st.q[0] * st.q[0] + st.q[1] * st.q[1] +
                          st.q[2] * st.q[2] + st.q[3] * st.q[3];
        if (std::isfinite(n2)) {
            const double n = std::sqrt(n2);
            add_check(rep, std::fabs(n - 1.0) <= kQuatNormTol, Severity::kError,
                      CheckCode::kQuatNotUnit, "q",
                      fmt("|q| 偏离 1（上游可能没归一化）", n).c_str());
        }
    }

    // ---- 视线：|los_hat| = 1（契约）或仅"非零" ----
    {
        const double ln = norm(st.los_hat);
        add_check(rep, ln > kRangeZeroTol, Severity::kError, CheckCode::kLosZero,
                  "los_hat", "视线方向为零矢量，无方向可言");
        if (expected_los_units) {
            add_check(rep, std::fabs(ln - 1.0) <= kLosNormTol, Severity::kError,
                      CheckCode::kLosNotUnit, "los_hat",
                      fmt("|los_hat| 偏离 1（上游没单位化）", ln).c_str());
        }
    }

    // ---- 距离：range ≥ 0 ----
    add_check(rep, st.range >= 0.0, Severity::kError, CheckCode::kRangeNegative,
              "range", fmt("距离为负", st.range).c_str());
    add_check(rep, st.range > kRangeZeroTol, Severity::kWarn, CheckCode::kRangeZero,
              "range", "距离≈0：ω 的 R² 分母会放大噪声（末端应已冻结）");

    // ---- 时间：不许为负 ----
    // ⚠ 只查"负"是能查的；"倒退"和"跳变"是跨帧性质，本函数查不了
    //   —— 序列层面的单调性由 StateTimeTracker 负责（out_of_order()）。
    add_check(rep, st.t >= 0.0, Severity::kWarn, CheckCode::kTimeNegative,
              "t", fmt("时间为负（可能是跨源相减或回绕未处理）", st.t).c_str());

    return rep;
}

// ============================================================================
//  §2  GuidanceCmd
// ============================================================================

Report check_guidance_cmd(const GuidanceCmd &cmd, double a_max_mps2,
                          bool has_a_max, double expect_t, bool has_expect_t) {
    Report rep;

    add_check(rep, std::isfinite(cmd.t), Severity::kError, CheckCode::kNotFinite,
              "cmd.t", fmt("指令时间戳非有限", cmd.t).c_str());
    add_check(rep, finite3(cmd.a_cmd), Severity::kError, CheckCode::kNotFinite,
              "a_cmd", "指令加速度出现 NaN / Inf");
    add_check(rep, finite3(cmd.a_cmd_unsat), Severity::kWarn, CheckCode::kNotFinite,
              "a_cmd_unsat", "限幅前指令出现 NaN / Inf");
    add_check(rep, finite3(cmd.r_hat), Severity::kWarn, CheckCode::kNotFinite,
              "r_hat", "视线方向出现 NaN / Inf");
    add_check(rep, std::isfinite(cmd.a_cmd_norm), Severity::kError,
              CheckCode::kNotFinite, "a_cmd_norm", "|a_cmd| 非有限");
    add_check(rep, std::isfinite(cmd.t_go), Severity::kWarn, CheckCode::kNotFinite,
              "t_go", "t_go 非有限");

    // ---- |a_cmd| 与 a_cmd_norm 必须一致 ----
    {
        const double an = norm(cmd.a_cmd);
        add_check(rep, std::fabs(an - cmd.a_cmd_norm) <= kCmdTol,
                  Severity::kError, CheckCode::kCmdNormMismatch, "a_cmd_norm",
                  fmt("|a_cmd| 与 a_cmd_norm 不一致", an - cmd.a_cmd_norm).c_str());
    }

    // ---- 限幅自洽（给了 a_max 才查）----
    if (has_a_max && std::isfinite(a_max_mps2) && a_max_mps2 > 0.0) {
        const double an = norm(cmd.a_cmd);
        const double un = norm(cmd.a_cmd_unsat);

        add_check(rep, an <= a_max_mps2 + kCmdTol, Severity::kError,
                  CheckCode::kCmdOverLimit, "a_cmd",
                  fmt("限幅后 |a_cmd| 仍超过 a_max", an).c_str());

        if (cmd.sat) {
            // sat=true ⇒ 限幅前必须真的超限（HOLD 模式会覆盖 a_cmd，故排除）
            if (cmd.mode != GuidanceMode::HOLD) {
                add_check(rep, un > a_max_mps2, Severity::kError,
                          CheckCode::kCmdUnsatBelowMax, "a_cmd_unsat",
                          fmt("sat=true 但限幅前未超过 a_max", un).c_str());
                add_check(rep, std::fabs(an - a_max_mps2) <= kCmdTol,
                          Severity::kWarn, CheckCode::kCmdSatInconsistent, "a_cmd",
                          fmt("sat=true 时限幅后应恰好等于 a_max", an).c_str());
            }
        } else if (cmd.mode != GuidanceMode::HOLD) {
            // sat=false ⇒ 限幅前也不该超限
            add_check(rep, un <= a_max_mps2 + kCmdTol, Severity::kWarn,
                      CheckCode::kCmdSatInconsistent, "a_cmd_unsat",
                      fmt("sat=false 但限幅前已超过 a_max", un).c_str());
        }
    }

    // ---- mode 合法性 / SEARCH 契约 ----
    add_check(rep, mode_valid(cmd.mode), Severity::kError, CheckCode::kCmdModeInvalid,
              "mode", "mode 不是 PN / APN / HOLD / SEARCH 之一");
    if (cmd.mode == GuidanceMode::SEARCH) {
        add_check(rep, norm(cmd.a_cmd) <= kCmdTol, Severity::kError,
                  CheckCode::kCmdNormMismatch, "a_cmd",
                  "SEARCH 模式必须输出零指令");
        add_check(rep, !cmd.valid, Severity::kWarn, CheckCode::kCmdModeInvalid,
                  "valid", "SEARCH 模式但 valid=true（状态自相矛盾）");
    }

    // ---- cmd.t 与 state.t 一致 ----
    if (has_expect_t) {
        add_check(rep, std::fabs(cmd.t - expect_t) <= kCmdTol, Severity::kWarn,
                  CheckCode::kCmdTimeMismatch, "cmd.t",
                  fmt("cmd.t 与 state.t 之差不一致", cmd.t - expect_t).c_str());
    }

    return rep;
}

// ============================================================================
//  §3  合体入口
// ============================================================================

Report check_frame(const GuidanceState &st, const GuidanceCmd &cmd,
                   double a_max_mps2, bool has_a_max) {
    Report rep = check_guidance_state(st);
    Report rc = check_guidance_cmd(cmd, a_max_mps2, has_a_max,
                                   st.t, true);
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

bool invariants_hold(const GuidanceState &st, const GuidanceCmd &cmd,
                     double a_max_mps2, bool has_a_max) {
    return check_frame(st, cmd, a_max_mps2, has_a_max).ok();
}

}  // namespace adapter
}  // namespace pure_uav
