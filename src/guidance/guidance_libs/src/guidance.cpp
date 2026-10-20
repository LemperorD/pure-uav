#include "guidance.hpp"

#include "pn_law.hpp"

namespace pure_uav {

GuidanceCmd guidance_step(const GuidanceState &state,
                          const GuidanceParams &params,
                          GuidanceMemory &memory) {
    GuidanceCmd cmd;
    cmd.t     = state.t;
    cmd.valid = state.valid;
    cmd.N     = params.N;
    cmd.r_hat = state.los_hat;
    cmd.R     = state.range;

    // ---- 1) 有效性检查 ----
    // 目标丢失 → SEARCH，输出零指令，并清掉历史（重新捕获时从零开始）
    if (!state.valid) {
        cmd.mode = GuidanceMode::SEARCH;
        memory.has_prev = false;
        return cmd;
    }

    // ---- 2) 计算 ω（视线角速率）—— 两套算法，按 params.los_rate_source 切换 ----
    //   A) FROM_RELATIVE_VELOCITY：ω = (r × v_rel) / R²   （仿真；需要 v_T）
    //   B) FROM_LOS_DIFFERENCE   ：ω = û × (dû/dt)        （真机；只需 û 历史）
    //   切换点就在 GuidanceParams::los_rate_source，其余代码不动。
    //   推导见《比例导引讲义.html》第 12 节。
    const Vec3 r     = state.los_hat * state.range;   // r = û · R
    const Vec3 v_rel = state.v_T - state.v_M;

    Vec3 omega;   // 默认零矢量
    switch (params.los_rate_source) {
        case LosRateSource::FROM_RELATIVE_VELOCITY:
            omega = los_rate_vector(r, v_rel, state.range);
            break;
        case LosRateSource::FROM_LOS_DIFFERENCE:
            if (memory.has_prev) {   // 第一帧没有历史，ω 保持为零
                const double dt = state.t - memory.t_prev;
                omega = los_rate_from_los_difference(state.los_hat,
                                                     memory.los_hat_prev, dt);
            }
            break;
    }

    // 几何量：接近速度 Vc（TPN / APN 需要）
    const double Vc = closing_velocity(r, v_rel, state.range);

    cmd.lam_dot = norm(omega);   // 调试量：便于与真值对照
    cmd.Vc      = Vc;

    // 更新记忆：差分法需要上一帧的 û 与时间戳
    memory.los_hat_prev = state.los_hat;
    memory.t_prev       = state.t;
    memory.has_prev     = true;

    // ---- 3) 选导引律 → 未限幅指令 a_cmd_unsat ----
    // 默认 PPN：a = N · (ω × v_M)，只需要 ω 和自身速度 v_M。
    //   PPN 不含距离项，对测距误差免疫 —— 这正是无人机选它的原因。
    switch (params.law) {
        case GuidanceLaw::PPN:
            cmd.a_cmd_unsat = pn_ppn(omega, state.v_M, params.N);
            cmd.mode = GuidanceMode::PN;
            break;
        case GuidanceLaw::TPN:
            cmd.a_cmd_unsat = pn_tpn(omega, state.los_hat, Vc, params.N);
            cmd.mode = GuidanceMode::PN;
            break;
        case GuidanceLaw::APN:
            cmd.a_cmd_unsat = pn_apn(omega, state.los_hat, Vc, params.N, state.a_T_perp);
            cmd.mode = GuidanceMode::APN;
            break;
    }

    // ---- 4) 末端冻结 ----
    // t_go / R 很小时，1/R 会把 λ̇ 估计噪声放大成大指令 → 冻结为上一周期指令，防发散。
    // 判据三选一（见 FreezeCriterion）：
    //   默认 BY_TGO 以剩余时间 t_go 为基准（物理量＝时间，随接近速度自动缩放）；
    //   BY_RANGE 以剩余距离 R 为基准（真机可直接测 R，不需估计 Vc）；
    //   BY_TGO_OR_RANGE 两者取"或"，用 R 兜底（防 Vc→0 时 t_go 发散）。
    //   同 Vc 下三者等价：R_freeze ≈ Vc · t_go_freeze。
    cmd.t_go  = state.range / (Vc > 1e-6 ? Vc : 1e-6);
    cmd.a_cmd = cmd.a_cmd_unsat;

    bool freeze = false;
    switch (params.freeze_criterion) {
        case FreezeCriterion::BY_TGO:
            freeze = (cmd.t_go < params.t_go_freeze);
            break;
        case FreezeCriterion::BY_RANGE:
            freeze = (state.range < params.R_freeze);
            break;
        case FreezeCriterion::BY_TGO_OR_RANGE:
            freeze = (cmd.t_go < params.t_go_freeze) ||
                     (state.range < params.R_freeze);
            break;
    }
    if (freeze) {
        cmd.mode  = GuidanceMode::HOLD;
        cmd.a_cmd = memory.a_cmd_prev;
    }

    // ---- 5) 过载限幅（矢量等比缩放；禁止对 x/y/z 分量各自 clip）----
    const double a_norm = norm(cmd.a_cmd);
    if (a_norm > params.a_max) {
        cmd.a_cmd = cmd.a_cmd * (params.a_max / a_norm);   // 方向不变，只缩模长
        cmd.sat = true;
    }
    // 限幅后的模长只可能是 a_max 或原值 ⇒ 直接复用已算出的 a_norm，省一次 sqrt。
    // （原写法 norm(cmd.a_cmd) 会重复开方；两者差值在 1 ulp 量级，远小于 kCmdTol=1e-9）
    cmd.a_cmd_norm = cmd.sat ? params.a_max : a_norm;

    // 保存本周期指令，供下一周期末端冻结使用
    memory.a_cmd_prev = cmd.a_cmd;

    return cmd;
}

} // namespace pure_uav
