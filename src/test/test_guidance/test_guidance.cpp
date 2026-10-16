// 制导模块单元测试 —— 王乙芃
//
// 风格：无外部测试框架，纯 main() + 轻量断言（与仓库 test_math 一致，但会真正判定成败）。
// 覆盖：
//   1. Vec3 基础运算
//   2. 讲义 §10 手算例（R / Vc / λ̇ / |a| / t_go）—— 与讲义数值逐项对照
//   3. ω 两套算法一致性（真值法 vs 视线差分法）
//   4. 接近速度 Vc 符号
//   5. guidance_step 端到端（TPN / PPN / APN + APN 补偿恒等式）
//   6. 三条铁律：① ω 分母 R² ② 矢量等比限幅 ③ 末端冻结 HOLD；外加 SEARCH
//   7. ω 来源切换（FROM_LOS_DIFFERENCE 在 guidance_step 中生效）
//   8~12. 传感器墙（投影 / 四元数 / 逼真度 / 丢帧 / 延迟）
//   13. 目标模型（M2）：发射参数 / 解析式 / DART(Cd=0)≡BALLISTIC 交叉验证 /
//       气动阻力 / CIRCLE / SNAKE / 查表连续性
//   14. 末端冻结判据：t_go 基准 ≡ R 基准（同 Vc）/ R 兜底防 Vc→0
//
// 运行：./bin/test/test_guidance   （退出码 0=全过，1=有失败）

#include <cmath>
#include <cstdio>

#include "vec3.hpp"
#include "guidance_types.hpp"
#include "pn_law.hpp"
#include "guidance.hpp"
#include "sensor.hpp"
#include "target.hpp"

using namespace pure_uav;

// ============================================================
//  轻量断言
// ============================================================
static int g_pass = 0;
static int g_fail = 0;

static void ok(bool cond, const char *what) {
    if (cond) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("  [FAIL] %s\n", what);
    }
}

static void near(double got, double want, double tol, const char *what) {
    if (std::fabs(got - want) <= tol) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("  [FAIL] %s: got %.8g, want %.8g (tol %.2g)\n",
                    what, got, want, tol);
    }
}

static void near_vec(const Vec3 &got, const Vec3 &want, double tol, const char *what) {
    if (norm(got - want) <= tol) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("  [FAIL] %s: got (%.6g, %.6g, %.6g), want (%.6g, %.6g, %.6g)\n",
                    what, got.x, got.y, got.z, want.x, want.y, want.z);
    }
}

static void section(const char *name) { std::printf("\n== %s ==\n", name); }

// ============================================================
//  讲义 §10 手算例（2D 嵌入 3D，z=0）
//    M(0,0) v=(18,0)   T(300,40) v=(-25,0)
// ============================================================
static const Vec3 P_M(0, 0, 0);
static const Vec3 V_M(18, 0, 0);
static const Vec3 P_T(300, 40, 0);
static const Vec3 V_T(-25, 0, 0);

static GuidanceState make_state(double t, double range, const Vec3 &u,
                                const Vec3 &vM, const Vec3 &vT) {
    GuidanceState st;
    st.t       = t;
    st.los_hat = u;
    st.range   = range;
    st.v_M     = vM;
    st.v_T     = vT;
    st.valid   = true;
    return st;
}

// ============================================================
//  1. Vec3 基础
// ============================================================
static void test_vec3() {
    section("1. Vec3 基础");
    near(dot(Vec3(1, 2, 3), Vec3(4, -5, 6)), 12.0, 1e-12, "dot");
    near_vec(cross(Vec3(1, 0, 0), Vec3(0, 1, 0)), Vec3(0, 0, 1), 1e-12, "cross(î,ĵ)=k̂");
    near(norm(Vec3(3, 4, 0)), 5.0, 1e-12, "norm((3,4,0))=5");
    near_vec(normalized(Vec3(0, 3, 0)), Vec3(0, 1, 0), 1e-12, "normalized((0,3,0))");
    near_vec(normalized(Vec3(0, 0, 0)), Vec3(0, 0, 0), 1e-12, "normalized(零矢量) 防除零");
}

// ============================================================
//  2. 讲义 §10 手算例
// ============================================================
static void test_handcalc() {
    section("2. 讲义 §10 手算例（R / Vc / λ̇ / |a| / t_go）");
    const Vec3   r     = P_T - P_M;
    const double R     = norm(r);
    const Vec3   v_rel = V_T - V_M;
    const Vec3   u     = normalized(r);

    near(R, 302.7, 0.05, "R ≈ 302.7 m");

    const double Vc = closing_velocity(r, v_rel, R);
    near(Vc, 42.6, 0.05, "Vc ≈ 42.6 m/s");

    const Vec3 w = los_rate_vector(r, v_rel, R);
    near(norm(w), 0.0188, 1e-4, "|ω| ≈ 0.0188 rad/s (=1.08°/s)");
    near(w.z, 0.0188, 1e-4, "ω 沿 +z（2D 平面外）");

    // 铁律①：分母是 R²。等价形式 (û×v_rel)/R 应给同一结果。
    near_vec(w, cross(u, v_rel) / R, 1e-12, "ω = (r×v_rel)/R²  ≡  (û×v_rel)/R");
    near(norm(w), norm(cross(r, v_rel)) / (R * R), 1e-12, "|ω| = |r×v_rel| / R²");

    const double N = 4.0;
    near(norm(pn_tpn(w, u, Vc, N)), 3.20, 0.02, "|a|_TPN(N=4) ≈ 3.20 m/s² (=0.33g)");
    near(norm(pn_ppn(w, V_M, N)), 1.35, 0.02, "|a|_PPN(N=4) ≈ 1.35 m/s²");
    near(R / Vc, 7.1, 0.02, "t_go = R/Vc ≈ 7.1 s");
}

// ============================================================
//  3. ω 两套算法一致性
// ============================================================
static void test_los_rate_two_ways() {
    section("3. ω 两套算法一致性（真值法 vs 视线差分法）");
    const Vec3   r     = P_T - P_M;
    const double R     = norm(r);
    const Vec3   v_rel = V_T - V_M;
    const Vec3   u     = normalized(r);

    const Vec3 w_true = los_rate_vector(r, v_rel, R);

    // 用 dû/dt = ω×û 前推一小步，再用差分法反算 ω
    const double dt     = 1e-3;
    const Vec3   u_next = normalized(u + cross(w_true, u) * dt);
    const Vec3   w_diff = los_rate_from_los_difference(u_next, u, dt);
    near_vec(w_diff, w_true, 1e-4, "差分法 ω ≈ 真值法 ω");

    near_vec(los_rate_from_los_difference(u, u, 0.0), Vec3(0, 0, 0), 1e-12,
             "dt=0 → 零矢量（防除零）");
}

// ============================================================
//  4. 接近速度符号
// ============================================================
static void test_closing_velocity() {
    section("4. 接近速度 Vc 符号");
    near(closing_velocity(Vec3(100, 0, 0), Vec3(-10, 0, 0), 100.0), 10.0, 1e-12,
         "接近 → Vc > 0");
    near(closing_velocity(Vec3(100, 0, 0), Vec3(10, 0, 0), 100.0), -10.0, 1e-12,
         "远离 → Vc < 0");
}

// ============================================================
//  5. guidance_step 端到端
// ============================================================
static void test_guidance_step_laws() {
    section("5. guidance_step 端到端（TPN / PPN / APN）");
    const Vec3   r = P_T - P_M;
    const double R = norm(r);
    const Vec3   u = normalized(r);

    GuidanceParams p;
    p.N               = 4.0;
    p.a_max           = 1000.0;   // 本组不限幅
    p.t_go_freeze     = 0.3;
    p.los_rate_source = LosRateSource::FROM_RELATIVE_VELOCITY;

    // --- TPN ---
    {
        GuidanceMemory m;
        p.law = GuidanceLaw::TPN;
        const GuidanceCmd c = guidance_step(make_state(0.0, R, u, V_M, V_T), p, m);
        near(c.a_cmd_norm, 3.20, 0.02, "TPN |a| ≈ 3.20");
        near(c.t_go, 7.1, 0.02, "TPN t_go ≈ 7.1");
        near(c.lam_dot, 0.0188, 1e-4, "TPN lam_dot ≈ 0.0188");
        near(c.Vc, 42.6, 0.05, "TPN Vc ≈ 42.6");
        ok(c.mode == GuidanceMode::PN, "TPN mode = PN");
        ok(!c.sat, "TPN 未饱和");
    }

    // --- PPN ---
    {
        GuidanceMemory m;
        p.law = GuidanceLaw::PPN;
        const GuidanceCmd c = guidance_step(make_state(0.0, R, u, V_M, V_T), p, m);
        near(c.a_cmd_norm, 1.35, 0.02, "PPN |a| ≈ 1.35");
        ok(c.mode == GuidanceMode::PN, "PPN mode = PN");
    }

    // --- APN：a_T⊥ = 0 时退化为 TPN ---
    {
        GuidanceMemory m;
        p.law = GuidanceLaw::APN;
        GuidanceState st = make_state(0.0, R, u, V_M, V_T);
        st.a_T_perp = Vec3(0, 0, 0);
        const GuidanceCmd c = guidance_step(st, p, m);
        near(c.a_cmd_norm, 3.20, 0.02, "APN(a_T⊥=0) |a| ≈ TPN");
        ok(c.mode == GuidanceMode::APN, "APN mode = APN");
    }

    // --- APN 补偿项恒等式：APN = TPN + (N/2)·a_T⊥ ---
    {
        GuidanceMemory m;
        p.law = GuidanceLaw::APN;
        const Vec3 a_T(0, 5, 0);            // 故意给非垂直分量，验证内部投影
        GuidanceState st = make_state(0.0, R, u, V_M, V_T);
        st.a_T_perp = a_T;
        const GuidanceCmd c = guidance_step(st, p, m);

        const Vec3   v_rel  = V_T - V_M;
        const double Vc     = closing_velocity(r, v_rel, R);
        const Vec3   w      = los_rate_vector(r, v_rel, R);
        const Vec3   a_tpn  = pn_tpn(w, u, Vc, p.N);
        const Vec3   a_perp = a_T - u * dot(a_T, u);   // 手工投影
        near_vec(c.a_cmd_unsat - a_tpn, a_perp * (p.N * 0.5), 1e-9,
                 "APN = TPN + (N/2)·a_T⊥ 恒等式");
    }
}

// ============================================================
//  6. 三条铁律
// ============================================================
static void test_iron_rules() {
    const Vec3   r = P_T - P_M;
    const double R = norm(r);
    const Vec3   u = normalized(r);

    GuidanceParams p;
    p.law             = GuidanceLaw::TPN;
    p.N               = 4.0;
    p.t_go_freeze     = 0.3;
    p.los_rate_source = LosRateSource::FROM_RELATIVE_VELOCITY;

    // --- 铁律②：过载限幅 = 矢量等比缩放 ---
    section("6a. 铁律② 过载限幅（矢量等比缩放，禁止逐分量 clip）");
    {
        GuidanceMemory m;
        p.a_max = 1.0;
        const GuidanceCmd c = guidance_step(make_state(0.0, R, u, V_M, V_T), p, m);
        ok(c.sat, "触发 sat");
        near(c.a_cmd_norm, 1.0, 1e-9, "|a_cmd| = a_max（精确）");
        near(norm(cross(c.a_cmd, c.a_cmd_unsat)), 0.0, 1e-9, "方向不变（叉积 = 0）");
        ok(dot(c.a_cmd, c.a_cmd_unsat) > 0, "方向同向（点积 > 0）");
        // 等比：各分量缩放系数相同
        const double kx = c.a_cmd.x / c.a_cmd_unsat.x;
        const double ky = c.a_cmd.y / c.a_cmd_unsat.y;
        near(kx, ky, 1e-9, "各分量缩放系数相同（等比）");
    }

    // --- 铁律③：末端冻结（HOLD）---
    section("6b. 铁律③ 末端冻结 HOLD（t_go < 阈值）");
    {
        GuidanceMemory m;
        p.a_max = 1000.0;
        const GuidanceCmd c1 = guidance_step(make_state(0.0, R, u, V_M, V_T), p, m);
        ok(c1.mode == GuidanceMode::PN, "远距 → PN");

        // 距离缩到 1 m → t_go ≈ 0.023 s < 0.3 → HOLD，且沿用上一帧指令
        const GuidanceCmd c2 = guidance_step(make_state(0.002, 1.0, u, V_M, V_T), p, m);
        ok(c2.mode == GuidanceMode::HOLD, "近距 → HOLD");
        ok(c2.t_go < p.t_go_freeze, "t_go < 冻结阈值");
        near_vec(c2.a_cmd, c1.a_cmd, 1e-12, "HOLD 沿用上一帧指令（挡住 1/R 放大）");
    }

    // --- 目标丢失 → SEARCH 且清历史 ---
    section("6c. 目标丢失 → SEARCH（并清记忆）");
    {
        GuidanceMemory m;
        p.a_max = 1000.0;
        (void)guidance_step(make_state(0.0, R, u, V_M, V_T), p, m);
        ok(m.has_prev, "有效帧后 has_prev = true");

        GuidanceState bad = make_state(0.002, R, u, V_M, V_T);
        bad.valid = false;
        const GuidanceCmd c2 = guidance_step(bad, p, m);
        ok(c2.mode == GuidanceMode::SEARCH, "SEARCH 模式");
        near_vec(c2.a_cmd, Vec3(0, 0, 0), 1e-12, "零指令");
        near(c2.a_cmd_norm, 0.0, 1e-12, "|a_cmd| = 0");
        ok(!m.has_prev, "已清记忆（重新捕获从零开始）");
    }
}

// ============================================================
//  7. ω 来源切换
// ============================================================
static void test_los_rate_source_switch() {
    section("7. ω 来源切换（FROM_LOS_DIFFERENCE 在 guidance_step 中生效）");
    const Vec3   r      = P_T - P_M;
    const double R      = norm(r);
    const Vec3   u      = normalized(r);
    const Vec3   w_true = los_rate_vector(r, V_T - V_M, R);

    GuidanceParams p;
    p.law             = GuidanceLaw::TPN;
    p.N               = 4.0;
    p.a_max           = 1000.0;
    p.t_go_freeze     = 0.3;
    p.los_rate_source = LosRateSource::FROM_LOS_DIFFERENCE;

    GuidanceMemory m;
    const double   dt = 1e-3;

    // 第一帧：无历史 → ω 保持为零
    const GuidanceCmd c1 = guidance_step(make_state(0.0, R, u, V_M, V_T), p, m);
    near(c1.lam_dot, 0.0, 1e-12, "首帧无历史 → lam_dot = 0");

    // 第二帧：视线转过一小步 → 差分法应还原出 ω
    const Vec3        u2 = normalized(u + cross(w_true, u) * dt);
    const GuidanceCmd c2 = guidance_step(make_state(dt, R, u2, V_M, V_T), p, m);
    near(c2.lam_dot, norm(w_true), 1e-4, "次帧差分法 lam_dot ≈ 真值");
}

// ============================================================
//  8. 传感器墙：投影往返（无噪 → 应还原真值）
// ============================================================
static TruthState make_truth_for_sensor() {
    TruthState tr;
    tr.t   = 0.0;
    tr.p_M = Vec3(0, 0, 0);
    tr.v_M = Vec3(30, 0, 0);
    tr.p_T = Vec3(300, 30, 10);
    tr.v_T = Vec3(-30, 0, 0);
    tr.a_T = Vec3(0, 0, 0);
    quat_from_forward(tr.v_M, tr.q);   // 机头朝速度方向
    return tr;
}

static void test_sensor_projection() {
    section("8. 传感器墙：投影往返（无噪 → 还原真值）");
    const TruthState tr = make_truth_for_sensor();

    SensorConfig cfg;
    cfg.fidelity        = SensorFidelity::SEMI_REAL;
    cfg.pixel_noise_std = 0.0;     // 关噪声
    cfg.range_scale_err = 0.0;
    Sensor sensor(cfg);

    const GuidanceState st = sensor.observe(tr, 0.0);
    const Vec3          r  = tr.p_T - tr.p_M;

    ok(st.valid, "有效帧");
    near_vec(st.los_hat, normalized(r), 1e-9, "还原视线方向 û（世界→像素→世界）");
    // size 反推得到的是沿光轴的"深度"，不是斜距；近轴时二者接近
    near(st.range, dot(normalized(tr.v_M), r), 1e-6, "距离 = 沿光轴深度");
    near_vec(st.v_T, Vec3(0, 0, 0), 1e-12, "SEMI：目标速度未知（置零）");
}

// ============================================================
//  9. 姿态四元数
// ============================================================
static void test_quat() {
    section("9. 姿态四元数（分量顺序 [w,x,y,z]，机头朝速度方向）");
    double q[4];
    quat_from_forward(Vec3(1, 0, 0), q);
    near_vec(quat_rotate_vec(q, Vec3(0, 0, 1)), Vec3(1, 0, 0), 1e-12,
             "R(q)·(0,0,1) = 前向");
    const Vec3 w(3, -2, 5);
    near_vec(quat_rotate_vec(q, quat_rotate_vec_inv(q, w)), w, 1e-12, "旋转往返");

    quat_from_forward(Vec3(0, 0, 1), q);
    near(q[0], 1.0, 1e-12, "forward = +z → 单位四元数（q[0]=w=1）");

    // ★★ 分量顺序锁定 [w,x,y,z]（与 PX4 / MAVLink 一致）★★
    //    绕世界 +z(上) 转 +90° 的四元数 = [cos45, 0, 0, sin45]。
    //    若有人把顺序改回 [x,y,z,w]，下面两条会立刻 FAIL。
    {
        const double s2   = std::sqrt(0.5);
        const double qz[4] = {s2, 0.0, 0.0, s2};      // [w, x, y, z]
        near_vec(quat_rotate_vec(qz, Vec3(1, 0, 0)), Vec3(0, 1, 0), 1e-12,
                 "顺序锁定：绕 +z 转 90° 把东→北");
        near_vec(quat_rotate_vec(qz, Vec3(0, 1, 0)), Vec3(-1, 0, 0), 1e-12,
                 "顺序锁定：绕 +z 转 90° 把北→西");
    }
    near(GuidanceState{}.q[0], 1.0, 1e-12,
         "GuidanceState 默认 q = [1,0,0,0]（[w,x,y,z] 单位四元数）");
}

// ============================================================
//  10. 逼真度语义
// ============================================================
static void test_sensor_fidelity() {
    section("10. 传感器逼真度语义（IDEAL 透传 / SEMI 丢 v_T）");
    const TruthState tr = make_truth_for_sensor();

    SensorConfig cfg;
    cfg.fidelity = SensorFidelity::IDEAL;
    Sensor s_ideal(cfg);
    const GuidanceState a = s_ideal.observe(tr, 0.0);
    near_vec(a.v_T, tr.v_T, 1e-12, "IDEAL 透传目标速度 v_T");
    near(a.range, norm(tr.p_T - tr.p_M), 1e-9, "IDEAL range = 真值斜距");

    cfg.fidelity = SensorFidelity::SEMI_REAL;
    cfg.pixel_noise_std = 0.0;
    Sensor s_semi(cfg);
    const GuidanceState b = s_semi.observe(tr, 0.0);
    near_vec(b.v_T, Vec3(0, 0, 0), 1e-12, "SEMI：v_T 未知（制导须改用差分法）");
    ok(b.valid, "SEMI 有效帧");
}

// ============================================================
//  11. 丢帧
// ============================================================
static void test_sensor_drop() {
    section("11. 丢帧（FULL_REAL, drop_rate = 1）");
    const TruthState tr = make_truth_for_sensor();

    SensorConfig cfg;
    cfg.fidelity  = SensorFidelity::FULL_REAL;
    cfg.drop_rate = 1.0;
    Sensor sensor(cfg);

    bool all_invalid = true;
    for (int i = 0; i < 20; ++i) {
        const GuidanceState st = sensor.observe(tr, i * 0.01);
        if (st.valid) all_invalid = false;
    }
    ok(all_invalid, "全部帧无效");
    ok(sensor.stats().dropped == sensor.stats().frames, "dropped == frames");
}

// ============================================================
//  12. 链路延迟
// ============================================================
static void test_sensor_latency() {
    section("12. 链路延迟（FULL_REAL, latency = 50 ms）");
    const TruthState tr = make_truth_for_sensor();

    SensorConfig cfg;
    cfg.fidelity   = SensorFidelity::FULL_REAL;
    cfg.drop_rate  = 0.0;
    cfg.latency_s  = 0.05;
    Sensor sensor(cfg);

    const double dt = 0.01;
    GuidanceState st;
    for (int i = 0; i <= 20; ++i) st = sensor.observe(tr, i * dt);   // t = 0 .. 0.20

    near(st.t, 0.20 - 0.05, 1e-9, "输出滞后 ≈ latency（t_out = t - 50ms）");
}

// ============================================================
//  13. 目标模型（M2）：解析式 / 预积分 / 气动阻力
// ============================================================
static constexpr double kPi = 3.14159265358979323846;

static void test_target_model() {
    section("13. 目标模型 TargetModel");

    // --- 13.1 发射参数 → 初速度 ---
    {
        TargetConfig c;
        c.type = TargetType::DART;
        c.launch_speed = 20.0;
        TargetModel m(c);
        near_vec(m.launch_velocity(), Vec3(20, 0, 0), 1e-9, "azim=0, elev=0 → +x");

        c.launch_elev = kPi / 2.0;
        TargetModel m2(c);
        near_vec(m2.launch_velocity(), Vec3(0, 0, 20), 1e-9, "elev=90° → +z");

        c.launch_elev = 0.0;
        c.launch_azim = kPi;
        TargetModel m3(c);
        near_vec(m3.launch_velocity(), Vec3(-20, 0, 0), 1e-9, "azim=180° → −x");

        c.launch_azim = kPi / 2.0;
        TargetModel m4(c);
        near_vec(m4.launch_velocity(), Vec3(0, 20, 0), 1e-9, "azim=90° → +y");
    }

    // --- 13.2 STRAIGHT：匀速直线 ---
    {
        TargetConfig c;
        c.type = TargetType::STRAIGHT;
        c.p0 = Vec3(1, 2, 3);
        c.v0 = Vec3(4, -5, 6);
        TargetModel m(c);
        const TargetState s = m.state(2.0);
        near_vec(s.p, Vec3(9, -8, 15), 1e-9, "STRAIGHT p = p0 + v0·t");
        near_vec(s.v, Vec3(4, -5, 6), 1e-9, "STRAIGHT v 恒定");
        near_vec(s.a, Vec3(0, 0, 0), 1e-12, "STRAIGHT a = 0");
    }

    // --- 13.3 BALLISTIC：解析抛物线 + 解析落地时刻 ---
    {
        TargetConfig c;
        c.type = TargetType::BALLISTIC;
        c.p0 = Vec3(0, 0, 10);
        c.v0 = Vec3(5, 0, 8);
        TargetModel m(c);
        const double t = 1.3;
        const TargetState s = m.state(t);
        near_vec(s.p, Vec3(5 * t, 0, 10 + 8 * t - 0.5 * 9.81 * t * t), 1e-9,
                 "BALLISTIC p = p0 + v0·t + ½g·t²");
        near_vec(s.v, Vec3(5, 0, 8 - 9.81 * t), 1e-9, "BALLISTIC v = v0 + g·t");
        near_vec(s.a, Vec3(0, 0, -9.81), 1e-12, "BALLISTIC a = g");
        // 10 + 8t − 4.905t² = 0 → t = (8 + √(64 + 196.2)) / 9.81
        const double t_land = (8.0 + std::sqrt(64.0 + 4.0 * 4.905 * 10.0)) / (2.0 * 4.905);
        near(m.flight_time(), t_land, 1e-9, "BALLISTIC 落地时刻 = 解析解");
        ok(m.hits_ground(), "BALLISTIC 标记为落地");
    }

    // --- 13.4 ★ 交叉验证：DART 且 Cd=0 应当 ≡ BALLISTIC ---
    {
        TargetConfig cd;
        cd.type = TargetType::DART;
        cd.drag_cd = 0.0;
        cd.p0 = Vec3(0, 0, 10);
        cd.v0 = Vec3(5, 0, 8);
        TargetModel md(cd);

        TargetConfig cb = cd;
        cb.type = TargetType::BALLISTIC;
        TargetModel mb(cb);

        bool same = true;
        for (int i = 0; i <= 20; ++i) {
            const double t = 2.0 * i / 20.0;
            const TargetState a = md.state(t), b = mb.state(t);
            if (norm(a.p - b.p) > 1e-6 || norm(a.v - b.v) > 1e-6) same = false;
        }
        ok(same, "DART(Cd=0) ≡ BALLISTIC（数值预积分 vs 解析式，逐点 < 1e-6）");
    }

    // --- 13.5 气动阻力：射程变短 / 时长变短 / 落点变慢 / 上升段单调减速 ---
    {
        TargetConfig c;
        c.type = TargetType::DART;
        c.p0 = Vec3(0, 0, 1.0);
        c.launch_speed = 18.0;
        c.launch_elev  = 0.5;
        c.launch_azim  = kPi;
        TargetModel md(c);

        TargetConfig c0 = c;
        c0.drag_cd = 0.0;
        TargetModel m0(c0);

        const TargetState sd = md.state(md.flight_time());
        const TargetState s0 = m0.state(m0.flight_time());
        const double rd = std::fabs(sd.p.x - c.p0.x);
        const double r0 = std::fabs(s0.p.x - c.p0.x);
        ok(rd < r0, "阻力使水平射程变短");
        ok(md.flight_time() < m0.flight_time(), "阻力使飞行时长变短");
        ok(norm(sd.v) < norm(s0.v), "阻力使落点速度变小");

        // 上升段速率应单调衰减（下降段重力加速会让速率回升，故只看前 0.5 s）
        bool mono = true;
        double prev = norm(md.state(0.0).v);
        for (int i = 1; i <= 20; ++i) {
            const double sp = norm(md.state(0.5 * i / 20.0).v);
            if (sp > prev + 1e-9) mono = false;
            prev = sp;
        }
        ok(mono, "上升段速率单调衰减");
    }

    // --- 13.6 DART 的 a_T = g + 阻力，且阻力与相对速度反向 ---
    {
        TargetConfig c;
        c.type = TargetType::DART;
        c.p0 = Vec3(0, 0, 10);
        c.launch_speed = 20.0;
        c.launch_elev  = 0.0;
        c.launch_azim  = 0.0;
        TargetModel m(c);

        const TargetState s = m.state(0.0);
        const Vec3   a_drag = s.a - c.gravity;   // 剥离重力
        const double k      = 0.5 * c.rho * c.drag_cd * c.drag_area / c.mass;
        near_vec(a_drag, s.v * (-k * norm(s.v)), 1e-9, "a_drag = −k·|v|·v");
        ok(dot(a_drag, s.v) < 0.0, "阻力与速度反向（点积 < 0）");
        ok(norm(s.a) > norm(c.gravity), "水平发射时 |a_T| > g（阻力叠在重力上）");

        // 有风时用相对速度
        TargetConfig cw = c;
        cw.wind = Vec3(-5, 0, 0);
        TargetModel mw(cw);
        const Vec3 v_rel = mw.state(0.0).v - cw.wind;
        ok(norm(v_rel) > norm(mw.state(0.0).v), "有风时 v_rel = v − wind");
    }

    // --- 13.7 CIRCLE：速率恒定、a = ω×v 且 ⊥ v ---
    {
        TargetConfig c;
        c.type = TargetType::CIRCLE;
        c.p0 = Vec3(0, 0, 5);
        c.v0 = Vec3(10, 0, 0);
        c.turn_rate = 0.5;
        TargetModel m(c);

        const TargetState s = m.state(1.0);
        near(norm(s.v), 10.0, 1e-6, "CIRCLE |v| 恒定");
        near_vec(s.a, cross(Vec3(0, 0, 0.5), s.v), 1e-9, "CIRCLE a = ω×v");
        near(dot(s.a, s.v), 0.0, 1e-9, "CIRCLE a ⊥ v（向心加速度）");
    }

    // --- 13.8 SNAKE：侧向正弦加速度，无纵/垂向分量 ---
    {
        TargetConfig c;
        c.type = TargetType::SNAKE;
        c.p0 = Vec3(0, 0, 5);
        c.v0 = Vec3(10, 0, 0);
        c.snake_acc  = 5.0;
        c.snake_freq = 2.0;
        TargetModel m(c);

        const TargetState s = m.state(0.25);
        near(s.a.x, 0.0, 1e-9, "SNAKE 无纵向加速度");
        near(s.a.z, 0.0, 1e-9, "SNAKE 无垂向加速度");
        near(s.a.y, 5.0 * std::sin(2.0 * 0.25), 1e-9, "SNAKE 侧向加速度 = A·sin(ωt)");
    }

    // --- 13.9 查表：端点 / 越界夹持 / 连续性 ---
    {
        TargetConfig c;
        c.type = TargetType::DART;
        c.p0 = Vec3(10, 0, 3);
        c.launch_speed = 15.0;
        c.launch_elev  = 0.4;
        c.launch_azim  = kPi;
        TargetModel m(c);

        near_vec(m.state(0.0).p, c.p0, 1e-12, "state(0).p = 初始位置");
        near_vec(m.state(0.0).v, m.launch_velocity(), 1e-12, "state(0).v = 初速度");
        const TargetState s_end = m.state(m.flight_time());
        near_vec(m.state(m.flight_time() + 5.0).p, s_end.p, 1e-12, "t 越界 → 夹到末态");

        bool cont = true;
        const double dt = 0.001;
        for (int i = 0; i < 400; ++i) {
            const double t = i * 0.002;
            const TargetState a = m.state(t), b = m.state(t + dt);
            if (norm(b.p - (a.p + a.v * dt)) > 1e-4) cont = false;
        }
        ok(cont, "查表插值连续（p(t+dt) ≈ p(t) + v(t)·dt）");
    }
}

// ============================================================
//  14. 末端冻结判据：t_go 基准 / R 基准 / 两者取或
// ============================================================
static void test_freeze_criterion() {
    section("14. 末端冻结判据（t_go / R / 两者取或）");
    const Vec3   r  = P_T - P_M;
    const Vec3   u  = normalized(r);
    const double Vc = closing_velocity(r, V_T - V_M, norm(r));   // ≈ 42.6 m/s

    GuidanceParams p;
    p.law             = GuidanceLaw::TPN;
    p.N               = 4.0;
    p.a_max           = 1000.0;   // 本组不限幅
    p.los_rate_source = LosRateSource::FROM_RELATIVE_VELOCITY;

    // --- 14.1 R 基准：R < R_freeze → HOLD，并沿用上一帧指令 ---
    {
        GuidanceParams pr = p;
        pr.freeze_criterion = FreezeCriterion::BY_RANGE;
        pr.R_freeze         = 5.0;

        GuidanceMemory m;
        const GuidanceCmd c1 = guidance_step(make_state(0.0, 100.0, u, V_M, V_T), pr, m);
        ok(c1.mode == GuidanceMode::PN, "R=100 > 5 → 不冻结");
        const GuidanceCmd c2 = guidance_step(make_state(0.002, 1.0, u, V_M, V_T), pr, m);
        ok(c2.mode == GuidanceMode::HOLD, "R=1 < 5 → HOLD");
        near_vec(c2.a_cmd, c1.a_cmd, 1e-12, "R 基准 HOLD 同样沿用上一帧指令");
    }

    // --- 14.2 同一 Vc 下 t_go 基准 ≡ R 基准（取 R_freeze = Vc · t_go_freeze）---
    {
        GuidanceParams a = p;
        a.freeze_criterion = FreezeCriterion::BY_TGO;
        a.t_go_freeze      = 0.15;

        GuidanceParams b = p;
        b.freeze_criterion = FreezeCriterion::BY_RANGE;
        b.R_freeze         = Vc * 0.15;   // ≈ 6.4 m

        bool agree = true;
        for (int i = 0; i <= 40; ++i) {
            const double R = 20.0 - 19.0 * i / 40.0;   // 20 → 1 m
            GuidanceMemory ma, mb;
            const GuidanceCmd ca = guidance_step(make_state(0.0, R, u, V_M, V_T), a, ma);
            const GuidanceCmd cb = guidance_step(make_state(0.0, R, u, V_M, V_T), b, mb);
            if ((ca.mode == GuidanceMode::HOLD) != (cb.mode == GuidanceMode::HOLD)) {
                agree = false;
            }
        }
        ok(agree, "R_freeze = Vc·t_go_freeze 时两判据逐点一致");
    }

    // --- 14.3 Vc → 0 时，R 兜底防止"永不冻结" ---
    //     不接近（Vc = 0）→ t_go = R/max(Vc,ε) 发散，仅 t_go 基准永不冻结；
    //     加上 R 兜底后仍能在近距冻结。
    {
        GuidanceState st = make_state(0.0, 3.0, Vec3(1, 0, 0), Vec3(0, 0, 0), Vec3(0, 10, 0));
        near(closing_velocity(Vec3(3, 0, 0), Vec3(0, 10, 0), 3.0), 0.0, 1e-12,
             "构造 Vc = 0（v_rel ⊥ r）");

        GuidanceParams pt = p;
        pt.freeze_criterion = FreezeCriterion::BY_TGO;
        pt.t_go_freeze      = 0.15;
        GuidanceMemory mt;
        const GuidanceCmd ct = guidance_step(st, pt, mt);
        ok(ct.mode != GuidanceMode::HOLD, "Vc=0：仅 t_go 基准 → 不冻结（t_go 发散）");

        GuidanceParams ph = pt;
        ph.freeze_criterion = FreezeCriterion::BY_TGO_OR_RANGE;
        ph.R_freeze         = 5.0;
        GuidanceMemory mh;
        const GuidanceCmd ch = guidance_step(st, ph, mh);
        ok(ch.mode == GuidanceMode::HOLD, "Vc=0：R 兜底 → 冻结（R=3 < 5）");
    }
}

// ============================================================
//  15. ★ 不变量（invariant）—— 内部口径冻结
// ============================================================
//
// 《数据对齐-自决项详解.md》④「内部口径冻结」要求的四类不变量：
//     |q| = 1          · |los_hat| = 1      · range ≥ 0      · 时间戳单调不减（见 15.3）
// 这些是**制导契约**：上游（视觉 / 惯导 / 通信）无论怎么给数据，
// 只要转换为内部口径后违反其中任何一条，制导就必然出错。
// 本组测试把它们钉死，防止后续改动悄悄破坏。
//
// 配套：接口侧还有一套更完整的自检实现（`src/interface/state_adapter/state_check.hpp`，
//       收到一帧就能体检并指出是哪个字段的问题），本组只做制导核的契约级验证。
static void test_invariants() {
    section("15. 不变量（内部口径冻结）");

    // 判据（与 state_check.hpp 的阈值保持一致）
    const double kQuatNormTol = 1e-9;
    const double kLosNormTol  = 1e-9;

    GuidanceParams p;   // 默认 PPN / N = 4 / a_max = 30

    // --- 15.1 正常闭环：所有不变量成立 ---
    {
        GuidanceState st = make_state(0.5, 120.0, Vec3(1, 0, 0), Vec3(20, 0, 0), Vec3(-10, 4, 0));
        GuidanceMemory m;
        const GuidanceCmd c = guidance_step(st, p, m);

        // |q| = 1（默认单位四元数）
        const double qn = std::sqrt(st.q[0] * st.q[0] + st.q[1] * st.q[1] +
                                    st.q[2] * st.q[2] + st.q[3] * st.q[3]);
        near(qn, 1.0, kQuatNormTol, "不变量 |q| = 1（默认 [1,0,0,0]）");
        near(st.q[0], 1.0, 0.0, "  四元数顺序 [w,x,y,z]：q[0] = w = 1");

        // |los_hat| = 1
        near(norm(st.los_hat), 1.0, kLosNormTol, "不变量 |los_hat| = 1");

        // range ≥ 0
        ok(st.range >= 0.0, "不变量 range ≥ 0");

        // 指令有限、限幅内
        ok(std::isfinite(c.a_cmd.x) && std::isfinite(c.a_cmd.y) && std::isfinite(c.a_cmd.z),
           "不变量 a_cmd 三个分量有限");
        ok(norm(c.a_cmd) <= p.a_max + 1e-9, "不变量 |a_cmd| ≤ a_max");

        // |a_cmd| 与 a_cmd_norm 一致
        near(norm(c.a_cmd), c.a_cmd_norm, 1e-12, "不变量 |a_cmd| = a_cmd_norm");

        // r_hat 是单位矢量（制导回填的视线方向）
        near(norm(c.r_hat), 1.0, kLosNormTol, "不变量 |r_hat| = 1");

        // cmd.t = state.t，R 回填正确
        near(c.t, st.t, 0.0, "不变量 cmd.t = state.t");
        near(c.R, st.range, 0.0, "不变量 cmd.R = state.range");
    }

    // --- 15.2 极端几何下不变量依然成立（限幅 / 冻结 / 丢失）---
    {
        // (a) 极大 λ̇：指令必被限幅，模长仍守 a_max
        GuidanceState st = make_state(0.01, 500.0, Vec3(1, 0, 0), Vec3(30, 0, 0), Vec3(-30, 60, 0));
        GuidanceParams pm = p;
        pm.a_max = 5.0;
        GuidanceMemory m;
        const GuidanceCmd c = guidance_step(st, pm, m);
        ok(c.sat, "极端几何：触发限幅");
        ok(norm(c.a_cmd) <= pm.a_max + 1e-9, "  限幅后 |a_cmd| ≤ a_max（矢量等比缩放）");
        near(norm(c.a_cmd), c.a_cmd_norm, 1e-12, "  |a_cmd| 与 a_cmd_norm 一致");
        // 未限幅指令与限幅指令方向一致（等比缩放的直接后果）
        if (norm(c.a_cmd_unsat) > 1e-9 && norm(c.a_cmd) > 1e-9) {
            const double cosang = dot(normalized(c.a_cmd), normalized(c.a_cmd_unsat));
            near(cosang, 1.0, 1e-12, "  限幅不改变方向（矢量等比缩放，非逐分量 clip）");
        }

        // (b) 末端冻结：R → 0 时 |a_cmd| 仍有限且 ≤ a_max
        GuidanceState st2 = make_state(1.0, 0.05, Vec3(1, 0, 0), Vec3(15, 0, 0), Vec3(-15, 1, 0));
        GuidanceMemory m2;
        const GuidanceCmd c2 = guidance_step(st2, p, m2);
        ok(std::isfinite(norm(c2.a_cmd)), "末端 R→0：a_cmd 仍有限（未发散）");
        ok(norm(c2.a_cmd) <= p.a_max + 1e-9, "  末端 |a_cmd| ≤ a_max");
        ok(c2.t_go >= 0.0, "  t_go ≥ 0");

        // (c) 目标丢失：SEARCH + 零指令
        GuidanceState st3 = make_state(1.0, 100.0, Vec3(1, 0, 0), Vec3(20, 0, 0), Vec3(0, 0, 0));
        st3.valid = false;
        GuidanceMemory m3;
        const GuidanceCmd c3 = guidance_step(st3, p, m3);
        ok(c3.mode == GuidanceMode::SEARCH, "目标丢失：mode = SEARCH");
        near(norm(c3.a_cmd), 0.0, 0.0, "  零指令");
        ok(std::isfinite(c3.a_cmd_norm), "  a_cmd_norm 有限（不是 NaN）");
    }

    // --- 15.3 时间戳单调性：制导不动时间，只透传；契约要求上游单调 ---
    {
        GuidanceState st = make_state(1.234, 100.0, Vec3(1, 0, 0), Vec3(20, 0, 0), Vec3(0, 0, 0));
        GuidanceMemory m;
        const GuidanceCmd c = guidance_step(st, p, m);
        near(c.t, 1.234, 0.0, "时间戳透传：cmd.t = state.t");
        ok(m.t_prev == st.t, "记忆里保存的时间戳 = 本帧时间戳");
        ok(m.has_prev, "  has_prev = true");

        // 差分法内部用 dt = t − t_prev；t 倒退会让 dt < 0 ⇒ 必须由上游保证单调
        GuidanceParams pd = p;
        pd.los_rate_source = LosRateSource::FROM_LOS_DIFFERENCE;
        GuidanceMemory md;
        GuidanceState s1 = make_state(1.0, 100.0, Vec3(1, 0, 0), Vec3(20, 0, 0), Vec3(0, 0, 0));
        guidance_step(s1, pd, md);
        // 第二帧时间戳倒退（上游乱序）⇒ 制导核自身不做保护（防除零只挡 dt≈0）
        GuidanceState s2 = make_state(0.5, 100.0, Vec3(1, 0.01, 0), Vec3(20, 0, 0), Vec3(0, 0, 0));
        const GuidanceCmd cd = guidance_step(s2, pd, md);
        std::printf("  （提示：时间戳倒退时 dt = %.3f s，制导不自行纠正 —— "
                    "单调性由上游保证，见 state_adapter::StateTimeTracker）\n",
                    s2.t - s1.t);
        ok(std::isfinite(norm(cd.a_cmd)), "时间戳倒退时输出仍有限（不产生 NaN/Inf）");
    }

    // --- 15.4 单位约定的自证：内部一律 SI + rad + 秒 ---
    {
        // los_hat / v_M / a_cmd 都是"同一世界系里的矢量"，所以：
        //   PPN 的 a = N(ω × v_M) 与 v_M、ω 同系；量纲 = (rad/s)·(m/s) = m/s²
        GuidanceState st = make_state(2.0, 100.0, Vec3(1, 0, 0), Vec3(20, 0, 0), Vec3(-5, 5, 0));
        GuidanceMemory m;
        const GuidanceCmd c = guidance_step(st, p, m);
        // 手算复核量纲：ω = (r × v_rel)/R², r = (100,0,0), v_rel = (-25,5,0)
        const Vec3 r = st.los_hat * st.range;
        const Vec3 v_rel = st.v_T - st.v_M;
        const Vec3 omega = los_rate_vector(r, v_rel, st.range);
        near(norm(omega), c.lam_dot, 1e-12, "λ̇ 量纲/数值：|ω| 与 cmd.lam_dot 一致 (rad/s)");
        const Vec3 a_expect = pn_ppn(omega, st.v_M, p.N);
        near_vec(c.a_cmd_unsat, a_expect, 1e-9,
                 "PPN：a = N(ω × v_M)（单位 m/s²，世界系）");
    }
}

// ============================================================
int main() {
    std::printf("制导模块单元测试 test_guidance\n");

    test_vec3();
    test_handcalc();
    test_los_rate_two_ways();
    test_closing_velocity();
    test_guidance_step_laws();
    test_iron_rules();
    test_los_rate_source_switch();
    test_sensor_projection();
    test_quat();
    test_sensor_fidelity();
    test_sensor_drop();
    test_sensor_latency();
    test_target_model();
    test_freeze_criterion();
    test_invariants();

    std::printf("\n----------------------------------------\n");
    std::printf("通过 %d, 失败 %d\n", g_pass, g_fail);
    std::printf("%s\n", (g_fail == 0) ? "ALL PASS" : "SOME TESTS FAILED");
    return (g_fail == 0) ? 0 : 1;
}
