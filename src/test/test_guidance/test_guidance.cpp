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
//   15. 不变量（内部口径冻结）
//   16. ★ 机载主循环外壳 GuidanceLoop：像素→世界换算、PN 方向、SEARCH、
//       降级策略（保持上一条 0.2 s → 零速度悬停）、数据过期、无新帧重发、统计
//   17. ★ 端到端闭环回归：合成世界里真跑一整段交会（min R / 饱和 / HOLD / 每拍发送）
//   18. ★ CSV 录制 → 回放：同一份输入喂回同一个 loop，输出必须逐位一致（黑匣子）
//
// 运行：./bin/test/test_guidance   （退出码 0=全过，1=有失败）

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include "vec3.hpp"
#include "guidance_types.hpp"
#include "pn_law.hpp"
#include "guidance.hpp"
#include "sensor.hpp"
#include "target.hpp"
#include "guidance_loop.hpp"     // 机载主循环外壳（§16）
#include "synthetic_world.hpp"   // 合成世界 + 闭环驱动（§17）
#include "csv_io.hpp"            // 录制 / 回放（§18）

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
        const PureGuidanceCmd c = guidance_step(make_state(0.0, R, u, V_M, V_T), p, m);
        near(c.a_cmd_norm, 3.20, 0.02, "TPN |a| ≈ 3.20");
        near(c.t_go, 7.1, 0.02, "TPN t_go ≈ 7.1");
        near(c.lam_dot, 0.0188, 1e-4, "TPN lam_dot ≈ 0.0188");
        near(c.Vc, 42.6, 0.05, "TPN Vc ≈ 42.6");
        ok(c.mode == PureGuidanceMode::PN, "TPN mode = PN");
        ok(!c.sat, "TPN 未饱和");
    }

    // --- PPN ---
    {
        GuidanceMemory m;
        p.law = GuidanceLaw::PPN;
        const PureGuidanceCmd c = guidance_step(make_state(0.0, R, u, V_M, V_T), p, m);
        near(c.a_cmd_norm, 1.35, 0.02, "PPN |a| ≈ 1.35");
        ok(c.mode == PureGuidanceMode::PN, "PPN mode = PN");
    }

    // --- APN：a_T⊥ = 0 时退化为 TPN ---
    {
        GuidanceMemory m;
        p.law = GuidanceLaw::APN;
        GuidanceState st = make_state(0.0, R, u, V_M, V_T);
        st.a_T_perp = Vec3(0, 0, 0);
        const PureGuidanceCmd c = guidance_step(st, p, m);
        near(c.a_cmd_norm, 3.20, 0.02, "APN(a_T⊥=0) |a| ≈ TPN");
        ok(c.mode == PureGuidanceMode::APN, "APN mode = APN");
    }

    // --- APN 补偿项恒等式：APN = TPN + (N/2)·a_T⊥ ---
    {
        GuidanceMemory m;
        p.law = GuidanceLaw::APN;
        const Vec3 a_T(0, 5, 0);            // 故意给非垂直分量，验证内部投影
        GuidanceState st = make_state(0.0, R, u, V_M, V_T);
        st.a_T_perp = a_T;
        const PureGuidanceCmd c = guidance_step(st, p, m);

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
        const PureGuidanceCmd c = guidance_step(make_state(0.0, R, u, V_M, V_T), p, m);
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
        const PureGuidanceCmd c1 = guidance_step(make_state(0.0, R, u, V_M, V_T), p, m);
        ok(c1.mode == PureGuidanceMode::PN, "远距 → PN");

        // 距离缩到 1 m → t_go ≈ 0.023 s < 0.3 → HOLD，且沿用上一帧指令
        const PureGuidanceCmd c2 = guidance_step(make_state(0.002, 1.0, u, V_M, V_T), p, m);
        ok(c2.mode == PureGuidanceMode::HOLD, "近距 → HOLD");
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
        const PureGuidanceCmd c2 = guidance_step(bad, p, m);
        ok(c2.mode == PureGuidanceMode::SEARCH, "SEARCH 模式");
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
    const PureGuidanceCmd c1 = guidance_step(make_state(0.0, R, u, V_M, V_T), p, m);
    near(c1.lam_dot, 0.0, 1e-12, "首帧无历史 → lam_dot = 0");

    // 第二帧：视线转过一小步 → 差分法应还原出 ω
    const Vec3        u2 = normalized(u + cross(w_true, u) * dt);
    const PureGuidanceCmd c2 = guidance_step(make_state(dt, R, u2, V_M, V_T), p, m);
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
        const PureGuidanceCmd c1 = guidance_step(make_state(0.0, 100.0, u, V_M, V_T), pr, m);
        ok(c1.mode == PureGuidanceMode::PN, "R=100 > 5 → 不冻结");
        const PureGuidanceCmd c2 = guidance_step(make_state(0.002, 1.0, u, V_M, V_T), pr, m);
        ok(c2.mode == PureGuidanceMode::HOLD, "R=1 < 5 → HOLD");
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
            const PureGuidanceCmd ca = guidance_step(make_state(0.0, R, u, V_M, V_T), a, ma);
            const PureGuidanceCmd cb = guidance_step(make_state(0.0, R, u, V_M, V_T), b, mb);
            if ((ca.mode == PureGuidanceMode::HOLD) != (cb.mode == PureGuidanceMode::HOLD)) {
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
        const PureGuidanceCmd ct = guidance_step(st, pt, mt);
        ok(ct.mode != PureGuidanceMode::HOLD, "Vc=0：仅 t_go 基准 → 不冻结（t_go 发散）");

        GuidanceParams ph = pt;
        ph.freeze_criterion = FreezeCriterion::BY_TGO_OR_RANGE;
        ph.R_freeze         = 5.0;
        GuidanceMemory mh;
        const PureGuidanceCmd ch = guidance_step(st, ph, mh);
        ok(ch.mode == PureGuidanceMode::HOLD, "Vc=0：R 兜底 → 冻结（R=3 < 5）");
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
// 配套：适配层侧还有一套更完整的自检实现（`src/guidance/adapter/include/state_check.hpp`，
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
        const PureGuidanceCmd c = guidance_step(st, p, m);

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
        const PureGuidanceCmd c = guidance_step(st, pm, m);
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
        const PureGuidanceCmd c2 = guidance_step(st2, p, m2);
        ok(std::isfinite(norm(c2.a_cmd)), "末端 R→0：a_cmd 仍有限（未发散）");
        ok(norm(c2.a_cmd) <= p.a_max + 1e-9, "  末端 |a_cmd| ≤ a_max");
        ok(c2.t_go >= 0.0, "  t_go ≥ 0");

        // (c) 目标丢失：SEARCH + 零指令
        GuidanceState st3 = make_state(1.0, 100.0, Vec3(1, 0, 0), Vec3(20, 0, 0), Vec3(0, 0, 0));
        st3.valid = false;
        GuidanceMemory m3;
        const PureGuidanceCmd c3 = guidance_step(st3, p, m3);
        ok(c3.mode == PureGuidanceMode::SEARCH, "目标丢失：mode = SEARCH");
        near(norm(c3.a_cmd), 0.0, 0.0, "  零指令");
        ok(std::isfinite(c3.a_cmd_norm), "  a_cmd_norm 有限（不是 NaN）");
    }

    // --- 15.3 时间戳单调性：制导不动时间，只透传；契约要求上游单调 ---
    {
        GuidanceState st = make_state(1.234, 100.0, Vec3(1, 0, 0), Vec3(20, 0, 0), Vec3(0, 0, 0));
        GuidanceMemory m;
        const PureGuidanceCmd c = guidance_step(st, p, m);
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
        const PureGuidanceCmd cd = guidance_step(s2, pd, md);
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
        const PureGuidanceCmd c = guidance_step(st, p, m);
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
//  16. ★ 机载主循环外壳 GuidanceLoop
// ============================================================
//
// 它把"纯函数内核"包成有状态、可替换数据源的外壳。本组验证五件事：
//   ① 像素 → 相机系 → 机体 → 世界 的换算正确（用已知几何反推像素）
//   ② 正常制导：PN 指令方向指向目标所在侧、量级合理
//   ③ 目标丢失 / 脏数据 / 数据过期 的三种降级行为
//   ④ "没有新目标帧"时仍然重发（PX4 offboard 要求 setpoint 持续到达）
//   ⑤ 统计计数
namespace {

struct LoopStubIns : app::IInsSource {
    app::InsSample next;
    bool pending = false;
    bool poll(app::InsSample &o) override {
        if (!pending) return false;
        o = next;
        pending = false;
        return true;
    }
    bool alive() const override { return true; }
};

struct LoopStubTgt : app::ITargetSource {
    app::TargetSample next;
    bool pending = false;
    bool poll(app::TargetSample &o) override {
        if (!pending) return false;
        o = next;
        pending = false;
        return true;
    }
    bool alive() const override { return true; }
};

struct LoopStubSink : app::ICommandSink {
    app::GuidanceOutput last;
    int sends = 0;
    bool send(const app::GuidanceOutput &o) override {
        last = o;
        ++sends;
        return true;
    }
    bool alive() const override { return true; }
};

// 相机：针孔 f=500、主点 (320,240)；外参 = 相机(x右,y下,z前) → 机体 FRD(x前,y右,z下)
app::CameraModel loop_cam() {
    app::CameraModel c;
    c.fx = 500.0; c.fy = 500.0; c.cx = 320.0; c.cy = 240.0;
    const double R[9] = {0, 0, 1, 1, 0, 0, 0, 1, 0};   // 行主序
    for (int i = 0; i < 9; ++i) c.R_body_cam[i] = R[i];
    return c;
}

app::InsSample loop_ins(std::uint64_t us) {
    app::InsSample s;
    s.boot_us = us;
    s.p = Vec3(0.0, 0.0, -5.0);      // NED：z=-5 ⇒ 高度 5 m
    s.v = Vec3(10.0, 0.0, 0.0);      // 朝 +x（北）10 m/s
    s.q[0] = 1.0;                    // 单位四元数：机体与局部系对齐
    s.has_position = true;
    s.has_attitude = true;
    return s;
}

// 目标：世界视线朝 +x 偏 +y（右），tanθ 给定；像素由内参反算 ⇒ 顺带验证反投影
app::TargetSample loop_tgt(std::uint64_t us, double tan_theta, bool present = true) {
    app::TargetSample t;
    t.boot_us   = us;
    t.u_px      = 320.0 + 500.0 * tan_theta;
    t.v_px      = 240.0;
    t.present   = present;
    t.has_range = true;
    t.range_m   = 50.0;
    t.has_v_T   = true;              // 仿真真值：目标静止 ⇒ 可用真值法算 ω
    t.v_T       = Vec3(0.0, 0.0, 0.0);
    return t;
}

GuidanceParams loop_params() {
    GuidanceParams p;
    p.law = GuidanceLaw::PPN;
    p.N = 4.0;
    p.a_max = 30.0;
    p.los_rate_source = LosRateSource::FROM_RELATIVE_VELOCITY;   // 仿真真值法
    return p;
}

}  // namespace

static void test_guidance_loop() {
    section("16. 机载主循环（GuidanceLoop）");

    // --- 16.1 init 校验：相机模型必须有（目标数据是像素）---
    {
        LoopStubIns ins; LoopStubTgt tgt; LoopStubSink sink;
        app::LoopConfig cfg;                       // cam 未填
        app::GuidanceLoop loop(loop_params(), cfg, ins, tgt, sink);
        ok(!loop.init(), "无相机模型 ⇒ init 失败");

        app::LoopConfig cfg2;
        cfg2.cam = loop_cam();
        cfg2.loop_hz = 50;
        app::GuidanceLoop loop2(loop_params(), cfg2, ins, tgt, sink);
        ok(loop2.init(), "相机模型有效 + 50 Hz ⇒ init 成功");
    }

    // --- 16.2 正常制导：像素反投影 + PPN 方向 ---
    {
        LoopStubIns ins; LoopStubTgt tgt; LoopStubSink sink;
        app::LoopConfig cfg; cfg.cam = loop_cam(); cfg.loop_hz = 50;
        app::GuidanceLoop loop(loop_params(), cfg, ins, tgt, sink);
        loop.init();

        ins.next = loop_ins(1000000ull); ins.pending = true;
        tgt.next = loop_tgt(1000000ull, 0.2); tgt.pending = true;   // 右偏 11.3°

        app::GuidanceOutput out;
        const app::LoopStatus s = loop.run_once(&out);
        ok(s == app::LoopStatus::kOk, "run_once 返回 kOk");
        ok(out.health == app::InputHealth::kOk, "体检通过");
        ok(out.mode == PureGuidanceMode::PN, "mode = PN");
        ok(out.valid, "valid = true");
        ok(out.ctrl == app::ControlMode::kAcceleration, "ctrl = 加速度");
        ok(out.a_cmd_ned.y > 0.5, "加速度指向 +y（目标所在侧）");
        near(out.a_cmd_ned.x, 0.0, 1e-9, "x 分量≈0（PPN 只出横向）");
        near(out.a_cmd_ned.z, 0.0, 1e-9, "z 分量≈0（共面）");
        ok(out.R > 40.0 && out.R < 60.0, "R 在 50 m 附近（像素反投影出的距离）");
        std::printf("        a_ned=(%.4f, %.4f, %.4f)  R=%.2f  Vc=%.2f\n",
                    out.a_cmd_ned.x, out.a_cmd_ned.y, out.a_cmd_ned.z, out.R, out.Vc);
        ok(sink.sends == 1, "发出 1 条指令");
    }

    // --- 16.3 共线目标：ω=0 ⇒ 零指令（与 §3 的零速/共线退化一致）---
    {
        LoopStubIns ins; LoopStubTgt tgt; LoopStubSink sink;
        app::LoopConfig cfg; cfg.cam = loop_cam();
        app::GuidanceLoop loop(loop_params(), cfg, ins, tgt, sink);
        loop.init();
        ins.next = loop_ins(1000000ull); ins.pending = true;
        tgt.next = loop_tgt(1000000ull, 0.0); tgt.pending = true;
        app::GuidanceOutput out;
        loop.run_once(&out);
        ok(out.a_cmd_norm < 1e-9, "正前方共线 ⇒ 指令为零");
    }

    // --- 16.4 目标丢失：SEARCH + 零指令 ---
    {
        LoopStubIns ins; LoopStubTgt tgt; LoopStubSink sink;
        app::LoopConfig cfg; cfg.cam = loop_cam();
        app::GuidanceLoop loop(loop_params(), cfg, ins, tgt, sink);
        loop.init();
        ins.next = loop_ins(1000000ull); ins.pending = true;
        tgt.next = loop_tgt(1000000ull, 0.2, /*present=*/false); tgt.pending = true;
        app::GuidanceOutput out;
        loop.run_once(&out);
        ok(out.mode == PureGuidanceMode::SEARCH, "present=false ⇒ mode = SEARCH");
        ok(!out.valid, "  valid = false");
        ok(out.a_cmd_norm == 0.0, "  SEARCH 输出零指令");
    }

    // --- 16.5 脏数据（NaN 像素）→ 限时保持上一拍 ---
    {
        LoopStubIns ins; LoopStubTgt tgt; LoopStubSink sink;
        app::LoopConfig cfg; cfg.cam = loop_cam();
        cfg.on_unhealthy = app::LoopConfig::OnUnhealthy::kHoldLastCommand;
        cfg.hold_last_max_s = 0.2;
        app::GuidanceLoop loop(loop_params(), cfg, ins, tgt, sink);
        loop.init();

        ins.next = loop_ins(1000000ull); ins.pending = true;
        tgt.next = loop_tgt(1000000ull, 0.2); tgt.pending = true;
        app::GuidanceOutput good, bad;
        loop.run_once(&good);

        ins.next = loop_ins(1020000ull); ins.pending = true;
        tgt.next = loop_tgt(1020000ull, 0.2);
        tgt.next.u_px = std::nan("");
        tgt.pending = true;
        loop.run_once(&bad);

        ok(bad.health != app::InputHealth::kOk, "脏数据被体检抓到");
        ok(bad.ctrl == app::ControlMode::kAcceleration, "限时内仍走加速度通道（保持）");
        near(bad.a_cmd_ned.y, good.a_cmd_ned.y, 1e-12, "保持的是上一拍指令");
        ok(loop.stats().unhealthy == 1, "unhealthy 计数 +1");
    }

    // --- 16.6 脏数据持续超时 → 零速度悬停（★ 保持窗口从"最后一条好指令"起算）---
    {
        LoopStubIns ins; LoopStubTgt tgt; LoopStubSink sink;
        app::LoopConfig cfg; cfg.cam = loop_cam();
        cfg.on_unhealthy = app::LoopConfig::OnUnhealthy::kHoldLastCommand;
        cfg.hold_last_max_s = 0.05;
        app::GuidanceLoop loop(loop_params(), cfg, ins, tgt, sink);
        loop.init();

        ins.next = loop_ins(1000000ull); ins.pending = true;
        tgt.next = loop_tgt(1000000ull, 0.2); tgt.pending = true;
        app::GuidanceOutput o;
        loop.run_once(&o);

        std::this_thread::sleep_for(std::chrono::milliseconds(80));   // 超过 0.05 s

        ins.next = loop_ins(1080000ull); ins.pending = true;
        tgt.next = loop_tgt(1080000ull, 0.2);
        tgt.next.u_px = std::nan("");
        tgt.pending = true;
        loop.run_once(&o);

        ok(o.ctrl == app::ControlMode::kVelocity, "超时后转速度通道");
        ok(o.v_cmd_ned.x == 0.0 && o.v_cmd_ned.y == 0.0 && o.v_cmd_ned.z == 0.0,
           "  速度指令为零（真悬停，不是零加速度）");
        near(o.a_cmd_norm, 0.0, 1e-12, "  加速度归零");
        ok(o.hold_yaw, "  不干预航向");
        ok(loop.stats().safe_hover == 1, "  safe_hover 计数 +1");
    }

    // --- 16.7 策略直接 kSafeHover：脏数据立刻悬停 ---
    {
        LoopStubIns ins; LoopStubTgt tgt; LoopStubSink sink;
        app::LoopConfig cfg; cfg.cam = loop_cam();
        cfg.on_unhealthy = app::LoopConfig::OnUnhealthy::kSafeHover;
        app::GuidanceLoop loop(loop_params(), cfg, ins, tgt, sink);
        loop.init();
        ins.next = loop_ins(1000000ull); ins.pending = true;
        tgt.next = loop_tgt(1000000ull, 0.2);
        tgt.next.u_px = std::nan("");
        tgt.pending = true;
        app::GuidanceOutput o;
        loop.run_once(&o);
        ok(o.ctrl == app::ControlMode::kVelocity, "ctrl = 速度");
        ok(o.v_cmd_ned.x == 0.0, "零速度");
    }

    // --- 16.8 数据过期（目标帧落后 1 s > 0.1 s）---
    {
        LoopStubIns ins; LoopStubTgt tgt; LoopStubSink sink;
        app::LoopConfig cfg; cfg.cam = loop_cam(); cfg.max_sample_age_s = 0.10;
        app::GuidanceLoop loop(loop_params(), cfg, ins, tgt, sink);
        loop.init();
        ins.next = loop_ins(3000000ull); ins.pending = true;          // ins 3.0 s
        tgt.next = loop_tgt(2000000ull, 0.2); tgt.pending = true;     // tgt 2.0 s
        app::GuidanceOutput o;
        loop.run_once(&o);
        ok(o.health == app::InputHealth::kStale, "health = kStale");
        ok(o.ctrl == app::ControlMode::kVelocity, "过期 ⇒ 降级（首拍无历史 ⇒ 零速度悬停）");
    }

    // --- 16.9 没有新目标帧：仍然重发（PX4 offboard 需要持续到达）---
    {
        LoopStubIns ins; LoopStubTgt tgt; LoopStubSink sink;
        app::LoopConfig cfg; cfg.cam = loop_cam();
        app::GuidanceLoop loop(loop_params(), cfg, ins, tgt, sink);
        loop.init();
        ins.next = loop_ins(1000000ull); ins.pending = true;
        tgt.next = loop_tgt(1000000ull, 0.2); tgt.pending = true;
        app::GuidanceOutput first, resent;
        loop.run_once(&first);
        const int sends_before = sink.sends;

        ins.next = loop_ins(1020000ull); ins.pending = true;          // 只有飞控帧
        loop.run_once(&resent);
        ok(sink.sends == sends_before + 1, "没有新的目标帧也照样发");
        near(resent.a_cmd_ned.y, first.a_cmd_ned.y, 1e-12, "  重发的是上一拍指令");
        ok(loop.stats().ticks == 2, "  ticks 计数正确");
    }

    // --- 16.10 统计计数 ---
    {
        LoopStubIns ins; LoopStubTgt tgt; LoopStubSink sink;
        app::LoopConfig cfg; cfg.cam = loop_cam();
        app::GuidanceLoop loop(loop_params(), cfg, ins, tgt, sink);
        loop.init();
        for (int k = 0; k < 5; ++k) {
            ins.next = loop_ins(1000000ull + 20000ull * k); ins.pending = true;
            tgt.next = loop_tgt(1000000ull + 20000ull * k, 0.2); tgt.pending = true;
            app::GuidanceOutput o;
            loop.run_once(&o);
        }
        const app::LoopStats &st = loop.stats();
        ok(st.ticks == 5, "ticks = 5");
        ok(st.ins_frames == 5, "ins_frames = 5");
        ok(st.tgt_frames == 5, "tgt_frames = 5");
        ok(st.sent == 5, "sent = 5");
        ok(st.sink_errors == 0, "无发送错误");
    }
}

// ============================================================
//  17. ★ 端到端闭环回归（合成世界）
// ============================================================
//
// 为什么需要这一组：§1~§16 都是**分段**验证（每个函数各自拿桩数据测）。
// 分段全绿 ≠ 整条链没坏 —— 2026-10-20 就踩过：HOLD 保持窗口的起算点写错，
// 分段测试全过，是行为测试才抓到的。
//
// 本组把"像素 → 反投影 → 制导核 → 出口换算 → 点质量积分 → 再拍成像素"整条链
// 跑一整段交会，并断言**闭环结果**（最近距离 / 饱和 / HOLD / 每拍发送）。
// 任何一环被悄悄改坏（矩阵乘法、单位化、饱和、重发、时间戳），这里都会红。
namespace {

struct FirstOutCtx {
    app::GuidanceOutput first{};
    int n = 0;
};

void first_out_hook(void *user, int, const app::SyntheticWorld &,
                    const app::GuidanceOutput &o) {
    auto *ctx = static_cast<FirstOutCtx *>(user);
    if (ctx->n++ == 0) ctx->first = o;
}

struct CollectSink : app::ICommandSink {
    std::vector<app::GuidanceOutput> outs;
    bool send(const app::GuidanceOutput &o) override {
        outs.push_back(o);
        return true;
    }
    bool alive() const override { return true; }
};

}  // namespace

static void test_closed_loop_regression() {
    section("17. 端到端闭环回归（合成世界）");

    app::SceneConfig sc;   // 默认场景：5 m 高、朝 +x 8 m/s、靶 40 m 前右偏 3 m
    sc.dt = 0.02;          // 50 Hz

    app::SyntheticWorld world(app::demo_camera(), sc);

    GuidanceParams p;
    p.law   = GuidanceLaw::PPN;
    p.N     = 4.0;
    p.a_max = 10.0;
    p.los_rate_source = LosRateSource::FROM_RELATIVE_VELOCITY;

    app::LoopConfig cfg;
    cfg.cam              = app::demo_camera();
    cfg.loop_hz          = 50;
    cfg.max_sample_age_s = 0.2;   // 合成世界每拍都更新

    app::GuidanceLoop loop(p, cfg, world, world, world);
    ok(loop.init(), "§17 init 成功");

    FirstOutCtx ctx;
    const app::ClosedLoopReport rep =
        app::run_synthetic_closed_loop(world, loop, 400, first_out_hook, &ctx);

    // ---- 闭环结果 ----
    ok(rep.ticks > 200, "跑够拍数（> 200 拍）");
    ok(rep.min_r <= 0.10, "最近距离 ≤ 0.10 m（真的打到靶；2026-10-20 实测 0.033）");
    ok(rep.min_r > 0.0, "最近距离 > 0（不是退化成一出发就在靶点）");
    ok(rep.min_r_tick > 100, "最近距离出现在交会后段（> 100 拍）");
    ok(rep.stop == app::LoopStatus::kOk, "跑满拍数、没被异常状态打断");
    ok(loop.stats().tgt_frames < loop.stats().ticks,
       "后段目标飞越出画：目标帧数 < 拍数（否则说明像素一直有效，几何写错了）");

    // ---- 指令链路（算法一改就红）----
    ok(ctx.n > 200, "每拍都产出了指令（hook 被回调）");
    ok(ctx.first.mode == PureGuidanceMode::PN, "首拍 mode = PN");
    ok(ctx.first.a_cmd_ned.y > 0.3, "首拍加速度指向靶侧（+y）");
    // ★ PPN 的加速度**垂直于速度**（不是垂直于视线）：速度沿 NED +x ⇒ 沿 x 的分量必须为 0。
    //   2026-10-21 实测 a_ned=(0.000000, 0.481135, 0.000000)：
    //   4 × |v_M|(8) × |ω|(0.015035) = 0.4811 ⇒ 与被测实现逐位吻合。
    near(ctx.first.a_cmd_ned.x, 0.0, 1e-12, "首拍沿速度方向的分量为 0（PPN 垂直于速度）");
    near(ctx.first.a_cmd_ned.z, 0.0, 1e-12, "首拍 z = 0（共面，不出纵向加速度）");

    // ---- PX4 的硬要求：指令必须每拍都发（不是"只在有新数据时才发"）----
    const app::LoopStats &s = loop.stats();
    ok(s.sent == s.ticks, "每拍都发出指令（sent == ticks）");
    ok(s.sink_errors == 0, "无发送失败");
    ok(s.saturated == 0, "全程不饱和（a_max=10 够用）");
    ok(world.hold() > 0, "末端进入 HOLD（t_go 变小后冻结）");
}

// ============================================================
//  18. ★ CSV 录制 → 回放（黑匣子）
// ============================================================
//
// 验证三件事：
//   ① 录制：装饰器包在数据源外面，主循环一行不用改就能落盘（行数 == 拍数）；
//   ② 格式：`%.17g` 存盘 ⇒ 读回来**逐位一致**（不丢精度）；
//   ③ 回放：同一份输入喂回**同一个 loop**，输出必须与录制时逐位相同。
//      ⇒ 以后改坏了任何一步（反投影、饱和、重发），拿历史数据一放就知道。
namespace {

// CSV 路径放在当前工作目录（测试从 bin/test 运行），结束会删掉
const char *kRoundTripCsv = "test_guidance_roundtrip.csv";

GuidanceParams csv_params() {
    GuidanceParams p;
    p.law   = GuidanceLaw::PPN;
    p.N     = 4.0;
    p.a_max = 10.0;
    p.los_rate_source = LosRateSource::FROM_RELATIVE_VELOCITY;
    return p;
}

app::LoopConfig csv_config() {
    app::LoopConfig cfg;
    cfg.cam              = app::demo_camera();
    cfg.loop_hz          = 50;
    cfg.max_sample_age_s = 0.2;
    return cfg;
}

}  // namespace

static void test_csv_roundtrip() {
    section("18. CSV 录制 → 回放（黑匣子）");

    const int n = 20;
    CollectSink rec_outs;

    // ---- ① 录制：跑 n 拍，输入落盘，同时收集输出 ----
    {
        app::SceneConfig sc;
        sc.dt = 0.02;
        app::SyntheticWorld world(app::demo_camera(), sc);

        app::InputRecorder rec;
        ok(rec.open(kRoundTripCsv), "§18 打开录制文件");
        if (!rec.ok()) return;   // 打不开就没法继续（不给假绿）

        app::RecordingInsSource    rins(world, rec);
        app::RecordingTargetSource rtgt(world, rec);
        app::TeeSink               tee(world, rec_outs);   // 执行器 + 收集

        app::GuidanceLoop loop(csv_params(), csv_config(), rins, rtgt, tee);
        ok(loop.init(), "§18 录制 loop init");

        for (int k = 0; k < n; ++k) {
            world.step();
            loop.run_once(nullptr);
        }
        ok(rec.rows() == static_cast<std::uint64_t>(n), "录制行数 = 拍数");
        ok(rec_outs.outs.size() == static_cast<std::size_t>(n), "收集到 n 条指令");
        rec.close();   // ★ 必须 close 才会 flush，回读才看得到内容
        ok(rec.ok(), "录制无写错误");
    }

    // ---- ② 回读：格式与字段 ----
    {
        app::CsvReader rd;
        ok(rd.open(kRoundTripCsv), "§18 打开回读文件");
        app::CsvRow row;
        ok(rd.next(row), "读到第一行");
        ok(row.tick == 0, "首行 tick = 0");
        ok(row.ins_new, "首行 ins_new = 1（合成世界每拍都有新帧）");
        ok(row.tgt_new, "首行 tgt_new = 1");
        ok(row.ins.boot_us == 1020000ull, "首行时间戳 = t0 + dt = 1.02 s（飞控 boot µs）");
        near(row.ins.p.x, 0.16, 1e-12, "首行 p_n = 8 m/s × 0.02 s = 0.16 m");
        near(row.ins.p.z, -5.0, 1e-12, "首行高度不变（z=-5）");
        near(row.ins.q[0], 1.0, 1e-15, "首行 qw = 1（机体与局部系对齐）");
        ok(row.tgt.present, "首行 present = 1");
        ok(row.tgt.has_range, "首行 has_range = 1");
        ok(row.tgt.range_m > 39.0 && row.tgt.range_m < 41.0, "首行距离 ≈ 40 m");
        ok(rd.next(row), "读到第二行");
        ok(row.tick == 1, "第二行 tick = 1");
        ok(rd.rows() == 2, "已解析 2 行");
        while (rd.next(row)) {}
        ok(rd.rows() == static_cast<std::uint64_t>(n), "总行数 = 拍数");
        ok(rd.error() == nullptr, "无解析错误");
    }

    // ---- ③ 回放：同一份输入 → 同一个 loop ⇒ 输出必须逐位一致 ----
    {
        app::CsvReader rd;
        ok(rd.open(kRoundTripCsv), "§18 重新打开回放文件");
        app::CsvReplaySource src(rd);

        CollectSink replay_outs;
        app::GuidanceLoop loop(csv_params(), csv_config(), src, src, replay_outs);
        ok(loop.init(), "§18 回放 loop init");

        for (int k = 0; k < n && !src.done(); ++k) loop.run_once(nullptr);

        ok(src.rows() == static_cast<std::uint64_t>(n), "回放读完 n 行");
        ok(replay_outs.outs.size() == rec_outs.outs.size(), "指令条数一致");

        bool same = (replay_outs.outs.size() == rec_outs.outs.size());
        int  first_bad = -1;
        if (same) {
            for (std::size_t i = 0; i < replay_outs.outs.size(); ++i) {
                const app::GuidanceOutput &a = rec_outs.outs[i];
                const app::GuidanceOutput &b = replay_outs.outs[i];
                if (a.boot_us != b.boot_us || a.ctrl != b.ctrl || a.mode != b.mode ||
                    a.valid != b.valid || a.sat != b.sat || a.health != b.health ||
                    a.a_cmd_ned.x != b.a_cmd_ned.x || a.a_cmd_ned.y != b.a_cmd_ned.y ||
                    a.a_cmd_ned.z != b.a_cmd_ned.z || a.a_cmd_norm != b.a_cmd_norm ||
                    a.t_go != b.t_go || a.Vc != b.Vc || a.R != b.R ||
                    a.lam_dot != b.lam_dot) {
                    same = false;
                    first_bad = static_cast<int>(i);
                    break;
                }
            }
        }
        if (first_bad >= 0) std::printf("    首个不一致的指令序号：%d\n", first_bad);
        ok(same, "录制与回放的指令逐位一致（%.17g 存盘不丢精度）");
    }

    std::remove(kRoundTripCsv);
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
    test_guidance_loop();
    test_closed_loop_regression();
    test_csv_roundtrip();

    std::printf("\n----------------------------------------\n");
    std::printf("通过 %d, 失败 %d\n", g_pass, g_fail);
    std::printf("%s\n", (g_fail == 0) ? "ALL PASS" : "SOME TESTS FAILED");
    return (g_fail == 0) ? 0 : 1;
}
