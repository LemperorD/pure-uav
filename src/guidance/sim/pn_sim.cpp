// 3D 拦截数值仿真
//
// 设计见《仿真代码设计思路.md》。当前阶段：
//   · 目标：TargetModel（M2）—— STRAIGHT / BALLISTIC / DART / CIRCLE / SNAKE
//           飞镖（DART）= 重力 + 气动阻力，数值预积分（见 target.hpp）
//   · 传感器：三级逼真度 IDEAL / SEMI_REAL / FULL_REAL（见 sensor.hpp）
//   · 无人机：点质量，a_cmd 直接当实际加速度
//   · 积分：RK4
// 输出 CSV 给 Python 画图（见 plot_sim.py / plot_target.py）。
//
// 用法：
//   ./bin/pn_sim [输出CSV路径] [N] [law] [fidelity] [target]
//     law      : PPN(默认) / TPN / APN（也接受 0/1/2）
//     fidelity : IDEAL(默认) / SEMI / FULL
//     target   : dart(默认) / ballistic / straight / circle / snake

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "guidance.hpp"
#include "pn_law.hpp"
#include "sensor.hpp"
#include "target.hpp"

using namespace pure_uav;

// 端口无关的 π（不依赖 M_PI，避免 MinGW/严格 ANSI 下未定义）
static constexpr double kPi = 3.14159265358979323846;
static double deg2rad(double d) { return d * kPi / 180.0; }

// ============================================================
//  场景定义
// ============================================================
struct Scenario {
    double t_end       = 20.0;    // 时间上限(s)
    double dt_sim      = 0.001;   // 积分步长(s)
    double dt_guidance = 0.002;   // 制导/传感器周期(s)
    double hit_radius  = 0.1;     // 命中半径(m)

    Vec3 p_M0, v_M0;              // 无人机初值
    TargetConfig target;          // 目标模型配置（类型 + 初值 + 气动）

    GuidanceParams gp;
    SensorConfig   sensor;        // 传感器墙配置（含 fidelity）
};

// ============================================================
//  组装真值（仿真内部）；姿态用"机头朝速度方向"近似
// ============================================================
static TruthState make_truth(double t, const Vec3 &p_M, const Vec3 &v_M,
                             const TargetState &tgt) {
    TruthState tr;
    tr.t   = t;
    tr.p_M = p_M;
    tr.v_M = v_M;
    tr.p_T = tgt.p;
    tr.v_T = tgt.v;
    tr.a_T = tgt.a;
    quat_from_forward(v_M, tr.q);   // 相机随速度方向（点质量模型的姿态近似）
    return tr;
}

// ============================================================
//  RK4 推进（点质量；a 在步内视为常数 = 制导零阶保持）
// ============================================================
static void rk4_step(Vec3 &p, Vec3 &v, const Vec3 &a, double dt) {
    const Vec3 k1p = v,                    k1v = a;
    const Vec3 k2p = v + k1v * (dt * 0.5), k2v = a;
    const Vec3 k3p = v + k2v * (dt * 0.5), k3v = a;
    const Vec3 k4p = v + k3v * dt,         k4v = a;
    p += (k1p + k2p * 2.0 + k3p * 2.0 + k4p) * (dt / 6.0);
    v += (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6.0);
}

// ============================================================
//  单次仿真
// ============================================================
struct SimResult {
    bool   hit           = false;
    double miss_distance = 1e9;   // 最近距离(m)
    double t_miss        = 0.0;   // 最近距离时刻(s)
    double peak_accel    = 0.0;   // 峰值加速度(m/s²)
    double peak_lam_dot  = 0.0;   // 峰值视线角速率(rad/s)
    double lam_dot_end   = 0.0;   // 命中前一刻的 λ̇（看是否收敛）
    int    steps         = 0;
    int    sat_steps     = 0;
    // 传感器统计
    long long frames     = 0;
    long long dropped    = 0;
    long long lost       = 0;
};

static SimResult run_sim(const Scenario &s, const char *csv_path) {
    SimResult res;

    Vec3 p_M = s.p_M0, v_M = s.v_M0;
    GuidanceMemory mem;
    GuidanceCmd cmd;
    GuidanceState st;             // 传感器最近一次输出（两次更新间保持）
    Sensor sensor(s.sensor);      // 每个场景独立的传感器（自带 RNG / 延迟缓冲）
    TargetModel tmodel(s.target); // 目标真值模型（预生成轨迹，只依赖 t）

    double t      = 0.0;
    double next_g = 0.0;           // 下次制导更新时刻
    double min_R  = 1e9, t_min = 0.0;
    bool   have_min = false;
    bool   passed   = false;       // 是否已越过最近点
    double R_upd  = 0.0;           // 最近一次传感器更新时刻的真值 R（与 R_est 同时刻）

    FILE *fp = csv_path ? std::fopen(csv_path, "w") : nullptr;
    if (csv_path && !fp) {
        std::printf("[warn] 无法写入 CSV: %s\n", csv_path);
    }
    if (fp) {
        std::fprintf(fp, "t,pMx,pMy,pMz,vMx,vMy,vMz,pTx,pTy,pTz,vTx,vTy,vTz,"
                         "R,R_est,R_upd,Vc,lam_dot,acx,acy,acz,a_norm,sat,mode,t_go\n");
    }

    // 目标落地即仿真结束（其后轨迹无意义）
    const double t_span = std::min(s.t_end, tmodel.flight_time() + 0.05);

    while (t <= t_span) {
        const TargetState tgt = tmodel.state(t);
        const double      R   = norm(tgt.p - p_M);

        // 传感器与制导同周期采样（相机不会比制导更快）；两次更新之间零阶保持
        if (t + 1e-9 >= next_g) {
            const TruthState tr = make_truth(t, p_M, v_M, tgt);
            st  = sensor.observe(tr, t);
            cmd = guidance_step(st, s.gp, mem);
            R_upd = R;                 // 与 R_est 同一时刻的真值，便于算测距误差
            next_g += s.dt_guidance;
        }
        if (!have_min || R < min_R) { min_R = R; t_min = t; have_min = true; }
        if (R < s.hit_radius) res.hit = true;      // 只要穿过命中球就算命中（不提前退出）
        if (R > min_R + 0.1) passed = true;        // 距离已拉大 0.1 m → 越过最近点

        // 峰值指标只在 R > R_metric 时统计：
        //   末端 R→0 时 λ̇ 必然出现飞越尖峰（几何使然，非制导问题），
        //   若一并统计会把峰值|a|永远顶到 a_max，指标失去区分度。
        const double a_norm = norm(cmd.a_cmd);
        if (R > 1.0) {
            if (a_norm > res.peak_accel)        res.peak_accel = a_norm;
            if (cmd.lam_dot > res.peak_lam_dot) res.peak_lam_dot = cmd.lam_dot;
        }
        res.lam_dot_end = cmd.lam_dot;
        if (cmd.sat) res.sat_steps++;

        if (fp) {
            std::fprintf(fp,
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,%.4f,%.6f,%.4f,%.4f,%.4f,%.4f,%d,%d,%.4f\n",
                t,
                p_M.x, p_M.y, p_M.z, v_M.x, v_M.y, v_M.z,
                tgt.p.x, tgt.p.y, tgt.p.z, tgt.v.x, tgt.v.y, tgt.v.z,
                R, st.range, R_upd, cmd.Vc, cmd.lam_dot,
                cmd.a_cmd.x, cmd.a_cmd.y, cmd.a_cmd.z, cmd.a_cmd_norm,
                (int)cmd.sat, (int)cmd.mode, cmd.t_go);
        }

        // 终止判据：跑到（略过）最近点即停 —— 保留真实脱靶量，而不是"命中球半径"
        if (passed) break;

        rk4_step(p_M, v_M, cmd.a_cmd, s.dt_sim);
        t += s.dt_sim;
        res.steps++;
    }

    if (fp) std::fclose(fp);
    res.miss_distance = min_R;
    res.t_miss        = t_min;
    res.frames        = sensor.stats().frames;
    res.dropped       = sensor.stats().dropped;
    res.lost          = sensor.stats().lost;
    return res;
}

static const char *law_name(GuidanceLaw l) {
    switch (l) {
        case GuidanceLaw::PPN: return "PPN";
        case GuidanceLaw::TPN: return "TPN";
        case GuidanceLaw::APN: return "APN";
    }
    return "?";
}

// 命令行导引律解析：接受名字（PPN/TPN/APN，大小写不敏感）或数字（0/1/2）
static GuidanceLaw parse_law(const char *s) {
    if (!s) return GuidanceLaw::PPN;
    if (s[0] >= '0' && s[0] <= '9') {
        const int v = std::atoi(s);
        if (v == 0) return GuidanceLaw::PPN;
        if (v == 1) return GuidanceLaw::TPN;
        if (v == 2) return GuidanceLaw::APN;
        return GuidanceLaw::PPN;
    }
    auto eq = [](const char *a, const char *b) {
        for (; *a && *b; ++a, ++b) {
            char ca = (*a >= 'a' && *a <= 'z') ? char(*a - 32) : *a;
            char cb = (*b >= 'a' && *b <= 'z') ? char(*b - 32) : *b;
            if (ca != cb) return false;
        }
        return *a == '\0' && *b == '\0';
    };
    if (eq(s, "TPN")) return GuidanceLaw::TPN;
    if (eq(s, "APN")) return GuidanceLaw::APN;
    return GuidanceLaw::PPN;   // "PPN" 及无法识别者都回落到 PPN
}

// 命令行传感器逼真度解析：IDEAL / SEMI / FULL（大小写不敏感）
static SensorFidelity parse_fidelity(const char *s) {
    if (!s) return SensorFidelity::IDEAL;
    auto eq = [](const char *a, const char *b) {
        for (; *a && *b; ++a, ++b) {
            char ca = (*a >= 'a' && *a <= 'z') ? char(*a - 32) : *a;
            char cb = (*b >= 'a' && *b <= 'z') ? char(*b - 32) : *b;
            if (ca != cb) return false;
        }
        return *a == '\0' && *b == '\0';
    };
    if (eq(s, "SEMI") || eq(s, "SEMI_REAL")) return SensorFidelity::SEMI_REAL;
    if (eq(s, "FULL") || eq(s, "FULL_REAL")) return SensorFidelity::FULL_REAL;
    return SensorFidelity::IDEAL;
}

static const char *fidelity_name(SensorFidelity f) {
    switch (f) {
        case SensorFidelity::IDEAL:     return "IDEAL";
        case SensorFidelity::SEMI_REAL: return "SEMI_REAL";
        case SensorFidelity::FULL_REAL: return "FULL_REAL";
    }
    return "?";
}

// 传感器逼真度 ↔ ω 来源 的配对：
//   非理想传感器拿不到目标速度 v_T，制导必须改用视线差分法算 ω（真机唯一可行）。
static void pair_los_source(Scenario &s) {
    s.gp.los_rate_source = (s.sensor.fidelity == SensorFidelity::IDEAL)
                               ? LosRateSource::FROM_RELATIVE_VELOCITY
                               : LosRateSource::FROM_LOS_DIFFERENCE;
}

// 命令行目标类型解析：dart / ballistic / straight / circle / snake（大小写不敏感）
static TargetType parse_target(const char *s) {
    if (!s) return TargetType::DART;
    auto eq = [](const char *a, const char *b) {
        for (; *a && *b; ++a, ++b) {
            char ca = (*a >= 'a' && *a <= 'z') ? char(*a - 32) : *a;
            char cb = (*b >= 'a' && *b <= 'z') ? char(*b - 32) : *b;
            if (ca != cb) return false;
        }
        return *a == '\0' && *b == '\0';
    };
    if (eq(s, "STRAIGHT"))  return TargetType::STRAIGHT;
    if (eq(s, "BALLISTIC")) return TargetType::BALLISTIC;
    if (eq(s, "CIRCLE"))    return TargetType::CIRCLE;
    if (eq(s, "SNAKE"))     return TargetType::SNAKE;
    return TargetType::DART;   // "DART" 及无法识别者都回落到 DART
}

// ============================================================
//  main
// ============================================================
int main(int argc, char **argv) {
    // ============================================================
    //  场景：RM 尺度的迎头拦截飞镖
    //    飞镖从 (40, 0, 2.0) 朝 −x 发射，18 m/s，仰角 30°
    //      → 无风抛物线：射程 ≈ 2·vx·vz/g ≈ 28.6 m（正好一个半场），
    //        顶点 ≈ 6.1 m，落地 ≈ 2.0 s
    //    无人机从 (0, 0, 6) 沿 +x 飞 18 m/s
    //      → 约 1.2 s 后在 (21, 0, 5.8) 附近相遇（V_M/V_c ≈ 0.54）
    // ============================================================
    Scenario s;
    s.p_M0 = Vec3(0, 0, 6.0);
    s.v_M0 = Vec3(18, 0, 0);

    s.target.type         = TargetType::DART;
    s.target.p0           = Vec3(40, 0, 2.0);
    s.target.launch_speed = 18.0;
    s.target.launch_elev  = deg2rad(30.0);
    s.target.launch_azim  = kPi;              // 朝 −x（正对无人机）

    s.gp.law         = GuidanceLaw::PPN;
    // 默认 N=6 而非教科书的 4：本场景迎头 V_M/V_c ≈ 0.52，PPN 的等效增益
    // N_eff = N·(V_M/V_c)，N=4 时只有 2.08 —— 刚好压在稳定边界上，λ̇ 不衰减、
    // 指令全程饱和到 30 m/s²（见下方 N 扫描）。N=6 → N_eff≈3.1，才留出裕度。
    s.gp.N           = (argc >= 3) ? std::atof(argv[2]) : 6.0;
    s.gp.a_max       = 30.0;
    // 末端冻结阈值：0.3 s 在本场景（Vc ≈ 33 m/s）会在 **R ≈ 10 m** 处就冻结，
    // 那时 λ̇ 还没收敛（PPN N_eff≈2 收敛慢），等于"提前锁定一条脱靶弹道"。
    // 实测：0.30→PPN 全脱靶且饱和 201 步；0.15→PPN 在 N≥4 命中、仅饱和 4 步。
    // 结论：冻结阈值必须与交会速度/收敛速度匹配，高速交会要取小值。
    s.gp.t_go_freeze = 0.15;

    if (argc >= 4) s.gp.law = parse_law(argv[3]);
    s.sensor.fidelity = (argc >= 5) ? parse_fidelity(argv[4]) : SensorFidelity::IDEAL;
    if (argc >= 6) s.target.type = parse_target(argv[5]);
    pair_los_source(s);

    const char *csv = (argc >= 2) ? argv[1] : "pn_sim_log.csv";

    // ---- 目标模型摘要 ----
    {
        const TargetModel tm(s.target);
        const Vec3 v0 = tm.launch_velocity();
        std::printf("=== 目标模型 %s ===\n", target_type_name(s.target.type));
        std::printf("  发射点   : (%.1f, %.1f, %.1f) m\n",
                    s.target.p0.x, s.target.p0.y, s.target.p0.z);
        std::printf("  初速度   : (%.2f, %.2f, %.2f) m/s  |v0|=%.2f\n",
                    v0.x, v0.y, v0.z, norm(v0));
        if (s.target.type == TargetType::DART) {
            std::printf("  气动     : m=%.2f kg, Cd=%.2f, A=%.4f m², rho=%.3f\n",
                        s.target.mass, s.target.drag_cd, s.target.drag_area, s.target.rho);
        }
        std::printf("  飞行时长 : %.2f s %s\n", tm.flight_time(),
                    tm.hits_ground() ? "(落地)" : "(超时未落地)");
    }

    // ---- 气动阻力到底有多大影响（同发射条件，只把 Cd 置零对比）----
    if (s.target.type == TargetType::DART) {
        TargetConfig tc0 = s.target;
        tc0.drag_cd = 0.0;
        const TargetModel tm0(tc0);          // 无阻力
        const TargetModel tmD(s.target);     // 有阻力
        const TargetState s0 = tm0.state(tm0.flight_time());
        const TargetState sD = tmD.state(tmD.flight_time());
        const double range0 = std::sqrt((s0.p.x - s.target.p0.x) * (s0.p.x - s.target.p0.x) +
                                        (s0.p.y - s.target.p0.y) * (s0.p.y - s.target.p0.y));
        const double rangeD = std::sqrt((sD.p.x - s.target.p0.x) * (sD.p.x - s.target.p0.x) +
                                        (sD.p.y - s.target.p0.y) * (sD.p.y - s.target.p0.y));
        std::printf("\n=== 气动阻力影响（同发射条件，仅切换 Cd）===\n");
        std::printf("  %-14s %-14s %-14s %-10s\n", "", "无阻力 Cd=0", "有阻力", "差异");
        std::printf("  %-14s %-14.2f %-14.2f %-10.1f%%\n", "飞行时长 (s)",
                    tm0.flight_time(), tmD.flight_time(),
                    100.0 * (tmD.flight_time() - tm0.flight_time()) / tm0.flight_time());
        std::printf("  %-14s %-14.2f %-14.2f %-10.1f%%\n", "水平射程 (m)",
                    range0, rangeD, 100.0 * (rangeD - range0) / range0);
        std::printf("  %-14s %-14.2f %-14.2f %-10.1f%%\n", "落点速度 (m/s)",
                    norm(s0.v), norm(sD.v), 100.0 * (norm(sD.v) - norm(s0.v)) / norm(s0.v));
        std::printf("  （Cd=%.2f, m=%.2f kg, A=%.4f m² → k=½ρCdA/m=%.4f 1/m；"
                    "v=18 m/s 时阻力 ≈ %.2f m/s² ≈ %.0f%% g）\n",
                    s.target.drag_cd, s.target.mass, s.target.drag_area,
                    0.5 * s.target.rho * s.target.drag_cd * s.target.drag_area / s.target.mass,
                    0.5 * s.target.rho * s.target.drag_cd * s.target.drag_area / s.target.mass * 18.0 * 18.0,
                    100.0 * 0.5 * s.target.rho * s.target.drag_cd * s.target.drag_area /
                        s.target.mass * 18.0 * 18.0 / 9.81);
    }

    // ---- 主场景 ----
    const SimResult r = run_sim(s, csv);
    std::printf("\n=== 主场景 (%s, N=%.1f, 传感器 %s) ===\n",
                law_name(s.gp.law), s.gp.N, fidelity_name(s.sensor.fidelity));
    std::printf("  命中         : %s  (命中球半径 %.2f m)\n", r.hit ? "是" : "否", s.hit_radius);
    std::printf("  最近距离     : %.3f m  (t=%.2f s)\n", r.miss_distance, r.t_miss);
    std::printf("  峰值加速度   : %.2f m/s^2 (%.2f g)   [R>1m，排除末端飞越尖峰]\n",
                r.peak_accel, r.peak_accel / 9.81);
    std::printf("  峰值 lam_dot : %.4f rad/s            [R>1m]\n", r.peak_lam_dot);
    std::printf("  饱和步数     : %d / %d\n", r.sat_steps, r.steps);
    std::printf("  CSV          : %s\n", csv);

    // ---- 目标模型对比：同一发射条件、同一导引律，只换目标动力学 ----
    //   看清"重力 / 阻力 / 机动"各自把拦截变成什么样。
    {
        std::printf("\n=== 目标模型对比（%s N=%.1f, 传感器 IDEAL）===\n",
                    law_name(s.gp.law), s.gp.N);
        std::printf("  %-11s %-9s %-12s %-8s %-10s %-9s\n",
                    "类型", "飞行(s)", "最近距离", "结果", "峰值|a|", "拦截(s)");

        struct Row { const char *name; TargetType t; };
        const Row rows[] = {
            {"STRAIGHT",  TargetType::STRAIGHT},
            {"BALLISTIC", TargetType::BALLISTIC},
            {"DART",      TargetType::DART},
            {"CIRCLE",    TargetType::CIRCLE},
            {"SNAKE",     TargetType::SNAKE},
        };
        for (const Row &row : rows) {
            Scenario sc  = s;
            sc.target.type = row.t;
            const TargetModel tm(sc.target);

            // 导出目标弹道（只依赖 t，与拦截过程无关）→ 供 plot_target.py 画图
            {
                char fn[96];
                std::snprintf(fn, sizeof(fn), "target_%s.csv", row.name);
                if (FILE *tf = std::fopen(fn, "w")) {
                    std::fprintf(tf, "t,px,py,pz,vx,vy,vz,ax,ay,az\n");
                    // 统一截到 2.5 s：STRAIGHT/CIRCLE 的 t_max 是 10 s，
                    // 全画出来会把弹道段压成一点。
                    const double t_dump = std::min(tm.flight_time(), 2.5);
                    const int    n  = 400;
                    const double dt = t_dump / n;
                    for (int i = 0; i <= n; ++i) {
                        const double    tt = i * dt;
                        const TargetState ts = tm.state(tt);
                        std::fprintf(tf, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                                     tt, ts.p.x, ts.p.y, ts.p.z,
                                     ts.v.x, ts.v.y, ts.v.z,
                                     ts.a.x, ts.a.y, ts.a.z);
                    }
                    std::fclose(tf);
                }
            }

            const SimResult rr = run_sim(sc, nullptr);
            std::printf("  %-11s %-9.2f %-12.3f %-8s %-10.2f %-9.2f\n",
                        row.name, tm.flight_time(), rr.miss_distance,
                        rr.hit ? "命中" : "脱靶", rr.peak_accel, rr.t_miss);
        }
    }

    // ---- 导引律对比：同一飞镖目标，PPN / TPN / APN ----
    //   飞镖是**受重力的抛物线目标**，其垂直速度一直在变（+vz0 → −），
    //   这正是 APN 的 a_T⊥ 补偿项要处理的情形。
    {
        std::printf("\n=== 导引律对比（飞镖目标, N=%.1f, 传感器 IDEAL）===\n", s.gp.N);
        std::printf("  %-6s %-12s %-8s %-10s %-12s %-8s\n",
                    "律", "最近距离", "结果", "峰值|a|", "峰值lam_dot", "饱和");
        const GuidanceLaw laws[] = {GuidanceLaw::PPN, GuidanceLaw::TPN, GuidanceLaw::APN};
        for (GuidanceLaw lw : laws) {
            Scenario sc = s;
            sc.gp.law = lw;
            const SimResult rr = run_sim(sc, nullptr);
            std::printf("  %-6s %-12.3f %-8s %-10.2f %-12.4f %-8d\n",
                        law_name(lw), rr.miss_distance, rr.hit ? "命中" : "脱靶",
                        rr.peak_accel, rr.peak_lam_dot, rr.sat_steps);
        }
    }

    // ---- 末端冻结判据对比：t_go 基准 vs R 基准 vs 两者取或 ----
    //   本场景 t=0 的 Vc ≈ 33 m/s，故等价关系 R_freeze ≈ Vc · t_go_freeze：
    //     0.30 s ↔ 10 m，0.15 s ↔ 5 m。
    //   验证"R 基准 ≡ t_go 基准"（同 Vc 下等价），并看固定 R 阈值对 Vc 的敏感性。
    {
        const TargetState ts0 = TargetModel(s.target).state(0.0);
        const Vec3   r0  = ts0.p - s.p_M0;
        const Vec3   vr0 = ts0.v - s.v_M0;
        const double R0  = norm(r0);
        const double Vc0 = (R0 > 1e-9) ? -dot(r0, vr0) / R0 : 0.0;

        std::printf("\n=== 末端冻结判据对比（%s, N=%.1f, 传感器 IDEAL）===\n",
                    law_name(s.gp.law), s.gp.N);
        std::printf("  场景 Vc ≈ %.1f m/s → 等价：R_freeze ≈ Vc · t_go_freeze\n", Vc0);
        std::printf("  %-24s %-12s %-8s %-10s %-8s\n",
                    "判据", "最近距离", "结果", "峰值|a|", "饱和");

        struct FRow { const char *name; FreezeCriterion c; double tgo; double Rf; };
        const FRow frows[] = {
            {"BY_TGO    t_go=0.30 s", FreezeCriterion::BY_TGO,          0.30,  0.0},
            {"BY_TGO    t_go=0.15 s", FreezeCriterion::BY_TGO,          0.15,  0.0},
            {"BY_RANGE  R=10.0 m",    FreezeCriterion::BY_RANGE,        0.0,  10.0},
            {"BY_RANGE  R=5.0 m",     FreezeCriterion::BY_RANGE,        0.0,   5.0},
            {"BY_TGO_OR_RANGE",       FreezeCriterion::BY_TGO_OR_RANGE, 0.15,  5.0},
        };
        for (const FRow &row : frows) {
            Scenario sc = s;
            sc.gp.freeze_criterion = row.c;
            if (row.tgo > 0.0) sc.gp.t_go_freeze = row.tgo;
            if (row.Rf  > 0.0) sc.gp.R_freeze    = row.Rf;
            const SimResult rr = run_sim(sc, nullptr);
            std::printf("  %-24s %-12.3f %-8s %-10.2f %-8d\n",
                        row.name, rr.miss_distance, rr.hit ? "命中" : "脱靶",
                        rr.peak_accel, rr.sat_steps);
        }
    }

    // ---- 传感器墙对比：同一场景、同一导引律，只换传感器逼真度 ----
    // 统一 100 Hz（贴近真实视觉帧率），横向可比。
    {
        const double dt_vis = 0.01;
        std::printf("\n=== 传感器墙对比（%s, N=%.1f, 制导 %.0f Hz）===\n",
                    law_name(s.gp.law), s.gp.N, 1.0 / dt_vis);
        std::printf("  像素噪声 %.2f px | 测距比例误差 %.0f%% | 延迟 %.0f ms | 丢帧率 %.0f%%\n",
                    s.sensor.pixel_noise_std, s.sensor.range_scale_err * 100.0,
                    s.sensor.latency_s * 1000.0, s.sensor.drop_rate * 100.0);
        std::printf("  %-10s %-12s %-8s %-10s %-8s %-8s\n",
                    "逼真度", "最近距离", "结果", "峰值|a|", "丢帧", "丢失");

        struct Row { const char *name; const char *csv; SensorFidelity f; };
        const Row rows[] = {
            {"IDEAL",     "sensor_ideal.csv", SensorFidelity::IDEAL},
            {"SEMI_REAL", "sensor_semi.csv",  SensorFidelity::SEMI_REAL},
            {"FULL_REAL", "sensor_full.csv",  SensorFidelity::FULL_REAL},
        };
        for (const Row &row : rows) {
            Scenario sc        = s;
            sc.dt_guidance     = dt_vis;
            sc.sensor.fidelity = row.f;
            pair_los_source(sc);
            const SimResult rr = run_sim(sc, row.csv);
            std::printf("  %-10s %-12.3f %-8s %-10.2f %-8lld %-8lld\n",
                        row.name, rr.miss_distance, rr.hit ? "命中" : "脱靶",
                        rr.peak_accel, rr.dropped, rr.lost);
        }
    }

    // ---- 像素噪声敏感性：噪声 → 精度 / 控制代价（找失效边界）----
    {
        const double dt_vis = 0.01;
        std::printf("\n=== 像素噪声敏感性（SEMI_REAL, %s N=%.1f）===\n",
                    law_name(s.gp.law), s.gp.N);
        std::printf("  %-12s %-12s %-8s %-10s\n", "像素噪声(px)", "最近距离", "结果", "峰值|a|");
        const double ns[] = {0.0, 0.1, 0.3, 0.5, 1.0, 2.0};
        for (double n : ns) {
            Scenario sc               = s;
            sc.dt_guidance            = dt_vis;
            sc.sensor.fidelity        = SensorFidelity::SEMI_REAL;
            sc.sensor.pixel_noise_std = n;
            pair_los_source(sc);
            const SimResult rr = run_sim(sc, nullptr);
            std::printf("  %-12.2f %-12.3f %-8s %-10.2f\n",
                        n, rr.miss_distance, rr.hit ? "命中" : "脱靶", rr.peak_accel);
        }
    }

    // ---- N 扫描：PPN vs TPN（看 PPN 的等效增益被 V_M/Vc 缩放的后果）----
    {
        // 用场景 t=0 的几何算实际接近速度（而非硬编码）
        const TargetState ts0 = TargetModel(s.target).state(0.0);
        const Vec3   r0  = ts0.p - s.p_M0;
        const Vec3   vr0 = ts0.v - s.v_M0;
        const double R0  = norm(r0);
        const double Vc0 = (R0 > 1e-9) ? -dot(r0, vr0) / R0 : 0.0;
        const double Vm0 = norm(s.v_M0);

        std::printf("\n=== N 扫描（最近距离 m；V_M/Vc = %.2f/%.2f = %.2f → PPN 需 N > %.1f，TPN 需 N > 2）===\n",
                    Vm0, Vc0, Vm0 / Vc0, 2.0 * Vc0 / Vm0);
        std::printf("  %-6s %-16s %-16s\n", "N", "PPN", "TPN");
        const double Ns[] = {2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0};
        for (double N : Ns) {
            Scenario sp  = s; sp.gp.law  = GuidanceLaw::PPN; sp.gp.N  = N;
            Scenario stp = s; stp.gp.law = GuidanceLaw::TPN; stp.gp.N = N;
            const SimResult rp = run_sim(sp,  nullptr);
            const SimResult rt = run_sim(stp, nullptr);
            char bp[32], bt[32];
            std::snprintf(bp, sizeof(bp), "%.3f %s", rp.miss_distance, rp.hit ? "命中" : "脱靶");
            std::snprintf(bt, sizeof(bt), "%.3f %s", rt.miss_distance, rt.hit ? "命中" : "脱靶");
            std::printf("  %-6.1f %-16s %-16s\n", N, bp, bt);
        }
    }

    return 0;
}
