// pure_guidance_shim.cpp
// =============================================================================
//  pure_uav 制导核 · C ABI 封装（阶段 B/C 仿真接入用）
//
//  职责：把 guidance_libs（TargetModel + Sensor + guidance_step）包成一层
//        “扁平 double 数组进出”的 C 接口，供 Python（ctypes）调用。
//
//  原则：
//    · 不跨边界传 C++ 类型 —— 只有 double* / int / void* 句柄；
//    · 制导、传感器、目标模型全部是仓库里的真代码，本文件**不含任何算法**；
//    · 所有矢量均为惯性系 ENU（与 guidance_libs 约定一致）。
//
//  编译（WSL / Linux，产物给 Python ctypes 用）：
//    g++ -O2 -std=c++17 -fPIC -shared \
//        -I <repo>/src/guidance/guidance_libs/include \
//        pure_guidance_shim.cpp \
//        <repo>/src/guidance/guidance_libs/src/guidance.cpp \
//        <repo>/src/guidance/guidance_libs/src/pn_law.cpp \
//        <repo>/src/guidance/guidance_libs/src/sensor.cpp \
//        <repo>/src/guidance/guidance_libs/src/target.cpp \
//        -o libpure_guidance.so
//
//  编译（Windows / MSYS2，本地自测用）：同上，去掉 -fPIC、改 -o *.dll，
//  并加 -static -static-libgcc -static-libstdc++（避免运行时 DLL 依赖）。
//
//  ---------------------------------------------------------------------------
//  数组布局（Python 侧必须与此完全一致）
//  ---------------------------------------------------------------------------
//  pgu_config_target  cfg[29]：
//    [0]      target_type  0=STRAIGHT 1=BALLISTIC 2=DART 3=CIRCLE 4=SNAKE
//    [1..3]   p0 (ENU, m)
//    [4..6]   v0 (ENU, m/s)            （launch_speed > 0 时忽略）
//    [7]      launch_speed (m/s)
//    [8]      launch_elev (rad)
//    [9]      launch_azim (rad)
//    [10]     mass (kg)                [DART]
//    [11]     drag_cd                  [DART]
//    [12]     drag_area (m^2)          [DART]
//    [13]     rho (kg/m^3)             [DART]
//    [14..16] gravity (ENU, m/s^2)
//    [17..19] wind (ENU, m/s)
//    [20..22] a_maneuver (ENU, m/s^2)
//    [23]     turn_rate (rad/s)        [CIRCLE]
//    [24]     snake_acc (m/s^2)        [SNAKE]
//    [25]     snake_freq (rad/s)       [SNAKE]
//    [26]     dt_table (s)
//    [27]     t_max (s)
//    [28]     ground_z (m)
//
//  pgu_config_guidance  cfg[7]：
//    [0] law            0=PPN 1=TPN 2=APN
//    [1] N              导航比
//    [2] a_max          过载上限 (m/s^2)
//    [3] freeze_crit    0=BY_TGO 1=BY_RANGE 2=BY_TGO_OR_RANGE
//    [4] t_go_freeze (s)
//    [5] R_freeze (m)
//    [6] los_rate_src   0=FROM_RELATIVE_VELOCITY 1=FROM_LOS_DIFFERENCE
//
//  pgu_config_sensor  cfg[15]：
//    [0] fidelity       0=IDEAL 1=SEMI_REAL 2=FULL_REAL
//    [1] pixel_noise_std   [2] range_scale_err   [3] range_bias
//    [4] latency_s         [5] drop_rate         [6] seed
//    [7..10] cam fx, fy, cx, cy
//    [11] width  [12] height
//    [13] target_size_m  [14] min_bbox_px
//
//  pgu_step(..., cmd_out[19], dbg_out[17])：
//    cmd_out:
//      [0]      t (s)
//      [1..3]   a_cmd (ENU, m/s^2)        ← 核心输出
//      [4]      |a_cmd|
//      [5]      sat (0/1)
//      [6]      valid (0/1)
//      [7]      t_go (s)
//      [8]      mode 0=PN 1=APN 2=HOLD 3=SEARCH
//      [9]      lam_dot
//      [10]     Vc
//      [11]     R（估计距离）
//      [12..14] r_hat
//      [15]     N
//      [16..18] a_cmd_unsat
//    dbg_out（真值，仅日志用）：
//      [0..2]   p_T  [3..5] v_T  [6..8] a_T
//      [9..11]  p_M  [12..14] v_M
//      [15]     R_true = |p_T - p_M|（无目标时 = −1）
//      [16]     sensor_valid (0/1)
//
//  pgu_target_state(h, t, out[9])：目标真值 {p,v,a}（ENU），供轨迹记录/度量用。
//  pgu_target_flight_time / pgu_target_hits_ground / pgu_target_launch_velocity
//  pgu_sensor_stats(h, out[3])：{frames, dropped, lost}
// =============================================================================

#include <cstdint>
#include <memory>

#include "guidance.hpp"
#include "sensor.hpp"
#include "target.hpp"

#if defined(_WIN32)
#define PGU_API __declspec(dllexport)
#else
#define PGU_API __attribute__((visibility("default")))
#endif

namespace {

using namespace pure_uav;

struct Ctx {
    std::unique_ptr<TargetModel> target;
    std::unique_ptr<Sensor>      sensor;
    GuidanceParams gp;
    GuidanceMemory mem;
};

inline int    as_int(double v) { return static_cast<int>(v); }
inline double as_num(bool b)   { return b ? 1.0 : 0.0; }

// SEARCH 分支输出（目标未出现 / 未配置）：零指令
void fill_search_cmd(double t, double* cmd_out) {
    for (int i = 0; i < 19; ++i) cmd_out[i] = 0.0;
    cmd_out[0] = t;
    cmd_out[8] = 3.0;   // mode = SEARCH
}

void fill_dbg(const Vec3 &pT, const Vec3 &vT, const Vec3 &aT,
              const Vec3 &pM, const Vec3 &vM, double R_true, bool sv,
              double* dbg_out) {
    dbg_out[0]  = pT.x; dbg_out[1]  = pT.y; dbg_out[2]  = pT.z;
    dbg_out[3]  = vT.x; dbg_out[4]  = vT.y; dbg_out[5]  = vT.z;
    dbg_out[6]  = aT.x; dbg_out[7]  = aT.y; dbg_out[8]  = aT.z;
    dbg_out[9]  = pM.x; dbg_out[10] = pM.y; dbg_out[11] = pM.z;
    dbg_out[12] = vM.x; dbg_out[13] = vM.y; dbg_out[14] = vM.z;
    dbg_out[15] = R_true;
    dbg_out[16] = as_num(sv);
}

} // namespace

extern "C" {

PGU_API int pgu_version() { return 1; }

PGU_API void* pgu_create() { return new Ctx(); }

PGU_API void pgu_destroy(void* h) { delete static_cast<Ctx*>(h); }

PGU_API void pgu_config_target(void* h, const double* cfg) {
    Ctx* c = static_cast<Ctx*>(h);
    TargetConfig tc;
    tc.type = static_cast<TargetType>(as_int(cfg[0]));
    tc.p0   = Vec3(cfg[1], cfg[2], cfg[3]);
    tc.v0   = Vec3(cfg[4], cfg[5], cfg[6]);
    tc.launch_speed = cfg[7];
    tc.launch_elev  = cfg[8];
    tc.launch_azim  = cfg[9];
    tc.mass      = cfg[10];
    tc.drag_cd   = cfg[11];
    tc.drag_area = cfg[12];
    tc.rho       = cfg[13];
    tc.gravity   = Vec3(cfg[14], cfg[15], cfg[16]);
    tc.wind      = Vec3(cfg[17], cfg[18], cfg[19]);
    tc.a_maneuver = Vec3(cfg[20], cfg[21], cfg[22]);
    tc.turn_rate  = cfg[23];
    tc.snake_acc  = cfg[24];
    tc.snake_freq = cfg[25];
    tc.dt_table   = cfg[26];
    tc.t_max      = cfg[27];
    tc.ground_z   = cfg[28];

    c->target = std::make_unique<TargetModel>(tc);
    c->mem    = GuidanceMemory{};   // 换目标 = 重新开始：清空跨周期记忆
}

PGU_API void pgu_config_guidance(void* h, const double* cfg) {
    Ctx* c = static_cast<Ctx*>(h);
    c->gp.law = static_cast<GuidanceLaw>(as_int(cfg[0]));
    c->gp.N   = cfg[1];
    c->gp.a_max = cfg[2];
    c->gp.freeze_criterion = static_cast<FreezeCriterion>(as_int(cfg[3]));
    c->gp.t_go_freeze = cfg[4];
    c->gp.R_freeze    = cfg[5];
    c->gp.los_rate_source = static_cast<LosRateSource>(as_int(cfg[6]));
}

PGU_API void pgu_config_sensor(void* h, const double* cfg) {
    Ctx* c = static_cast<Ctx*>(h);
    SensorConfig sc;
    sc.fidelity = static_cast<SensorFidelity>(as_int(cfg[0]));
    sc.pixel_noise_std = cfg[1];
    sc.range_scale_err = cfg[2];
    sc.range_bias      = cfg[3];
    sc.latency_s       = cfg[4];
    sc.drop_rate       = cfg[5];
    sc.seed            = static_cast<std::uint32_t>(as_int(cfg[6]));
    sc.cam.fx = cfg[7];
    sc.cam.fy = cfg[8];
    sc.cam.cx = cfg[9];
    sc.cam.cy = cfg[10];
    sc.cam.width  = as_int(cfg[11]);
    sc.cam.height = as_int(cfg[12]);
    sc.cam.target_size_m = cfg[13];
    sc.cam.min_bbox_px   = cfg[14];

    c->sensor = std::make_unique<Sensor>(sc);
}

PGU_API double pgu_target_flight_time(void* h) {
    Ctx* c = static_cast<Ctx*>(h);
    return c->target ? c->target->flight_time() : 0.0;
}

PGU_API int pgu_target_hits_ground(void* h) {
    Ctx* c = static_cast<Ctx*>(h);
    return (c->target && c->target->hits_ground()) ? 1 : 0;
}

PGU_API void pgu_target_launch_velocity(void* h, double* out3) {
    Ctx* c = static_cast<Ctx*>(h);
    const Vec3 v = c->target ? c->target->launch_velocity() : Vec3{};
    out3[0] = v.x; out3[1] = v.y; out3[2] = v.z;
}

PGU_API void pgu_target_state(void* h, double t_rel, double* out9) {
    Ctx* c = static_cast<Ctx*>(h);
    if (!c->target) {
        for (int i = 0; i < 9; ++i) out9[i] = 0.0;
        return;
    }
    const TargetState s = c->target->state(t_rel);
    out9[0] = s.p.x; out9[1] = s.p.y; out9[2] = s.p.z;
    out9[3] = s.v.x; out9[4] = s.v.y; out9[5] = s.v.z;
    out9[6] = s.a.x; out9[7] = s.a.y; out9[8] = s.a.z;
}

PGU_API void pgu_sensor_stats(void* h, double* out3) {
    Ctx* c = static_cast<Ctx*>(h);
    if (!c->sensor) { out3[0] = out3[1] = out3[2] = 0.0; return; }
    const SensorStats& st = c->sensor->stats();
    out3[0] = static_cast<double>(st.frames);
    out3[1] = static_cast<double>(st.dropped);
    out3[2] = static_cast<double>(st.lost);
}

// 单步：目标真值 → Sensor(observe) → guidance_step → 输出扁平数组。
//   t_rel          目标发射后的时间 (s)
//   target_present 0 = 目标未出现（制导进 SEARCH、输出零指令）
//   pM, vM         自身位置/速度（ENU）
//   q              姿态四元数 **[w,x,y,z]**（与 PX4 / MAVLink 一致；单位四元数 = [1,0,0,0]）
//                  IDEAL 传感器不消费；SEMI/FULL 会用它做世界→相机投影
PGU_API void pgu_step(void* h, double t_rel, double target_present,
                      const double* pM, const double* vM, const double* q,
                      double* cmd_out, double* dbg_out) {
    Ctx* c = static_cast<Ctx*>(h);

    TruthState tr;
    tr.t   = t_rel;
    tr.p_M = Vec3(pM[0], pM[1], pM[2]);
    tr.v_M = Vec3(vM[0], vM[1], vM[2]);
    tr.q[0] = q[0]; tr.q[1] = q[1]; tr.q[2] = q[2]; tr.q[3] = q[3];

    // 目标未出现（或未配置）：SEARCH 分支（零指令），并清掉跨周期记忆
    if (!c->target || !c->sensor || target_present < 0.5) {
        GuidanceState st;
        st.t     = t_rel;
        st.valid = false;
        st.p_M   = tr.p_M;
        st.v_M   = tr.v_M;
        const GuidanceCmd cmd = guidance_step(st, c->gp, c->mem);
        fill_search_cmd(cmd.t, cmd_out);
        fill_dbg(Vec3{}, Vec3{}, Vec3{}, tr.p_M, tr.v_M, -1.0, false, dbg_out);
        return;
    }

    const TargetState tgt = c->target->state(t_rel);
    tr.p_T = tgt.p;
    tr.v_T = tgt.v;
    tr.a_T = tgt.a;

    const GuidanceState st  = c->sensor->observe(tr, t_rel);
    const GuidanceCmd  cmd  = guidance_step(st, c->gp, c->mem);

    cmd_out[0]  = cmd.t;
    cmd_out[1]  = cmd.a_cmd.x;
    cmd_out[2]  = cmd.a_cmd.y;
    cmd_out[3]  = cmd.a_cmd.z;
    cmd_out[4]  = cmd.a_cmd_norm;
    cmd_out[5]  = as_num(cmd.sat);
    cmd_out[6]  = as_num(cmd.valid);
    cmd_out[7]  = cmd.t_go;
    cmd_out[8]  = static_cast<double>(cmd.mode);
    cmd_out[9]  = cmd.lam_dot;
    cmd_out[10] = cmd.Vc;
    cmd_out[11] = cmd.R;
    cmd_out[12] = cmd.r_hat.x;
    cmd_out[13] = cmd.r_hat.y;
    cmd_out[14] = cmd.r_hat.z;
    cmd_out[15] = cmd.N;
    cmd_out[16] = cmd.a_cmd_unsat.x;
    cmd_out[17] = cmd.a_cmd_unsat.y;
    cmd_out[18] = cmd.a_cmd_unsat.z;

    fill_dbg(tgt.p, tgt.v, tgt.a, tr.p_M, tr.v_M,
             norm(tgt.p - tr.p_M), st.valid, dbg_out);
}

} // extern "C"
