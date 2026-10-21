// =============================================================================
//  pure-uav 机载主程序（制导）
//
//  现阶段的目标：**把整条链接起来跑通**，而不是接真机。
//  所以这里用了一个"合成世界"（SyntheticWorld）代替真实数据源与执行器：
//
//      真值点质量积分（p, v）            ← 用上一拍指令积分（理想控制，无姿态环）
//            │
//            ├─► 相机投影：世界目标 → 像素 (u,v)   ← 真值 → 相机系 → 像素
//            │
//            └─► GuidanceLoop：像素 → 反投影 → 制导核 → NED 加速度
//                                   ↑                    │
//                                   └────────────────────┘
//
//  这条链里**除了"真值→像素"这一步是模拟的**，其余全是真实代码路径：
//    · 像素 → 世界视线：GuidanceLoop 内（与真机同一份）
//    · 制导核：guidance_step（纯函数，1439 条断言）
//    · 出口换算：cmd_accel_to_ned
//    · 降级策略 / 重发 / 统计：GuidanceLoop
//  ⇒ 等通信侧合并后，只需把 SyntheticWorld 换成
//    MavlinkInsSource + VisionTargetSource + MavlinkAccelSink 三个实现，
//    主程序这一段几乎不用改。
//
//  运行：./bin/main  [--hz=50] [--law=0|1|2] [--n=4] [--a-max=30] [--ticks=300]
// =============================================================================

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "guidance_loop.hpp"
#include "quat.hpp"

using namespace pure_uav;
using namespace pure_uav::app;

namespace {

// ---------------------------------------------------------------- 合成世界
//
// 一个类同时扮演三个角色（只为主程序演示方便）：
//   · IInsSource    : 给出真值 p / v / q
//   · ITargetSource : 把真值目标"拍"成像素 (u,v) + 距离
//   · ICommandSink  : 接收制导指令，供下一拍积分用
//
// ⚠ 积分是"理想点质量"：直接把指令当实际加速度，没有姿态环、没有阻力、
//   没有重力（所以 a_z=0 时高度不漂）。目的是验证**制导逻辑**，不是飞行动力学。
class SyntheticWorld final : public IInsSource, public ITargetSource, public ICommandSink {
public:
    SyntheticWorld(const CameraModel &cam, const Vec3 &p0, const Vec3 &v0,
                   const Vec3 &plate, double dt)
        : cam_(cam), p_(p0), v_(v0), plate_(plate), dt_(dt) {
        q_[0] = 1.0;                       // 机体与局部系对齐（q = 单位）
        t_us_ = 1000000ull;                // 从 1 s 起算（避免 0 被当成"未初始化"）
    }

    // ---- 每拍推进真值（main 在 run_once 之前调用）----
    void step() {
        // 用上一拍指令积分（理想控制）：v += a·dt, p += v·dt
        v_ = v_ + a_cmd_ * dt_;
        p_ = p_ + v_ * dt_;
        t_us_ += static_cast<std::uint64_t>(dt_ * 1e6);
        // 指令只在本拍有效（下一拍若没新指令就视为 0，模拟"指令断流"的极端情况）
        a_cmd_ = Vec3{};
    }

    // ---- IInsSource ----
    bool poll(InsSample &out) override {
        out.boot_us      = t_us_;
        out.p            = p_;
        out.v            = v_;
        for (int i = 0; i < 4; ++i) out.q[i] = q_[i];
        out.has_position = true;
        out.has_attitude = true;
        return true;                        // 合成世界每拍都有新帧
    }

    // ---- ITargetSource：真值目标 → 像素 ----
    bool poll(TargetSample &out) override {
        const Vec3 d_world = plate_ - p_;                 // 世界系视线（未归一化）
        const double R = norm(d_world);
        if (!(R > 1e-6)) return false;
        const Vec3 los_world = d_world / R;

        // 世界 → 机体（用 q 的逆）→ 相机系（用外参的逆）
        const Vec3 d_body = quat_rotate_vec_inv(q_, los_world);   // 机体 FRD
        const Vec3 d_cam = mat3_mul_transpose(cam_.R_body_cam, d_body);
        if (!(d_cam.z > 1e-6)) return false;               // 目标在相机后方
        // 针孔投影
        out.boot_us   = t_us_;
        out.u_px      = cam_.cx + cam_.fx * (d_cam.x / d_cam.z);
        out.v_px      = cam_.cy + cam_.fy * (d_cam.y / d_cam.z);
        out.has_box   = false;                             // 演示里不造框尺寸
        out.conf      = 1.0;
        out.class_id  = 0;                                 // 假定就是装甲板
        out.present   = true;
        out.has_range = true;                              // 真值测距（相当于激光/PnP）
        out.range_m   = R;
        out.has_v_T   = true;                              // 靶静止 ⇒ 可用真值法算 ω
        out.v_T       = Vec3(0.0, 0.0, 0.0);
        return true;
    }

    // ---- ICommandSink：接住制导指令（下一拍积分用）----
    bool send(const GuidanceOutput &o) override {
        ++sends_;
        last_ = o;
        if (o.ctrl == ControlMode::kAcceleration) a_cmd_ = o.a_cmd_ned;
        else                                      a_cmd_ = Vec3{};   // kVelocity：演示里当"零加速度"处理
        if (o.sat) ++sat_;
        if (o.mode == PureGuidanceMode::HOLD) ++hold_;
        if (o.mode == PureGuidanceMode::SEARCH) ++search_;
        return true;
    }

    bool alive() const override { return true; }

    // ---- 给 main 看的量 ----
    Vec3   pos() const { return p_; }
    Vec3   vel() const { return v_; }
    double true_range() const { return norm(plate_ - p_); }
    const GuidanceOutput &last() const { return last_; }
    int    sends() const { return sends_; }
    int    sat() const { return sat_; }
    int    hold() const { return hold_; }
    int    search() const { return search_; }

private:
    // R^T × v（外参是 相机→机体，投影时要用 机体→相机）
    static Vec3 mat3_mul_transpose(const double R[9], const Vec3 &v) {
        return Vec3(R[0] * v.x + R[3] * v.y + R[6] * v.z,
                    R[1] * v.x + R[4] * v.y + R[7] * v.z,
                    R[2] * v.x + R[5] * v.y + R[8] * v.z);
    }

    CameraModel cam_;
    Vec3   p_, v_, plate_;
    double q_[4] = {1, 0, 0, 0};
    double dt_;
    std::uint64_t t_us_;
    Vec3   a_cmd_{};          // 上一拍指令（用于积分）
    GuidanceOutput last_{};
    int sends_ = 0, sat_ = 0, hold_ = 0, search_ = 0;
};

// ---------------------------------------------------------------- 相机模型
//
// 相机约定：x 右 / y 下 / z 前（光轴）。外参给出"相机三轴在机体 FRD 下的坐标"：
//   camX(右)   → bodyY(右)   = (0,1,0)
//   camY(下)   → bodyZ(下)   = (0,0,1)
//   camZ(前/光轴) → bodyX(前) = (1,0,0)
// ⇒ 行主序 R_body_cam = [0 0 1; 1 0 0; 0 1 0]
// （真机上的真实值来自 VIO 标定：src/vio/extrinsic/results/final/）
CameraModel make_camera() {
    CameraModel c;
    c.fx = 600.0; c.fy = 600.0; c.cx = 320.0; c.cy = 240.0;
    const double R[9] = {0, 0, 1, 1, 0, 0, 0, 1, 0};
    for (int i = 0; i < 9; ++i) c.R_body_cam[i] = R[i];
    return c;
}

// ---------------------------------------------------------------- 极简参数解析
struct Args {
    int    hz    = 50;
    int    law   = 0;        // 0=PPN 1=TPN 2=APN
    double N     = 4.0;      // 导航比（工程 3~5）
    double a_max = 10.0;     // 过载上限（x500 量级）
    int    ticks = 300;      // 总拍数（50 Hz × 6 s）
    bool   verbose = true;
    bool   realtime = false; // 是否按真实时间节拍（默认全速跑，便于快速迭代）
};

bool parse_uint(const char *s, int &out) {
    char *end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (!end || *end != '\0' || v < 0) return false;
    out = static_cast<int>(v);
    return true;
}

bool parse_double(const char *s, double &out) {
    char *end = nullptr;
    const double v = std::strtod(s, &end);
    if (!end || *end != '\0') return false;
    out = v;
    return true;
}

const char *status_name(LoopStatus s) {
    switch (s) {
        case LoopStatus::kOk:             return "kOk";
        case LoopStatus::kNotInitialized: return "kNotInitialized";
        case LoopStatus::kStopped:        return "kStopped";
        case LoopStatus::kSensorTimeout:  return "kSensorTimeout";
        case LoopStatus::kUnhealthyInput: return "kUnhealthyInput";
        case LoopStatus::kSinkError:      return "kSinkError";
        case LoopStatus::kEstimatorReset: return "kEstimatorReset";
    }
    return "?";
}

void usage(const char *prog) {
    std::printf(
        "用法: %s [选项]\n"
        "  --hz=N        主循环频率（默认 50）\n"
        "  --law=N       0=PPN 1=TPN 2=APN（默认 0）\n"
        "  --n=X         导航比（默认 4）\n"
        "  --a-max=X     过载上限 m/s²（默认 10）\n"
        "  --ticks=N     总拍数（默认 300）\n"
        "  --quiet       只打汇总\n",
        prog);
}

}  // namespace

// ============================================================================
int main(int argc, char **argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        auto eq = [a](const char *key) { return std::strncmp(a, key, std::strlen(key)) == 0; };
        if (eq("--hz="))          { parse_uint(a + 5, args.hz); }
        else if (eq("--law="))    { parse_uint(a + 6, args.law); }
        else if (eq("--n="))      { parse_double(a + 4, args.N); }
        else if (eq("--a-max="))  { parse_double(a + 8, args.a_max); }
        else if (eq("--ticks="))  { parse_uint(a + 8, args.ticks); }
        else if (std::strcmp(a, "--realtime") == 0) { args.realtime = true; }
        else if (std::strcmp(a, "--quiet") == 0) { args.verbose = false; }
        else if (std::strcmp(a, "--help") == 0 || std::strcmp(a, "-h") == 0) { usage(argv[0]); return 0; }
        else { std::printf("未知参数：%s（--help 看用法）\n", a); return 2; }
    }
    if (args.hz <= 0 || args.ticks <= 0) {
        std::printf("参数非法：hz 与 ticks 必须 > 0\n");
        return 2;
    }

    // ---- 制导参数 ----
    GuidanceParams params;
    params.N     = args.N;
    params.a_max = args.a_max;
    params.law   = (args.law == 1) ? GuidanceLaw::TPN
                 : (args.law == 2) ? GuidanceLaw::APN
                                   : GuidanceLaw::PPN;
    // 合成世界给的是真值，可以直接用"真值法"算 ω（真机上必须改成 FROM_LOS_DIFFERENCE）
    params.los_rate_source = LosRateSource::FROM_RELATIVE_VELOCITY;

    // ---- 循环配置 ----
    LoopConfig cfg;
    cfg.cam     = make_camera();
    cfg.loop_hz = args.hz;
    // 合成世界每拍都更新，把"过期"阈值放宽一点点，避免浮点边界误判
    cfg.max_sample_age_s = 0.2;

    // ---- 场景：无人机在 5 m 高度朝 +x 飞 8 m/s，靶在 40 m 前方偏右 3 m ----
    const double dt = 1.0 / static_cast<double>(args.hz);
    SyntheticWorld world(cfg.cam,
                         Vec3(0.0, 0.0, -5.0),      // 起点：NED，z=-5 ⇒ 高 5 m
                         Vec3(8.0, 0.0, 0.0),       // 初速：朝北(+x) 8 m/s
                         Vec3(40.0, 3.0, -5.0),     // 装甲板（同高度，偏右 3 m）
                         dt);

    GuidanceLoop loop(params, cfg, world, world, world);
    if (!loop.init()) {
        std::printf("GuidanceLoop 初始化失败（检查相机模型 / 参数）\n");
        return 1;
    }

    const char *law_name = (args.law == 1) ? "TPN" : (args.law == 2) ? "APN" : "PPN";
    std::printf("=== pure-uav 制导主程序（合成世界，闭环）===\n");
    std::printf("  律=%s  N=%.1f  a_max=%.1f m/s²  loop=%d Hz  拍数=%d\n",
                law_name, args.N, args.a_max, args.hz, args.ticks);
    std::printf("  起点 (0,0,-5) NED，初速 (8,0,0)，靶 (40,3,-5)\n");
    std::printf("  ⚠ 理想点质量积分（无姿态环/重力），目的在验证制导链路\n");
    if (!args.realtime) {
        std::printf("  ⚠ 全速模式：降级计时按**墙钟**计（hold_last_max_s / sensor_timeout），\n");
        std::printf("     全速下不会走完 ⇒ 只能看链路是否跑通；要看降级行为请加 --realtime\n");
    }
    std::printf("\n");
    std::printf("   拍   t(s)   R(m)   Vc(m/s)  mode  t_go(s)  a_ned=(x, y, z)      sat\n");

    double min_R = 1e9;
    int    min_R_tick = -1;
    int    rc = 0;

    for (int k = 0; k < args.ticks; ++k) {
        world.step();                                  // 推进真值（用上一拍指令）
        GuidanceOutput out;
        const LoopStatus st = loop.run_once(&out);
        if (st != LoopStatus::kOk) {
            // kSensorTimeout 在演示里是**正常现象**：无人机飞越目标后目标离开视野，
            // 目标帧不再更新 ⇒ 超过 sensor_timeout_s。其余状态码才算异常。
            if (st == LoopStatus::kSensorTimeout) {
                std::printf("  -- 目标已离开视野（传感器超时），演示结束 --\n");
                rc = 0;
            } else {
                std::printf("  !! run_once 返回 %s，提前结束\n", status_name(st));
                rc = 1;
            }
            break;
        }

        const double R = world.true_range();
        if (R < min_R) { min_R = R; min_R_tick = k; }

        if (args.verbose && (k % 5 == 0)) {
            const char *mn = (out.mode == PureGuidanceMode::PN)     ? "PN  "
                           : (out.mode == PureGuidanceMode::APN)    ? "APN "
                           : (out.mode == PureGuidanceMode::HOLD)   ? "HOLD"
                                                                   : "SECH";
            std::printf("  %4d  %5.2f  %5.2f  %6.2f   %s  %6.3f   (%6.2f,%6.2f,%6.2f)  %s\n",
                        k, k * dt, R, out.Vc, mn, out.t_go,
                        out.a_cmd_ned.x, out.a_cmd_ned.y, out.a_cmd_ned.z,
                        out.sat ? "Y" : ".");
        }

        // 默认**全速**跑；加 --realtime 才按真实时间节拍（便于肉眼观察）
        if (args.realtime) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(
                static_cast<long long>(dt * 1e9)));
        }
    }

    const LoopStats &s = loop.stats();
    std::printf("\n=== 汇总 ===\n");
    std::printf("  拍数 ticks=%llu  ins=%llu  tgt=%llu  sent=%llu  sink_err=%llu\n",
                (unsigned long long)s.ticks, (unsigned long long)s.ins_frames,
                (unsigned long long)s.tgt_frames, (unsigned long long)s.sent,
                (unsigned long long)s.sink_errors);
    std::printf("  饱和 sat=%d  冻结 hold=%d  丢失 search=%d  降级 unhealthy=%llu  safe_hover=%llu\n",
                world.sat(), world.hold(), world.search(),
                (unsigned long long)s.unhealthy, (unsigned long long)s.safe_hover);
    std::printf("  最近距离 min R = %.3f m（第 %d 拍）  末位置 (%.2f, %.2f, %.2f)\n",
                min_R, min_R_tick, world.pos().x, world.pos().y, world.pos().z);
    std::printf("  末速度 (%.2f, %.2f, %.2f)  max_latency=%llu us\n",
                world.vel().x, world.vel().y, world.vel().z,
                (unsigned long long)s.max_latency_us);

    if (min_R < 1.0) std::printf("  ⇒ 链路闭环成立：PN 把最近距离压到了 1 m 以内 ✅\n");
    else             std::printf("  ⇒ 最近距离仍 > 1 m：检查初速方向 / a_max / N\n");

    return rc;
}
