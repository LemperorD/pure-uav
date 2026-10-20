// =============================================================================
//  GuidanceLoop 实现 —— 机载主循环的外壳（函数式内核 + 面向对象外壳）
//
//  设计要点见 guidance_loop.hpp 的头注释。这里只强调三个实现层面的关键决定：
//
//   ① **只在"目标帧更新"时才重算制导**
//      `FROM_LOS_DIFFERENCE` 用 `state.t - memory.t_prev` 当 dt，而 û 只在目标帧更新。
//      所以 `state.t` 必须取**目标帧的采样时刻**、且只在目标帧到达时调用 guidance_step；
//      其余拍子只重发上一拍指令（PX4 offboard 要求 setpoint 持续到达）。
//      反例（必须避免）：用飞控帧时间喂 tracker，却按目标帧更新 û
//        ⇒ dt=20 ms 而 û 实际跨了 33 ms ⇒ ω 被少算约 40% ⇒ 制导偏软。
//
//   ② **像素 → 相机系 → 机体(FRD) → 世界** 全部在本文件完成
//      视觉只给像素（《视觉制导接口约定》§6），内参/外参来自 LoopConfig::cam，
//      姿态 q 来自 InsSample。三步之后得到**世界系单位视线**，再交给 adapter。
//
//   ③ **降级不停流**：不健康时按 `on_unhealthy` 发"安全 setpoint"
//      （保持上一条限定 0.2 s → 零速度悬停），而不是断流——断流会让 PX4 掉出 offboard。
// =============================================================================

#include "guidance_loop.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include "guidance.hpp"        // guidance_step()
#include "quat.hpp"            // quat_rotate_vec()（相机→机体→世界 的最后一步）
#include "state_adapter.hpp"   // to_guidance_state() / cmd_accel_to_ned()
#include "state_check.hpp"     // quick_check()

namespace pure_uav {
namespace app {

namespace {

// ---------------------------------------------------------------- 小工具

// 3×3（行主序）× 3 向量
inline Vec3 mat3_mul(const double R[9], const Vec3 &v) noexcept {
    return Vec3(R[0] * v.x + R[1] * v.y + R[2] * v.z,
                R[3] * v.x + R[4] * v.y + R[5] * v.z,
                R[6] * v.x + R[7] * v.y + R[8] * v.z);
}

// 像素 → 世界系单位视线
//
//   第 1 步（本函数）：(u,v) → 相机系射线 d_cam = ((u−cx)/fx, (v−cy)/fy, 1)
//                       —— 相机约定：x 右 / y 下 / z 前（光轴）
//   第 2 步（本函数）：d_body = R_body_cam · d_cam   ← 外参（LoopConfig::cam）
//                       `R_body_cam` 的目标系是 **PX4 FRD**（x 前 / y 右 / z 下），
//                       与 InsSample.q 的机体定义一致。
//   第 3 步（本函数）：d_world = R(q) · d_body        ← 姿态（机体 → 世界）
//
// 返回 false = 参数/输入退化（相机无效、分母为 0、结果零矢量）。
bool pixel_to_world_los(const CameraModel &cam, const TargetSample &t,
                        const double q_frd_world[4], Vec3 &los_world) noexcept {
    if (!cam.valid()) return false;
    if (!std::isfinite(t.u_px) || !std::isfinite(t.v_px)) return false;

    const double xn = (t.u_px - cam.cx) / cam.fx;
    const double yn = (t.v_px - cam.cy) / cam.fy;
    if (!std::isfinite(xn) || !std::isfinite(yn)) return false;

    const Vec3 d_cam(xn, yn, 1.0);
    const Vec3 d_body  = mat3_mul(cam.R_body_cam, d_cam);
    const Vec3 d_world = quat_rotate_vec(q_frd_world, d_body);

    const double n = norm(d_world);
    if (!(n > 1e-9)) return false;                 // 同时挡住 NaN
    los_world = d_world / n;
    return true;
}

// 把体检的 CheckCode 映射成对外的 InputHealth
InputHealth to_health(adapter::CheckCode code) noexcept {
    switch (code) {
        case adapter::CheckCode::kOk:            return InputHealth::kOk;
        case adapter::CheckCode::kLosZero:       return InputHealth::kLosZero;
        case adapter::CheckCode::kLosNotUnit:    return InputHealth::kLosNotUnit;
        case adapter::CheckCode::kQuatNotUnit:   return InputHealth::kQuatNotUnit;
        case adapter::CheckCode::kRangeNegative: return InputHealth::kRangeNegative;
        default:                        return InputHealth::kNotFinite;
    }
}

// 安全悬停输出（零速度；走 PX4 速度环闭环，而不是"零加速度"那种纯前馈）
inline GuidanceOutput make_safe_hover(std::uint64_t boot_us, InputHealth health) noexcept {
    GuidanceOutput o;
    o.boot_us    = boot_us;
    o.ctrl       = ControlMode::kVelocity;
    o.a_cmd_ned  = Vec3{};
    o.v_cmd_ned  = Vec3{};                 // 零速度
    o.hold_yaw   = true;                   // 航向不干预（VELOCITY 模式将忽略 yaw）
    o.a_cmd_norm = 0.0;
    o.sat        = false;
    o.mode       = PureGuidanceMode::SEARCH;
    o.valid      = false;
    o.health     = health;
    return o;
}

}  // namespace

// ============================================================================
//  构造 / 初始化
// ============================================================================

GuidanceLoop::GuidanceLoop(const GuidanceParams &params, const LoopConfig &cfg,
                           IInsSource &ins, ITargetSource &tgt, ICommandSink &sink)
    : params_(params), cfg_(cfg), ins_(ins), tgt_(tgt), sink_(sink) {}

bool GuidanceLoop::init() {
    if (inited_) return true;

    // ---- 参数校验（构造即有效：失败就给 false，不抛异常）----
    if (!(params_.a_max > 0.0) || !(params_.N > 0.0)) return false;
    if (cfg_.loop_hz <= 0 || cfg_.loop_hz > 2000) return false;
    if (!(cfg_.max_sample_age_s > 0.0) || !(cfg_.sensor_timeout_s > 0.0)) return false;
    if (!(cfg_.hold_last_max_s >= 0.0)) return false;
    if (!cfg_.cam.valid()) return false;        // 目标数据是像素 ⇒ 必须有相机模型

    tracker_.reset();
    memory_        = GuidanceMemory{};
    stats_         = LoopStats{};
    have_ins_      = false;
    have_tgt_      = false;
    computed_      = false;
    last_out_      = GuidanceOutput{};
    last_ins_s_    = -1.0;
    last_tgt_s_    = -1.0;
    last_good_s_       = -1.0;
    t_start_       = std::chrono::steady_clock::now();

    inited_ = true;
    return true;
}

void GuidanceLoop::request_stop() { stop_requested_.store(true, std::memory_order_relaxed); }

double GuidanceLoop::local_now_s() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start_).count();
}

// ============================================================================
//  输出出口：发送 + 统计 + 延迟
// ============================================================================

LoopStatus GuidanceLoop::emit(const GuidanceOutput &o, GuidanceOutput *out) {
    if (!sink_.send(o)) {
        ++stats_.sink_errors;
        return LoopStatus::kSinkError;
    }
    ++stats_.sent;

    // 延迟：采样时刻 → 现在（"现在"用最新飞控帧的 boot 时间戳，同一时基）
    const std::uint64_t now_us = ins_now_.boot_us;
    if (now_us >= o.boot_us) {
        const std::uint64_t lat = now_us - o.boot_us;
        if (lat > stats_.max_latency_us) stats_.max_latency_us = lat;
    }
    if (out) *out = o;
    return LoopStatus::kOk;
}

// ============================================================================
//  一拍
// ============================================================================

LoopStatus GuidanceLoop::run_once(GuidanceOutput *out) {
    if (!inited_) return LoopStatus::kNotInitialized;
    if (stop_requested_.load(std::memory_order_relaxed)) return LoopStatus::kStopped;

    ++stats_.ticks;
    const double t_local = local_now_s();

    // ---- ① 取数（拉模型：拿不到新帧就复用上一帧）----
    const bool new_ins = ins_.poll(ins_now_);
    if (new_ins) {
        ++stats_.ins_frames;
        have_ins_   = true;
        last_ins_s_ = t_local;
    }
    const bool new_tgt = tgt_.poll(tgt_now_);
    if (new_tgt) {
        have_tgt_   = true;
        last_tgt_s_ = t_local;
        if (tgt_now_.present) ++stats_.tgt_frames;
    }

    // ---- ② 掉线判定（本地单调时钟；数据时间戳只用于"新鲜度"）----
    if (have_ins_ && (t_local - last_ins_s_) > cfg_.sensor_timeout_s) {
        return LoopStatus::kSensorTimeout;
    }
    if (have_tgt_ && (t_local - last_tgt_s_) > cfg_.sensor_timeout_s) {
        return LoopStatus::kSensorTimeout;
    }

    // ---- ③ 首帧还没凑齐：本拍不产出（返回 ok，避免启动即退出）----
    if (!have_ins_ || !have_tgt_) return LoopStatus::kOk;

    // ---- ④ 新鲜度（同一时基：都用 boot_us）----
    const std::uint64_t now_us =
        (ins_now_.boot_us != 0) ? ins_now_.boot_us : tgt_now_.boot_us;
    const double age_s = (now_us >= tgt_now_.boot_us)
                             ? static_cast<double>(now_us - tgt_now_.boot_us) * 1e-6
                             : 0.0;

    InputHealth health = InputHealth::kOk;
    if (age_s > cfg_.max_sample_age_s) health = InputHealth::kStale;

    // ---- ⑤ 有新的目标帧 && 数据新鲜 → 重算制导 ----
    if (health == InputHealth::kOk && new_tgt) {
        // 5.1 像素 → 世界系单位视线
        Vec3 los_world;
        bool los_ok = false;
        if (tgt_now_.present) {
            los_ok = pixel_to_world_los(cfg_.cam, tgt_now_, ins_now_.q, los_world);
            if (!los_ok) health = InputHealth::kLosZero;
        }

        if (health == InputHealth::kOk) {
            // 5.2 组装外部口径帧（唯一换算入口）
            adapter::RawFrame raw;
            raw.world = adapter::Frame::kNed;              // 外部口径：飞控 local 系（z 朝下）
            raw.att   = adapter::AttitudeFrame::kFrd;      // 我们的 q 是 FRD → 世界
            raw.ins.p = ins_now_.p;
            raw.ins.v = ins_now_.v;
            for (int i = 0; i < 4; ++i) raw.ins.q[i] = ins_now_.q[i];
            raw.ins.has_position = ins_now_.has_position;
            raw.ins.has_attitude = ins_now_.has_attitude;

            raw.target.los        = los_world;             // 已转到世界系
            raw.target.has_range  = tgt_now_.has_range;
            raw.target.range_m    = tgt_now_.range_m;
            raw.target.has_vT     = tgt_now_.has_v_T;
            raw.target.vT         = tgt_now_.v_T;
            raw.target.present    = tgt_now_.present;

            GuidanceState st = adapter::to_guidance_state(raw);

            // 5.3 时间：用**目标帧采样时刻**（ω 的 dt 必须对应 û 的更新间隔）
            tracker_.push_u64_us(tgt_now_.boot_us);
            st.t = tracker_.t_s();

            // 5.4 廉价体检（几 ns、零分配）；目标丢失时不查（valid=false 会走 SEARCH）
            if (tgt_now_.present) {
                adapter::CheckCode code = adapter::CheckCode::kOk;
                if (!adapter::quick_check(st, &code)) health = to_health(code);
            }

            // 5.5 调核（纯函数）→ 出口换算（唯一一次 ENU→NED）
            if (health == InputHealth::kOk) {
                const PureGuidanceCmd cmd = guidance_step(st, params_, memory_);

                double ned[3];
                adapter::cmd_accel_to_ned(cmd, ned);

                GuidanceOutput o;
                o.boot_us    = tgt_now_.boot_us;
                o.ctrl       = ControlMode::kAcceleration;
                o.a_cmd_ned  = Vec3(ned[0], ned[1], ned[2]);
                o.v_cmd_ned  = Vec3{};
                o.hold_yaw   = true;
                o.a_cmd_norm = cmd.a_cmd_norm;
                o.sat        = cmd.sat;
                o.mode       = cmd.mode;
                o.valid      = cmd.valid;
                o.t_go       = cmd.t_go;
                o.Vc         = cmd.Vc;
                o.R          = cmd.R;
                o.lam_dot    = cmd.lam_dot;
                o.health     = InputHealth::kOk;

                if (cmd.sat) ++stats_.saturated;
                if (cmd.mode == PureGuidanceMode::HOLD) ++stats_.hold;
                if (cmd.mode == PureGuidanceMode::SEARCH) ++stats_.search;

                computed_          = true;
                last_out_          = o;
                last_good_s_       = t_local;   // 保持窗口的起算点：最后一次成功算出指令
                return emit(o, out);
            }
        }
    }

    // ---- ⑥ 没有新的目标帧：重发上一拍（offboard 需要持续到达）----
    if (health == InputHealth::kOk && !new_tgt) {
        if (!computed_ || !cfg_.send_on_every_tick) return LoopStatus::kOk;
        GuidanceOutput o = last_out_;
        o.boot_us = tgt_now_.boot_us;                  // 沿用最新采样时刻做延迟统计
        return emit(o, out);
    }

    // ---- ⑦ 不健康：按策略降级（**不停流**）----
    ++stats_.unhealthy;
    // 从"最后一次成功算出指令"起算：一条旧指令最多再飞 hold_last_max_s
    const double stale_cmd_age_s = (last_good_s_ >= 0.0) ? (t_local - last_good_s_) : 1e9;

    GuidanceOutput o;
    switch (cfg_.on_unhealthy) {
        case LoopConfig::OnUnhealthy::kHoldLastCommand:
            // 先保持上一条（限时），超时后转零速度悬停，避免"抱着错指令一直推"
            if (computed_ && stale_cmd_age_s <= cfg_.hold_last_max_s) {
                o = last_out_;
                o.health = health;
            } else {
                o = make_safe_hover(tgt_now_.boot_us, health);
                ++stats_.safe_hover;
            }
            break;

        case LoopConfig::OnUnhealthy::kSafeHover:
            o = make_safe_hover(tgt_now_.boot_us, health);
            ++stats_.safe_hover;
            break;

        case LoopConfig::OnUnhealthy::kStop:
            // 不发（把主动权交给飞控 offboard 失联 failsafe）
            return LoopStatus::kUnhealthyInput;
    }

    last_out_ = o;
    return emit(o, out);
}

// ============================================================================
//  阻塞主循环
// ============================================================================

LoopStatus GuidanceLoop::run() {
    if (!inited_) return LoopStatus::kNotInitialized;

    const int hz = (cfg_.loop_hz > 0) ? cfg_.loop_hz : 1;
    const auto period = std::chrono::nanoseconds(
        static_cast<long long>(1e9 / static_cast<double>(hz)));

    // 绝对时间节拍：每拍把 next 往前推一个周期（不累积漂移）。
    // （Linux 上等价于 clock_nanosleep(TIMER_ABSTIME)；std::this_thread::sleep_until 是标准写法）
    auto next = std::chrono::steady_clock::now();
    while (!stop_requested_.load(std::memory_order_relaxed)) {
        const LoopStatus s = run_once(nullptr);
        if (s != LoopStatus::kOk) return s;

        next += period;
        std::this_thread::sleep_until(next);
    }
    return LoopStatus::kStopped;
}

}  // namespace app
}  // namespace pure_uav
