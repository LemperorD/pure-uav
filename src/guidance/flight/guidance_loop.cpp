// =============================================================================
//  guidance_loop.cpp —— 主循环 + 内存黑匣子实现
//
//  只在目标帧更新时重算；state.t 取目标帧时间（ω 的 dt 必须对应 û 更新间隔）。
//  设计原因见 assets/docs/decisions/0003-los-rate-source.md。
// =============================================================================

#include "guidance_loop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "guidance.hpp"        // guidance_step()
#include "quat.hpp"            // quat_rotate_vec()
#include "state_adapter.hpp"   // to_guidance_state() / cmd_accel_to_ned()
#include "state_check.hpp"     // quick_check()

namespace pure_uav {
namespace app {

// ============================================================================
//  LatencyStats 实现（只在循环外调用：复制到栈上排序，无堆分配）
// ============================================================================

std::uint64_t LatencyStats::percentile_ns(Stage s, double p) const {
    const int idx = static_cast<int>(s);
    if (idx < 0 || idx >= static_cast<int>(Stage::kCount)) return 0;
    const Ring &ring = r[idx];
    const int m = ring.size();
    if (m <= 0) return 0;

    std::uint64_t tmp[kCap];
    for (int i = 0; i < m; ++i) tmp[i] = ring.v[i];
    std::sort(tmp, tmp + m);

    const double q = (p < 0.0) ? 0.0 : (p > 1.0 ? 1.0 : p);
    int pos = static_cast<int>(std::ceil(q * static_cast<double>(m))) - 1;
    if (pos < 0) pos = 0;
    if (pos >= m) pos = m - 1;
    return tmp[pos];
}

std::uint64_t LatencyStats::max_ns(Stage s) const {
    const int idx = static_cast<int>(s);
    if (idx < 0 || idx >= static_cast<int>(Stage::kCount)) return 0;
    const Ring &ring = r[idx];
    const int m = ring.size();
    if (m <= 0) return 0;
    std::uint64_t mx = 0;
    for (int i = 0; i < m; ++i) {
        if (ring.v[i] > mx) mx = ring.v[i];
    }
    return mx;
}

// ============================================================================
//  RingLog 实现（push 零 I/O；dump 才碰磁盘）
// ============================================================================

void RingLog::push(const GuidanceOutput &o) noexcept {
    RingRecord &r = r_[head_];
    r = RingRecord{};                       // 清零（避免上一圈残留）

    r.boot_us = o.boot_us;
    r.ctrl    = static_cast<std::uint8_t>(o.ctrl);
    r.mode    = static_cast<std::uint8_t>(o.mode);
    r.health  = static_cast<std::uint8_t>(o.health);

    unsigned f = 0;
    if (o.valid)    f |= 1u;
    if (o.sat)      f |= 2u;
    if (o.hold_yaw) f |= 4u;
    if (o.ctrl == ControlMode::kVelocity) f |= 8u;
    r.flags = static_cast<std::uint8_t>(f);

    r.a_ned[0] = o.a_cmd_ned.x;
    r.a_ned[1] = o.a_cmd_ned.y;
    r.a_ned[2] = o.a_cmd_ned.z;
    r.v_ned[0] = o.v_cmd_ned.x;
    r.v_ned[1] = o.v_cmd_ned.y;
    r.v_ned[2] = o.v_cmd_ned.z;
    r.a_norm   = o.a_cmd_norm;
    r.t_go     = o.t_go;
    r.Vc       = o.Vc;
    r.R        = o.R;
    r.lam_dot  = o.lam_dot;

    head_ = (head_ + 1) % kCap;
    ++n_;
}

const RingRecord &RingLog::at(int i) const noexcept {
    static const RingRecord kZero{};
    const int m = size();
    if (m <= 0) return kZero;
    if (i < 0) i = 0;
    if (i >= m) i = m - 1;
    const int start = (n_ < static_cast<std::uint64_t>(kCap)) ? 0 : head_;
    return r_[(start + i) % kCap];
}

const RingRecord &RingLog::newest() const noexcept { return at(size() - 1); }

long RingLog::dump_csv(const char *path, double seconds) const {
    if (path == nullptr) return -1;
    std::FILE *f = std::fopen(path, "wb");
    if (f == nullptr) return -1;

    std::fprintf(f,
                 "boot_us,ctrl,mode,health,valid,sat,hold_yaw,"
                 "a_n,a_e,a_d,v_n,v_e,v_d,anorm,t_go,Vc,R,lam_dot\n");

    const int m = size();
    if (m <= 0) {
        return (std::fclose(f) == 0) ? 0 : -1;
    }

    const std::uint64_t newest_us = at(m - 1).boot_us;
    const bool use_thresh = (seconds > 0.0);
    std::uint64_t thresh = 0;
    if (use_thresh) {
        const double span_us = seconds * 1e6;
        thresh = (span_us >= static_cast<double>(newest_us))
                     ? 0
                     : newest_us - static_cast<std::uint64_t>(span_us);
    }

    long rows = 0;
    for (int i = 0; i < m; ++i) {
        const RingRecord &r = at(i);
        if (use_thresh && r.boot_us < thresh) continue;
        std::fprintf(f,
                     "%llu,%u,%u,%u,%d,%d,%d,"
                     "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                     static_cast<unsigned long long>(r.boot_us),
                     static_cast<unsigned>(r.ctrl), static_cast<unsigned>(r.mode),
                     static_cast<unsigned>(r.health),
                     (r.flags & 1u) ? 1 : 0, (r.flags & 2u) ? 1 : 0,
                     (r.flags & 4u) ? 1 : 0,
                     r.a_ned[0], r.a_ned[1], r.a_ned[2],
                     r.v_ned[0], r.v_ned[1], r.v_ned[2],
                     r.a_norm, r.t_go, r.Vc, r.R, r.lam_dot);
        ++rows;
    }

    return (std::fclose(f) == 0) ? rows : -1;
}

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
//   第 1 步：(u,v) → 相机系射线 d_cam = ((u−cx)/fx, (v−cy)/fy, 1)
//   第 2 步：d_body = R_body_cam · d_cam
//   第 3 步：d_world = R(q) · d_body
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

// 安全悬停输出（零速度；走 PX4 速度环闭环）
inline GuidanceOutput make_safe_hover(std::uint64_t boot_us, InputHealth health) noexcept {
    GuidanceOutput o;
    o.boot_us    = boot_us;
    o.ctrl       = ControlMode::kVelocity;
    o.a_cmd_ned  = Vec3{};
    o.v_cmd_ned  = Vec3{};
    o.hold_yaw   = true;
    o.a_cmd_norm = 0.0;
    o.sat        = false;
    o.mode       = PureGuidanceMode::SEARCH;
    o.valid      = false;
    o.health     = health;
    return o;
}

// 打点用 RAII：所有 return 路径都会落一条样本。
struct LatencyTrace {
    LatencyStats *dst = nullptr;
    std::chrono::steady_clock::time_point t0{}, t1{}, t2{}, t3{};
    bool armed = false;

    ~LatencyTrace() {
        if (!armed || dst == nullptr) return;
        const auto t4 = std::chrono::steady_clock::now();
        auto ns = [](const std::chrono::steady_clock::time_point &a,
                     const std::chrono::steady_clock::time_point &b) {
            return static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count());
        };
        dst->add(ns(t0, t1), ns(t1, t2), ns(t2, t3), ns(t3, t4), ns(t0, t4));
    }
};

}  // namespace

// ============================================================================
//  构造 / 初始化
// ============================================================================

GuidanceLoop::GuidanceLoop(const GuidanceParams &params, const LoopConfig &cfg,
                           IInsSource &ins, ITargetSource &tgt, ICommandSink &sink)
    : params_(params), cfg_(cfg), ins_(ins), tgt_(tgt), sink_(sink) {}

bool GuidanceLoop::init() {
    if (inited_) return true;

    if (!(params_.a_max > 0.0) || !(params_.N > 0.0)) return false;
    if (cfg_.loop_hz <= 0 || cfg_.loop_hz > 2000) return false;
    if (!(cfg_.max_sample_age_s > 0.0) || !(cfg_.sensor_timeout_s > 0.0)) return false;
    if (!(cfg_.hold_last_max_s >= 0.0)) return false;
    if (!cfg_.cam.valid()) return false;        // 目标数据是像素 ⇒ 必须有相机模型

    tracker_.reset();
    memory_        = GuidanceMemory{};
    stats_         = LoopStats{};
    blackbox_.clear();
    have_ins_      = false;
    have_tgt_      = false;
    computed_      = false;
    last_out_      = GuidanceOutput{};
    last_ins_s_    = -1.0;
    last_tgt_s_    = -1.0;
    last_good_s_   = -1.0;
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
    blackbox_.push(o);                 // ★ 先记黑匣子（即使发送失败也要留下证据）
    if (!sink_.send(o)) {
        ++stats_.sink_errors;
        return LoopStatus::kSinkError;
    }
    ++stats_.sent;

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

    LatencyTrace trace;
    trace.dst   = &stats_.latency;
    trace.t0    = std::chrono::steady_clock::now();
    trace.t1    = trace.t2 = trace.t3 = trace.t0;
    trace.armed = true;

    ++stats_.ticks;
    const double t_local = local_now_s();

    // ---- ① 取数 ----
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

    trace.t1 = std::chrono::steady_clock::now();
    trace.t2 = trace.t3 = trace.t1;

    // ---- ② 掉线判定 ----
    if (have_ins_ && (t_local - last_ins_s_) > cfg_.sensor_timeout_s) {
        return LoopStatus::kSensorTimeout;
    }
    if (have_tgt_ && (t_local - last_tgt_s_) > cfg_.sensor_timeout_s) {
        return LoopStatus::kSensorTimeout;
    }

    // ---- ③ 首帧还没凑齐 ----
    if (!have_ins_ || !have_tgt_) return LoopStatus::kOk;

    // ---- ④ 新鲜度 ----
    const std::uint64_t now_us =
        (ins_now_.boot_us != 0) ? ins_now_.boot_us : tgt_now_.boot_us;
    const double age_s = (now_us >= tgt_now_.boot_us)
                             ? static_cast<double>(now_us - tgt_now_.boot_us) * 1e-6
                             : 0.0;

    InputHealth health = InputHealth::kOk;
    if (age_s > cfg_.max_sample_age_s) health = InputHealth::kStale;

    // ---- ⑤ 有新的目标帧 && 数据新鲜 → 重算制导 ----
    if (health == InputHealth::kOk && new_tgt) {
        Vec3 los_world;
        bool los_ok = false;
        if (tgt_now_.present) {
            los_ok = pixel_to_world_los(cfg_.cam, tgt_now_, ins_now_.q, los_world);
            if (!los_ok) health = InputHealth::kLosZero;
        }

        if (health == InputHealth::kOk) {
            adapter::RawFrame raw;
            raw.world = adapter::Frame::kNed;
            raw.att   = adapter::AttitudeFrame::kFrd;
            raw.ins.p = ins_now_.p;
            raw.ins.v = ins_now_.v;
            for (int i = 0; i < 4; ++i) raw.ins.q[i] = ins_now_.q[i];
            raw.ins.has_position = ins_now_.has_position;
            raw.ins.has_attitude = ins_now_.has_attitude;

            raw.target.los        = los_world;
            raw.target.has_range  = tgt_now_.has_range;
            raw.target.range_m    = tgt_now_.range_m;
            raw.target.has_vT     = tgt_now_.has_v_T;
            raw.target.vT         = tgt_now_.v_T;
            raw.target.present    = tgt_now_.present;

            GuidanceState st = adapter::to_guidance_state(raw);

            tracker_.push_u64_us(tgt_now_.boot_us);
            st.t = tracker_.t_s();

            if (tgt_now_.present) {
                adapter::CheckCode code = adapter::CheckCode::kOk;
                if (!adapter::quick_check(st, &code)) health = to_health(code);
            }

            if (health == InputHealth::kOk) {
                trace.t2 = std::chrono::steady_clock::now();
                const PureGuidanceCmd cmd = guidance_step(st, params_, memory_);

                double ned[3];
                adapter::cmd_accel_to_ned(cmd, ned);
                trace.t3 = std::chrono::steady_clock::now();

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
                last_good_s_       = t_local;
                return emit(o, out);
            }
        }
    }

    // ---- ⑥ 没有新的目标帧：重发上一拍 ----
    if (health == InputHealth::kOk && !new_tgt) {
        if (!computed_ || !cfg_.send_on_every_tick) return LoopStatus::kOk;
        GuidanceOutput o = last_out_;
        o.boot_us = tgt_now_.boot_us;
        return emit(o, out);
    }

    // ---- ⑦ 不健康：按策略降级（不停流）----
    ++stats_.unhealthy;
    const double stale_cmd_age_s = (last_good_s_ >= 0.0) ? (t_local - last_good_s_) : 1e9;

    GuidanceOutput o;
    switch (cfg_.on_unhealthy) {
        case LoopConfig::OnUnhealthy::kHoldLastCommand:
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
            blackbox_.push(o);
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
