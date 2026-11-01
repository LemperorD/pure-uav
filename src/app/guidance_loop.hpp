#pragma once
// =============================================================================
//  guidance_loop.hpp —— 机载主循环外壳 + 内存黑匣子（合并后）
//
//  合并自：
//    · guidance_loop.hpp   主循环、统计、延迟打点
//    · ring_log.hpp        RingRecord / RingLog（最近 512 拍）
//  配置类型（CameraModel / LoopConfig）已移到 app_types.hpp。
//
//  【架构：函数式内核 + 面向对象外壳】
//      外壳（本类）   ：取数据 → 换算 → 调核 → 体检 → 发指令 → 统计
//      内核（纯函数） ：adapter::to_guidance_state() → guidance_step() → cmd_accel_to_ned()
//    内核**一行不改**（铁律 §1.5）。
//
//  【一拍做什么（run_once 的顺序，不允许颠倒）】
//      ① poll 两个数据源（拿“最近一帧”；没有新帧就复用上一帧 —— 零阶保持）
//      ② 目标：像素 (u,v) → 相机系 → 机体 → 世界（用本帧 q；内参与 LoopConfig::cam）
//      ③ 组装 adapter::RawFrame（外部口径 → 内部口径的唯一换算点）
//      ④ quick_check()（几 ns、零分配）—— 不健康就按 on_unhealthy 降级
//      ⑤ guidance_step()（纯函数）
//      ⑥ adapter::cmd_accel_to_ned()（唯一一次出口换算）
//      ⑦ sink->send()（PX4 需要“持续到达”⇒ 每拍都发）
//      ⑧ 统计（计数、延迟、饱和率）
//
//  【实时性约束】
//    · 循环内零分配、零 printf、零 std::string；
//    · 节拍用绝对时间睡眠（sleep_until），不累积漂移；
//    · 不用异常：init() 返回 bool，run_once() 返回 LoopStatus。
// =============================================================================

#include <atomic>
#include <chrono>
#include <cstdint>

#include "app_types.hpp"       // InsSample / TargetSample / 接口 / LoopConfig
#include "guidance_types.hpp"  // GuidanceParams / GuidanceMemory
#include "state_adapter.hpp"   // adapter::StateTimeTracker

namespace pure_uav {
namespace app {

// ============================================================================
//  §1  端到端打点（固定环形，循环内零分配）
// ============================================================================

struct LatencyStats {
    static constexpr int kCap = 128;      // 128 拍 ≈ 2.5 s @ 50 Hz
    enum class Stage : int { kPoll = 0, kProject, kGuidance, kSend, kTotal, kCount };

    struct Ring {
        std::uint64_t v[kCap] = {};
        int           head = 0;
        std::uint64_t n = 0;
        void add(std::uint64_t x) noexcept {
            v[head] = x;
            head = (head + 1) % kCap;
            ++n;
        }
        std::uint64_t count() const noexcept { return n; }
        int size() const noexcept { return static_cast<int>(n < kCap ? n : kCap); }
    };

    Ring r[static_cast<int>(Stage::kCount)];

    void add(std::uint64_t poll_ns, std::uint64_t project_ns,
             std::uint64_t guidance_ns, std::uint64_t send_ns,
             std::uint64_t total_ns) noexcept {
        r[static_cast<int>(Stage::kPoll)].add(poll_ns);
        r[static_cast<int>(Stage::kProject)].add(project_ns);
        r[static_cast<int>(Stage::kGuidance)].add(guidance_ns);
        r[static_cast<int>(Stage::kSend)].add(send_ns);
        r[static_cast<int>(Stage::kTotal)].add(total_ns);
    }

    std::uint64_t count() const noexcept { return r[0].count(); }
    int size() const noexcept { return r[0].size(); }

    std::uint64_t percentile_ns(Stage s, double p) const;
    std::uint64_t max_ns(Stage s) const;
};

// ============================================================================
//  §2  循环统计
// ============================================================================

struct LoopStats {
    std::uint64_t ticks        = 0;
    std::uint64_t ins_frames   = 0;
    std::uint64_t tgt_frames   = 0;
    std::uint64_t sent         = 0;
    std::uint64_t saturated    = 0;
    std::uint64_t hold         = 0;
    std::uint64_t search       = 0;
    std::uint64_t unhealthy    = 0;
    std::uint64_t safe_hover   = 0;
    std::uint64_t sink_errors  = 0;
    std::uint64_t max_latency_us = 0;
    LatencyStats  latency;
};

// ============================================================================
//  §3  内存黑匣子（最近 kCap 拍；循环内零 I/O，dump 才碰磁盘）
// ============================================================================

struct RingRecord {
    std::uint64_t boot_us = 0;
    std::uint8_t  ctrl    = 0;     // ControlMode
    std::uint8_t  mode    = 0;     // PureGuidanceMode
    std::uint8_t  health  = 0;     // InputHealth
    std::uint8_t  flags   = 0;     // bit0 valid / bit1 sat / bit2 hold_yaw / bit3 velocity
    double a_ned[3] = {0.0, 0.0, 0.0};
    double v_ned[3] = {0.0, 0.0, 0.0};
    double a_norm  = 0.0;
    double t_go    = 0.0;
    double Vc      = 0.0;
    double R       = 0.0;
    double lam_dot = 0.0;
};

class RingLog {
public:
    static constexpr int kCap = 512;   // ≈10.2 s @ 50 Hz

    void push(const GuidanceOutput &o) noexcept;

    void clear() noexcept {
        head_ = 0;
        n_    = 0;
    }

    bool empty() const noexcept { return n_ == 0; }
    int size() const noexcept { return static_cast<int>(n_ < kCap ? n_ : kCap); }
    std::uint64_t count() const noexcept { return n_; }

    const RingRecord &at(int i) const noexcept;
    const RingRecord &newest() const noexcept;

    // 最近 seconds 秒（按 boot_us）写成 CSV；<=0 表示全部；返回行数，失败 -1。
    // ⚠ 唯一 I/O 入口，绝不能在循环里调用。
    long dump_csv(const char *path, double seconds) const;

private:
    RingRecord    r_[kCap];
    int           head_ = 0;
    std::uint64_t n_    = 0;
};

// ============================================================================
//  §4  主循环
// ============================================================================

class GuidanceLoop {
public:
    // 依赖注入：只持有引用，不接管生命周期。
    GuidanceLoop(const GuidanceParams &params, const LoopConfig &cfg,
                 IInsSource &ins, ITargetSource &tgt, ICommandSink &sink);
    ~GuidanceLoop() = default;

    GuidanceLoop(const GuidanceLoop &) = delete;
    GuidanceLoop &operator=(const GuidanceLoop &) = delete;

    // 构造即有效：校验参数与相机、复位时基、清统计。失败返回 false。
    bool init();

    // 跑一拍；out 可空。
    LoopStatus run_once(GuidanceOutput *out = nullptr);

    // 阻塞跑主循环（绝对时间睡眠）。
    LoopStatus run();

    void request_stop();

    const LoopStats &stats() const { return stats_; }
    const RingLog &blackbox() const { return blackbox_; }
    bool initialized() const { return inited_; }

private:
    LoopStatus emit(const GuidanceOutput &o, GuidanceOutput *out);
    double local_now_s() const;

    // 外部依赖（引用，不拥有）
    const GuidanceParams params_;
    LoopConfig           cfg_;
    IInsSource    &ins_;
    ITargetSource &tgt_;
    ICommandSink  &sink_;

    // 内核状态
    GuidanceMemory memory_{};
    adapter::StateTimeTracker tracker_{};

    // 最近一帧采样（零阶保持）
    InsSample    ins_now_{};
    TargetSample tgt_now_{};
    bool have_ins_ = false;
    bool have_tgt_ = false;
    bool computed_ = false;
    GuidanceOutput last_out_{};
    double last_health_s_ = 0.0;

    // 计时（本地单调时钟，秒）
    std::chrono::steady_clock::time_point t_start_{};
    double last_ins_s_ = -1.0;
    double last_tgt_s_ = -1.0;
    double last_good_s_ = -1.0;

    // 生命周期
    bool inited_ = false;
    std::atomic<bool> stop_requested_{false};

    LoopStats stats_{};
    RingLog   blackbox_{};
};

}  // namespace app
}  // namespace pure_uav
