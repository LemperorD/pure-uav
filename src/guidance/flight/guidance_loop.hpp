#pragma once
// =============================================================================
//  guidance_loop.hpp —— 飞行外壳：主循环 + 内存黑匣子
//
//  一拍：poll → 像素反投影 → adapter → quick_check → guidance_step → NED → send → 统计。
//  实时性：循环内零分配、零 I/O；节拍用绝对时间睡眠。
//  设计/时间基准见 assets/docs/decisions/0003-los-rate-source.md，
//  整体说明见 assets/docs/guidance/README.md 与 assets/docs/app/README.md。
// =============================================================================

#include <atomic>
#include <chrono>
#include <cstdint>

#include "flight_types.hpp"    // InsSample / TargetSample / 接口 / LoopConfig
#include "guidance_types.hpp"  // GuidanceParams / GuidanceMemory
#include "state_adapter.hpp"   // adapter::StateTimeTracker

namespace pure_uav {
namespace guidance {
namespace flight {

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
    // 说明见 assets/docs/app/blackbox.md；唯一 I/O 入口，绝不能在循环里调用。
    long dump_csv(const char *path, double seconds) const;

private:
    RingRecord    r_[kCap];
    int           head_ = 0;
    std::uint64_t n_    = 0;
};

// ============================================================================
//  §4  主循环
// ============================================================================

/// 机载主循环：取数 → 反投影 → adapter → guidance_step → NED → 发送。
/// 只持有外部引用，不接管生命周期；循环内零分配、零 I/O。
class GuidanceLoop {
public:
    /// 构造：注入参数、配置、两个数据源、一个出口。
    GuidanceLoop(const GuidanceParams &params, const LoopConfig &cfg,
                 IInsSource &ins, ITargetSource &tgt, ICommandSink &sink);
    ~GuidanceLoop() = default;

    GuidanceLoop(const GuidanceLoop &) = delete;
    GuidanceLoop &operator=(const GuidanceLoop &) = delete;

    /// @brief 校验参数/相机并复位状态。
    /// @return false = 参数或相机无效。
    bool init();

    /// @brief 跑一拍。
    /// @param[out] out 可空；非空时写入本拍输出。
    /// @return kOk / kSensorTimeout / kUnhealthyInput / kSinkError / kStopped。
    LoopStatus run_once(GuidanceOutput *out = nullptr);

    /// @brief 按 cfg.loop_hz 阻塞运行，直到 request_stop 或错误。
    LoopStatus run();

    /// @brief 请求停止；下一次 run/run_once 返回 kStopped。
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

}  // namespace flight
}  // namespace guidance
}  // namespace pure_uav
