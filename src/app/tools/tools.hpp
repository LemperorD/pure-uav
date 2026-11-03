#pragma once
// =============================================================================
//  tools.hpp —— 地面工具箱（CSV 录制/回放 + 合成世界 + 两种跑法）
//
//  定位：只在**地面/开发机**使用；机载二进制不需要链接本文件。
//  依赖：tools → flight_types + guidance_loop；不反向依赖 application。
//  格式/回放边界见 assets/docs/app/csv-format.md；黑匣子见 assets/docs/app/blackbox.md。
// =============================================================================

#include <cstdint>
#include <cstdio>

#include "app_params.hpp"     // AppParams（宿主侧配置）
#include "flight_types.hpp"   // flight::InsSample / flight::TargetSample / flight::GuidanceOutput / flight::CameraModel

namespace pure_uav {
namespace guidance {
namespace flight {
class GuidanceLoop;   // 前置声明：只在函数参数里用，头文件不拖整个 loop
}  // namespace flight
}  // namespace guidance
}  // namespace pure_uav

namespace pure_uav {
namespace app {

// ============================================================================
//  §1  CSV 一行 / 录制 / 回放
// ============================================================================

struct CsvRow {
    std::uint64_t tick = 0;
    flight::InsSample     ins;
    bool          ins_new = false;
    flight::TargetSample  tgt;
    bool          tgt_new = false;
};

class InputRecorder {
public:
    InputRecorder() = default;
    ~InputRecorder();
    InputRecorder(const InputRecorder &) = delete;
    InputRecorder &operator=(const InputRecorder &) = delete;

    bool open(const char *path);
    void on_ins(const flight::InsSample &s, bool new_frame) noexcept;
    void on_target(const flight::TargetSample &s, bool new_frame) noexcept;

    bool ok() const { return ok_; }
    bool is_open() const { return f_ != nullptr; }
    std::uint64_t rows() const { return rows_; }
    void close();

private:
    std::FILE    *f_ = nullptr;
    flight::InsSample     ins_{};
    bool          ins_new_ = false;
    std::uint64_t tick_ = 0;
    std::uint64_t rows_ = 0;
    bool          ok_ = true;
};

// 装饰器：内层数据源 + 顺带记录（不改变返回值语义）
class RecordingInsSource final : public flight::IInsSource {
public:
    RecordingInsSource(flight::IInsSource &inner, InputRecorder &rec) noexcept
        : inner_(inner), rec_(rec) {}
    bool poll(flight::InsSample &out) override;
    bool alive() const override { return inner_.alive(); }

private:
    flight::IInsSource    &inner_;
    InputRecorder &rec_;
    flight::InsSample      last_{};
};

class RecordingTargetSource final : public flight::ITargetSource {
public:
    RecordingTargetSource(flight::ITargetSource &inner, InputRecorder &rec) noexcept
        : inner_(inner), rec_(rec) {}
    bool poll(flight::TargetSample &out) override;
    bool alive() const override { return inner_.alive(); }

private:
    flight::ITargetSource &inner_;
    InputRecorder &rec_;
    flight::TargetSample   last_{};
};

class CsvReader {
public:
    CsvReader() = default;
    ~CsvReader();
    CsvReader(const CsvReader &) = delete;
    CsvReader &operator=(const CsvReader &) = delete;

    bool open(const char *path);
    void close();
    bool next(CsvRow &row);

    std::uint64_t rows() const { return rows_; }
    bool ok() const { return ok_; }
    bool is_open() const { return f_ != nullptr; }
    const char *error() const { return err_[0] ? err_ : nullptr; }

private:
    std::FILE    *f_ = nullptr;
    std::uint64_t rows_ = 0;
    bool          ok_ = true;
    char          err_[160] = {0};
};

// 一个对象同时实现两个接口：一行就是“一拍的两半”。
// 契约：每拍先 poll(flight::InsSample&) 再 poll(flight::TargetSample&)。
class CsvReplaySource final : public flight::IInsSource, public flight::ITargetSource {
public:
    explicit CsvReplaySource(CsvReader &reader) noexcept : r_(reader) {}

    bool poll(flight::InsSample &out) override;
    bool poll(flight::TargetSample &out) override;
    bool alive() const override { return !done_; }

    bool done() const { return done_; }
    std::uint64_t rows() const { return rows_; }
    const char *error() const { return r_.error(); }

private:
    bool advance();

    CsvReader    &r_;
    CsvRow        row_{};
    std::uint64_t rows_ = 0;
    bool          done_ = false;
    bool          have_row_ = false;
    bool          served_ins_ = false;
    bool          served_tgt_ = false;
};

// 指令落盘（离线复盘 / 回归测试 / 演示统计）
class CsvAccelSink final : public flight::ICommandSink {
public:
    explicit CsvAccelSink(const char *path);
    ~CsvAccelSink() override;
    CsvAccelSink(const CsvAccelSink &) = delete;
    CsvAccelSink &operator=(const CsvAccelSink &) = delete;

    bool send(const flight::GuidanceOutput &out) override;
    bool alive() const override { return f_ != nullptr && ok_; }

    void close();
    bool ok() const { return ok_; }
    bool is_open() const { return f_ != nullptr; }
    std::uint64_t rows() const { return rows_; }

private:
    std::FILE    *f_ = nullptr;
    std::uint64_t rows_ = 0;
    bool          ok_ = true;
};

// 一发两处：执行器 + 文件
class TeeSink final : public flight::ICommandSink {
public:
    TeeSink(flight::ICommandSink &a, flight::ICommandSink &b) noexcept : a_(a), b_(b) {}
    bool send(const flight::GuidanceOutput &out) override {
        const bool ra = a_.send(out);
        const bool rb = b_.send(out);
        return ra && rb;
    }
    bool alive() const override { return a_.alive() && b_.alive(); }

private:
    flight::ICommandSink &a_;
    flight::ICommandSink &b_;
};

// ============================================================================
//  §2  合成世界（零外部依赖的闭环演示 / 回归）
// ============================================================================

flight::CameraModel demo_camera();

struct SceneConfig {
    Vec3   p0{0.0, 0.0, -5.0};    // 起点（NED；z=-5 ⇒ 高 5 m）
    Vec3   v0{8.0, 0.0, 0.0};     // 初速
    Vec3   plate{40.0, 3.0, -5.0};// 装甲板位置
    double dt = 0.02;             // 积分步长
    std::uint64_t t0_us = 1000000ull;
};

class SyntheticWorld final : public flight::IInsSource,
                             public flight::ITargetSource,
                             public flight::ICommandSink {
public:
    explicit SyntheticWorld(const flight::CameraModel &cam, const SceneConfig &sc = SceneConfig{});

    void step();

    bool poll(flight::InsSample &out) override;
    bool alive() const override { return true; }
    bool poll(flight::TargetSample &out) override;
    bool send(const flight::GuidanceOutput &out) override;

    Vec3   pos() const { return p_; }
    Vec3   vel() const { return v_; }
    double true_range() const { return norm(plate_ - p_); }
    const flight::GuidanceOutput &last() const { return last_; }

    int sends() const { return sends_; }
    int sat() const { return sat_; }
    int hold() const { return hold_; }
    int search() const { return search_; }

private:
    static Vec3 mat3_mul_transpose(const double R[9], const Vec3 &v);

    flight::CameraModel cam_;
    Vec3   p_, v_, plate_;
    double q_[4] = {1.0, 0.0, 0.0, 0.0};
    double dt_;
    std::uint64_t t_us_;
    Vec3   a_cmd_{};
    flight::GuidanceOutput last_{};
    int sends_ = 0, sat_ = 0, hold_ = 0, search_ = 0;
};

struct ClosedLoopReport {
    int ticks = 0;
    double min_r = 1e18;
    int min_r_tick = -1;
    flight::LoopStatus stop = flight::LoopStatus::kOk;
    bool target_lost = false;
};

using TickHook = void (*)(void *user, int k,
                          const SyntheticWorld &world, const flight::GuidanceOutput &out);

ClosedLoopReport run_synthetic_closed_loop(SyntheticWorld &world, flight::GuidanceLoop &loop,
                                           int ticks, TickHook hook = nullptr,
                                           void *user = nullptr);

// ============================================================================
//  §3  两种跑法（从 main.cpp 搬来的薄封装）
// ============================================================================

struct SimOptions {
    int    ticks = 300;
    bool   verbose = true;
    bool   realtime = false;
    const char *record = nullptr;     // 输入录制 CSV
    const char *out = nullptr;        // 指令落盘 CSV
    const char *blackbox = nullptr;   // 黑匣子落盘 CSV
    double blackbox_sec = 5.0;        // 黑匣子窗口（秒；<=0 全部）
};

int run_synthetic(const SimOptions &opt, const AppParams &p);
int run_replay(const char *replay, const SimOptions &opt, const AppParams &p);

}  // namespace app
}  // namespace pure_uav
