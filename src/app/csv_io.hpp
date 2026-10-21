#pragma once
// =============================================================================
//  CSV 录制 / 回放（黑匣子 + 离线复盘 + 回归测试）
//
//  【要解决什么问题】
//    真机一旦打偏，那个瞬间的输入（飞控帧 + 视觉帧）只存在于当场，
//    **没有任何办法重现**。本文件提供"录 → 放"：
//
//      真实源 ──► RecordingInsSource / RecordingTargetSource ──► GuidanceLoop
//                        │（装饰器，每拍落一行）
//                        ▼
//                    某个 .csv  ──►  CsvReplaySource  ──► 同一个 GuidanceLoop
//
//  ⇒ 录和放用的是**同一份 loop 代码、同一份制导核**，所以：
//      · 复盘：拿一份飞行数据在笔记本上逐拍重算，不装 Gazebo / 不装 PX4；
//      · 调参：同一份数据跑 PPN/TPN、N=3/4、a_max=10/15，直接对比指令差异；
//      · 交接：把 CSV 发给队友，对方编译就能复现你说的现象。
//
//  【文件格式（一行 = 主循环一拍）】
//    `#` 开头是注释（表头就是注释，读取时跳过）；字段全为数值、逗号分隔：
//
//      tick, ins_new, ins_boot_us, has_pos, has_att,
//      p_n, p_e, p_d, v_n, v_e, v_d, qw, qx, qy, qz,
//      tgt_new, tgt_boot_us, u_px, v_px, has_box, w_px, h_px, conf, class_id,
//      present, has_range, range_m, has_vT, vTn, vTe, vTd
//
//    · `ins_new / tgt_new` = 该拍数据源是否**真的给了一帧新数据**（0 表示零阶保持）。
//      ★ 这两个标志必须原样录制：回放时"没有新目标帧 ⇒ 重发上一拍指令"的行为
//        完全依赖它，丢掉就变成了每秒 50 次重算，ω 会被系统性改变。
//    · `boot_us` = **飞控 boot 时钟 µs**（不是本机时钟），见 sample.hpp 的时间戳约定。
//    · 浮点写 17 位有效数字 ⇒ 读回来**逐位一致**（回归测试依赖这一点）。
//
//  【⚠ 回放不重演"降级时机"】
//    GuidanceLoop 的超时/保持窗口用的是**本机单调时钟**；回放通常全速跑，
//    本机时间几乎不前进 ⇒ `sensor_timeout` / `hold_last_max` 不会触发。
//    ⇒ 回放验证的是**算得对不对**（数值、方向、模式），不是**什么时候降级**。
//      要验降级要么加 `--realtime` 按真实节拍放，要么用 test_guidance §16 的桩。
// =============================================================================

#include <cstdint>
#include <cstdio>

#include "command_sink.hpp"    // ICommandSink / GuidanceOutput
#include "sample.hpp"          // InsSample / TargetSample
#include "sensor_source.hpp"   // IInsSource / ITargetSource

namespace pure_uav {
namespace app {

// ---------------------------------------------------------------- 一行
struct CsvRow {
    std::uint64_t tick = 0;        // 拍序号（从 0 起）
    InsSample     ins;             // 该拍的飞控帧（零阶保持后的值）
    bool          ins_new = false; // 该拍是否有**新的**飞控帧
    TargetSample  tgt;             // 该拍的目标帧（零阶保持后的值）
    bool          tgt_new = false; // 该拍是否有**新的**目标帧
};

// ---------------------------------------------------------------- 录制
//
// 用法（主循环代码一行不用改）：
//     InputRecorder rec;
//     rec.open("flight.csv");
//     RecordingInsSource    ins(raw_ins, rec);
//     RecordingTargetSource tgt(raw_tgt, rec);
//     GuidanceLoop loop(params, cfg, ins, tgt, sink);
class InputRecorder {
public:
    InputRecorder() = default;
    ~InputRecorder();
    InputRecorder(const InputRecorder &) = delete;
    InputRecorder &operator=(const InputRecorder &) = delete;

    // 打开并写表头。false = 打不开（目录不存在 / 无权限）。
    bool open(const char *path);

    // 每拍一次（不管有没有新帧，都要调，保证行对齐）。
    void on_ins(const InsSample &s, bool new_frame) noexcept;

    // 每拍一次；**在这里落盘一行**（主循环先 poll 飞控、再 poll 目标）。
    void on_target(const TargetSample &s, bool new_frame) noexcept;

    // 全程无 I/O 错误（open 成功 + 每次写成功 + close 成功）。
    // ⚠ 与"现在是否打开"无关：close() 之后仍然可以查这次录制是否成功。
    bool ok() const { return ok_; }
    bool is_open() const { return f_ != nullptr; }
    std::uint64_t rows() const { return rows_; }
    void close();

private:
    std::FILE    *f_ = nullptr;
    InsSample     ins_{};
    bool          ins_new_ = false;
    std::uint64_t tick_ = 0;
    std::uint64_t rows_ = 0;
    bool          ok_ = true;
};

// 装饰器：内层数据源 + 顺带记录（不改变返回值语义）
class RecordingInsSource final : public IInsSource {
public:
    RecordingInsSource(IInsSource &inner, InputRecorder &rec) noexcept
        : inner_(inner), rec_(rec) {}
    bool poll(InsSample &out) override;
    bool alive() const override { return inner_.alive(); }

private:
    IInsSource    &inner_;
    InputRecorder &rec_;
    InsSample      last_{};   // 零阶保持值：没有新帧时也要写进 CSV，否则行会错位
};

class RecordingTargetSource final : public ITargetSource {
public:
    RecordingTargetSource(ITargetSource &inner, InputRecorder &rec) noexcept
        : inner_(inner), rec_(rec) {}
    bool poll(TargetSample &out) override;
    bool alive() const override { return inner_.alive(); }

private:
    ITargetSource &inner_;
    InputRecorder &rec_;
    TargetSample   last_{};
};

// ---------------------------------------------------------------- 读取
class CsvReader {
public:
    CsvReader() = default;
    ~CsvReader();
    CsvReader(const CsvReader &) = delete;
    CsvReader &operator=(const CsvReader &) = delete;

    bool open(const char *path);

    // 读下一行（跳过注释/空行）。false = 到末尾或解析出错。
    bool next(CsvRow &row);

    std::uint64_t rows() const { return rows_; }
    bool ok() const { return ok_; }
    const char *error() const { return err_[0] ? err_ : nullptr; }

private:
    std::FILE    *f_ = nullptr;
    std::uint64_t rows_ = 0;
    bool          ok_ = true;
    char          err_[160] = {0};
};

// ---------------------------------------------------------------- 回放
//
// 一个对象同时实现两个接口：录制时"一拍的飞控帧 + 目标帧"本来就是一行的两半，
// 硬拆成两个类反而要共享文件指针。主循环对 ins/tgt 各调一次 poll，顺序见下。
//
// ★ 契约：每拍先 `poll(InsSample&)` 再 `poll(TargetSample&)`（= run_once 的顺序）。
//   本类内部用"两个都交付过才推进下一行"来兜住顺序（乱序调用也不会丢帧）。
class CsvReplaySource final : public IInsSource, public ITargetSource {
public:
    explicit CsvReplaySource(CsvReader &reader) noexcept : r_(reader) {}

    bool poll(InsSample &out) override;
    bool poll(TargetSample &out) override;
    bool alive() const override { return !done_; }

    bool done() const { return done_; }              // 文件读完
    std::uint64_t rows() const { return rows_; }     // 已回放行数
    const char *error() const { return r_.error(); }

private:
    bool advance();                                  // 读下一行；末尾返回 false

    CsvReader    &r_;
    CsvRow        row_{};
    std::uint64_t rows_ = 0;
    bool          done_ = false;
    bool          have_row_ = false;
    bool          served_ins_ = false;
    bool          served_tgt_ = false;
};

// ---------------------------------------------------------------- 输出落盘
//
// 只写文件、不连任何硬件：用于离线复盘 / 回归测试 / 演示统计。
// 列的语义见 csv_io.cpp 里的 `kOutputHeader`：
//   ctrl 0=加速度 1=零速度；mode 0=PN 1=APN 2=HOLD 3=SEARCH；health 见 output.hpp
//
// ⚠ 实飞不要用它当黑匣子（stdio 落盘有机会阻塞主循环）；实飞用 MavlinkAccelSink
//   + 独立线程环形缓冲。
class CsvAccelSink final : public ICommandSink {
public:
    explicit CsvAccelSink(const char *path);
    ~CsvAccelSink() override;
    CsvAccelSink(const CsvAccelSink &) = delete;
    CsvAccelSink &operator=(const CsvAccelSink &) = delete;

    bool send(const GuidanceOutput &out) override;
    bool alive() const override { return f_ != nullptr && ok_; }

    bool ok() const { return f_ != nullptr && ok_; }
    std::uint64_t rows() const { return rows_; }

private:
    std::FILE    *f_ = nullptr;
    std::uint64_t rows_ = 0;
    bool          ok_ = true;
};

// ---------------------------------------------------------------- 一发两处
//
// 场景：合成世界既是"执行器"（要拿指令去积分），又想同时把指令落盘复盘。
class TeeSink final : public ICommandSink {
public:
    TeeSink(ICommandSink &a, ICommandSink &b) noexcept : a_(a), b_(b) {}
    bool send(const GuidanceOutput &out) override {
        const bool ra = a_.send(out);
        const bool rb = b_.send(out);   // 两边都发（不短路：落盘失败也要继续控制）
        return ra && rb;
    }
    bool alive() const override { return a_.alive() && b_.alive(); }

private:
    ICommandSink &a_;
    ICommandSink &b_;
};

}  // namespace app
}  // namespace pure_uav
