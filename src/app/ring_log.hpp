#pragma once
// =============================================================================
//  ring_log.hpp —— 机载黑匣子（固定环形缓冲，循环内零 I/O、零分配）
//
//  【和 csv_io 的区别】
//    `csv_io` 的 `InputRecorder` / `CsvAccelSink` 是**持续落盘**（每拍/限频写文件），
//    适合离线复盘；本模块是**常驻内存的环形缓冲**：
//      · 循环里只做 `push(const GuidanceOutput&)`（一次 ~100 B 的 POD 拷贝）；
//      · 只有上层**明确要求**时才 `dump_csv()`（异常现场 / 跑完 / 手动触发）；
//    ⇒ 出事时能回看"最后一口气"的前后 N 秒，而不用指望磁盘 I/O 在飞行中不卡。
//
//  【容量与时长】
//    kCap = 512 拍：50 Hz 下约 **10.2 s**。可根据机载内存调整（每拍 ~96 B ⇒ 约 49 KB）。
//
//  【不记录什么】
//    · 本拍没有产出指令（传感器超时 / 首帧未齐 / `kStop` 不发）时不会 push；
//    · 原始输入（像素/四元数）不在这里 —— 那属于 `csv_io` 的输入录制。
// =============================================================================

#include <cstdint>

#include "output.hpp"

namespace pure_uav {
namespace app {

// 一拍的关键量（固定 POD：无指针、无 STL，可安全 memcpy/落盘）
struct RingRecord {
    std::uint64_t boot_us = 0;     // 采样时刻（飞控 boot µs）
    std::uint8_t  ctrl    = 0;     // ControlMode
    std::uint8_t  mode    = 0;     // PureGuidanceMode
    std::uint8_t  health  = 0;     // InputHealth
    std::uint8_t  flags   = 0;     // bit0 valid / bit1 sat / bit2 hold_yaw / bit3 velocity
    double a_ned[3] = {0.0, 0.0, 0.0};   // 加速度（NED；kVelocity 时为 0）
    double v_ned[3] = {0.0, 0.0, 0.0};   // 速度（NED；kAcceleration 时为 0）
    double a_norm  = 0.0;
    double t_go    = 0.0;
    double Vc      = 0.0;
    double R       = 0.0;
    double lam_dot = 0.0;
};

class RingLog {
public:
    static constexpr int kCap = 512;   // ≈10.2 s @ 50 Hz

    // 记录一拍（循环内唯一调用；noexcept、零分配、无 I/O）
    void push(const GuidanceOutput &o) noexcept;

    void clear() noexcept {
        head_ = 0;
        n_    = 0;
    }

    bool empty() const noexcept { return n_ == 0; }
    int size() const noexcept { return static_cast<int>(n_ < kCap ? n_ : kCap); }
    std::uint64_t count() const noexcept { return n_; }   // 累计 push 次数（可 > kCap）

    // 逻辑顺序访问：0 = 最旧，size()-1 = 最新。越界夹到两端（方便 dump）。
    const RingRecord &at(int i) const noexcept;
    const RingRecord &newest() const noexcept;

    // 把最近 `seconds` 秒（按记录里的 boot_us）写成 CSV。
    //   seconds <= 0 ⇒ 全部；
    //   返回写出的数据行数；打不开文件返回 -1。
    // ⚠ 这是**唯一**做 I/O 的入口，绝不能在循环里调用。
    long dump_csv(const char *path, double seconds) const;

private:
    RingRecord    r_[kCap];
    int           head_ = 0;   // 下一个写入位置
    std::uint64_t n_    = 0;   // 累计写入次数
};

}  // namespace app
}  // namespace pure_uav
