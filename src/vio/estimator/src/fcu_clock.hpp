// 主机单调时钟 ↔ 飞控 IMU 时间戳的偏移估计。
//
// 做法：每收到一帧 HIGHRES_IMU，记录 (host_monotonic_ns, imu.timestamp_us)，
//       计算 offset = host_ns - fcu_us*1000。串口/调度延迟只会让 offset 偏大，
//       所以取最近窗口的 5% 分位数作为“时钟偏移 + 最小延迟”的估计。
//
// 相机帧到达主机时用同一 offset 反算：
//       fcu_us = (host_ns - offset_ns) / 1000
//
// 注意：相机帧的传输/解码延迟、IMU 的串口延迟都会引入固定偏差，这部分
// 由标定得到的 timeshift_cam_imu（或 OpenVINS 在线时间偏移）吸收。
//
// 【2026-10-29】飞控重启处理：
//   HIGHRES_IMU.time_usec 是**飞控 boot 以来**的微秒数。飞控重启后它从 ~0 重新开始，
//   而主机时钟继续前进。旧实现只往 offsets_ 里追加样本，重启后的样本会算出
//   "约 303 秒"这类完全错误的时间，而且 valid() 依旧返回 true —— 图像、IMU、输出位姿
//   就会落在两个不同的时间基准里。
//   现在 addSample() 返回 AddResult：检测到显著回退就自动清空旧样本、按新时间基重来，
//   调用方据此暂停发布并重建估计器（见 vio_live_main.cpp 的 onFcuTimebaseReset）。

#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>

namespace pure_uav {

class FcuClock {
public:
    // addSample() 的判定结果
    enum class AddResult {
        kOk,             // 正常样本，已纳入偏移估计
        kOutOfOrder,     // 重复或轻微回退（< kRollbackUs），丢弃，不影响估计
        kTimebaseReset,  // 飞控时间戳大幅回退 = 飞控重启/换了时间基；旧样本已清空
    };

    FcuClock() = default;

    // 加入一个 IMU 时间戳样本（host_ns 用 CLOCK_MONOTONIC）。
    // 返回 kTimebaseReset 时：内部已清空旧时间基的所有样本，本样本作为新时间基的
    // 第 1 帧保留；此时 valid() 为 false，需要重新累积 kMinSamples 帧才恢复。
    AddResult addSample(int64_t host_ns, uint64_t imu_timestamp_us);

    // 是否有足够样本且最近更新过（默认 2 秒内）。
    bool valid() const;

    // host 单调时钟 → IMU 时钟域（微秒）。valid() 为 false 时返回 false。
    bool hostToFcuUs(int64_t host_ns, uint64_t& imu_us) const;

    int64_t offsetNs() const;
    size_t sampleCount() const;

    // 统计（联调状态行/验收用）：被丢弃的乱序帧、检测到的时间基重置次数
    uint64_t outOfOrderCount() const { return out_of_order_.load(); }
    uint64_t timebaseResetCount() const { return timebase_resets_.load(); }
    uint64_t lastImuTimestampUs() const;

    // 手动清空（程序启动、串口重连等）。不清统计计数。
    void reset();

private:
    void recomputeLocked() const;
    void clearLocked();

    static constexpr size_t kMaxSamples = 600;  // 100Hz 下约 6 秒
    static constexpr size_t kMinSamples = 10;
    static constexpr int64_t kMaxAgeNs = 2000000000LL;  // 2 s
    // 时间戳回退超过它，才认为是"换了时间基"而不是乱序。
    // 200 Hz 流里真正的乱序通常只有几毫秒；飞控重启是回退到接近 0。
    static constexpr int64_t kRollbackUs = 100000;  // 100 ms

    mutable std::mutex mtx_;
    std::deque<int64_t> offsets_;  // host_ns - imu_us*1000
    int64_t last_host_ns_ = 0;
    mutable int64_t estimate_ns_ = 0;
    mutable bool has_estimate_ = false;
    size_t since_recompute_ = 0;
    uint64_t last_imu_us_ = 0;

    std::atomic<uint64_t> out_of_order_{0};
    std::atomic<uint64_t> timebase_resets_{0};
};

}  // namespace pure_uav
