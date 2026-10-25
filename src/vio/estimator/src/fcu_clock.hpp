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

#pragma once

#include <cstdint>
#include <deque>
#include <mutex>

namespace pure_uav {

class FcuClock {
public:
    FcuClock() = default;

    // 加入一个 IMU 时间戳样本（host_ns 用 CLOCK_MONOTONIC）。
    void addSample(int64_t host_ns, uint64_t imu_timestamp_us);

    // 是否有足够样本且最近更新过（默认 2 秒内）。
    bool valid() const;

    // host 单调时钟 → IMU 时钟域（微秒）。valid() 为 false 时返回 false。
    bool hostToFcuUs(int64_t host_ns, uint64_t& imu_us) const;

    int64_t offsetNs() const;
    size_t sampleCount() const;

    void reset();

private:
    void recomputeLocked() const;

    static constexpr size_t kMaxSamples = 600;  // 100Hz 下约 6 秒
    static constexpr size_t kMinSamples = 10;
    static constexpr int64_t kMaxAgeNs = 2000000000LL;  // 2 s

    mutable std::mutex mtx_;
    std::deque<int64_t> offsets_;  // host_ns - imu_us*1000
    int64_t last_host_ns_ = 0;
    mutable int64_t estimate_ns_ = 0;
    mutable bool has_estimate_ = false;
    size_t since_recompute_ = 0;
};

}  // namespace pure_uav
