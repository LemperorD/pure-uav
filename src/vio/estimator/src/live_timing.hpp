#pragma once

#include <cmath>
#include <cstdint>

namespace pure_uav {

// 只平滑采样抖动。时间基跳变由 FcuClock/调用方先重建；丢帧时重新锚定，
// 不能拖着旧网格通过 PLL 追赶，从而制造数秒的虚假积分步长。
class ImuTimestampFilter {
  public:
    void reset() {
        grid_us_ = -1.0;
        prev_raw_us_ = last_output_us_ = 0;
    }
    bool filter(uint64_t raw_us, double nominal_dt_us, uint64_t& output_us) {
        if (raw_us == 0 || !std::isfinite(nominal_dt_us) || nominal_dt_us <= 0) return false;
        if (prev_raw_us_ != 0 && raw_us <= prev_raw_us_) return false;
        double next = static_cast<double>(raw_us);
        if (prev_raw_us_ != 0) {
            const double delta = static_cast<double>(raw_us - prev_raw_us_);
            prev_raw_us_ = raw_us;
            if (delta < 0.3 * nominal_dt_us) return false;
            if (delta <= 3.0 * nominal_dt_us) {
                next = grid_us_ + nominal_dt_us;
                next += 0.03 * (static_cast<double>(raw_us) - next);
            }
        }
        prev_raw_us_ = raw_us;
        const auto candidate = static_cast<uint64_t>(std::llround(next));
        if (candidate <= last_output_us_) return false;
        grid_us_ = next;
        last_output_us_ = output_us = candidate;
        return true;
    }

  private:
    double grid_us_ = -1.0;
    uint64_t prev_raw_us_ = 0;
    uint64_t last_output_us_ = 0;
};

inline bool imuCoversImage(uint64_t last_fed_imu_us, uint64_t image_us, double camera_to_imu_s) {
    const double required = static_cast<double>(image_us) + camera_to_imu_s * 1e6;
    return last_fed_imu_us != 0 && std::isfinite(required) && required >= 0.0 && static_cast<double>(last_fed_imu_us) > required;
}

// MAVLink 的 8 位 reset_counter 按模 256 递增，不能在 255 饱和。
inline uint8_t odometryResetCounter(uint64_t rebuilds) {
    return static_cast<uint8_t>(rebuilds);
}

}  // namespace pure_uav
