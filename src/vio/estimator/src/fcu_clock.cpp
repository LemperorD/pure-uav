#include "fcu_clock.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <vector>

namespace pure_uav {

namespace {
int64_t steadyNowNs() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}
bool subtractChecked(int64_t a, int64_t b, int64_t& result) {
    if ((b > 0 && a < std::numeric_limits<int64_t>::min() + b) || (b < 0 && a > std::numeric_limits<int64_t>::max() + b)) {
        return false;
    }
    result = a - b;
    return true;
}
}  // namespace

void FcuClock::clearLocked() {
    offsets_.clear();
    last_host_ns_ = 0;
    estimate_ns_ = 0;
    has_estimate_ = false;
    since_recompute_ = 0;
    last_imu_us_ = 0;
    unix_offset_us_.reset();
}

FcuClock::AddResult FcuClock::addSample(int64_t host_ns, uint64_t imu_timestamp_us) {
    std::scoped_lock lk(mtx_);

    if (host_ns < 0 || imu_timestamp_us == 0 || imu_timestamp_us > static_cast<uint64_t>(std::numeric_limits<int64_t>::max() / 1000)) {
        out_of_order_.fetch_add(1);
        return AddResult::kOutOfOrder;
    }
    if (last_imu_us_ != 0) {
        // 显著回退 = 飞控重启（boot 时间从 ~0 重来）。必须清空旧样本，
        // 否则新旧两个时间基的 offset 会混在同一个 5% 分位里，映射结果直接错几百秒。
        const bool rollback = imu_timestamp_us < last_imu_us_ && last_imu_us_ - imu_timestamp_us > static_cast<uint64_t>(kRollbackUs);
        const int64_t host_delta_us = std::max<int64_t>(0, (host_ns - last_host_ns_) / 1000);
        const bool forward_jump = imu_timestamp_us > last_imu_us_ && imu_timestamp_us - last_imu_us_ > static_cast<uint64_t>(host_delta_us + kRollbackUs);
        if (rollback || forward_jump) {
            clearLocked();
            timebase_resets_.fetch_add(1);
            last_imu_us_ = imu_timestamp_us;
            offsets_.push_back(host_ns - (static_cast<int64_t>(imu_timestamp_us) * 1000));
            last_host_ns_ = host_ns;
            return AddResult::kTimebaseReset;
        }
        // 重复帧 / 轻微乱序：丢弃，不污染偏移估计（主机时钟也不回退）
        if (imu_timestamp_us <= last_imu_us_) {
            out_of_order_.fetch_add(1);
            return AddResult::kOutOfOrder;
        }
    }

    offsets_.push_back(host_ns - (static_cast<int64_t>(imu_timestamp_us) * 1000));
    if (offsets_.size() > kMaxSamples) {
        offsets_.pop_front();
    }
    last_imu_us_ = imu_timestamp_us;
    last_host_ns_ = std::max(last_host_ns_, host_ns);
    if (!has_estimate_ || ++since_recompute_ >= 16) {
        recomputeLocked();
        since_recompute_ = 0;
    }
    return AddResult::kOk;
}

void FcuClock::recomputeLocked() const {
    if (offsets_.size() < kMinSamples) {
        return;
    }
    std::vector<int64_t> v(offsets_.begin(), offsets_.end());
    const size_t k = v.size() / 20;  // 5% 分位：抗串口延迟毛刺
    std::nth_element(v.begin(), v.begin() + k, v.end());
    estimate_ns_ = v[k];
    has_estimate_ = true;
}

bool FcuClock::valid() const {
    std::scoped_lock lk(mtx_);
    if (!has_estimate_ || offsets_.size() < kMinSamples) {
        return false;
    }
    return (steadyNowNs() - last_host_ns_) < kMaxAgeNs;
}

bool FcuClock::hostToFcuUs(int64_t host_ns, uint64_t& imu_us) const {
    std::scoped_lock lk(mtx_);
    return hostToFcuUsLocked(host_ns, imu_us);
}

bool FcuClock::hostToFcuUsLocked(int64_t host_ns, uint64_t& imu_us) const {
    if (!has_estimate_ || offsets_.size() < kMinSamples) {
        return false;
    }
    if ((steadyNowNs() - last_host_ns_) >= kMaxAgeNs) {
        return false;
    }
    int64_t ns = 0;
    if (host_ns < 0 || !subtractChecked(host_ns, estimate_ns_, ns) || ns < 0) {
        return false;
    }
    imu_us = static_cast<uint64_t>(ns / 1000);
    return true;
}

bool FcuClock::fcuToUnixUs(uint64_t imu_us, uint64_t& unix_us) const {
    std::scoped_lock lk(mtx_);
    if (imu_us == 0 || imu_us > static_cast<uint64_t>(std::numeric_limits<int64_t>::max() / 1000) || !has_estimate_ || offsets_.size() < kMinSamples || steadyNowNs() - last_host_ns_ >= kMaxAgeNs) {
        return false;
    }
    if (!unix_offset_us_.has_value()) {
        // 同时采样两种主机时钟，用单调时钟采样区间的中点减少读钟误差。
        // 只锁存一次，不能让串口到达抖动或后续系统校时改变 IMU 采样间隔。
        const int64_t before_ns = steadyNowNs();
        const int64_t now_unix_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        const int64_t after_ns = steadyNowNs();
        uint64_t reference_us = 0;
        if (now_unix_us <= 0 || !hostToFcuUsLocked(before_ns + (after_ns - before_ns) / 2, reference_us)) {
            return false;
        }
        unix_offset_us_ = now_unix_us - static_cast<int64_t>(reference_us);
    }
    int64_t mapped_us = 0;
    if (!subtractChecked(static_cast<int64_t>(imu_us), -*unix_offset_us_, mapped_us) || mapped_us <= 0) {
        return false;
    }
    unix_us = static_cast<uint64_t>(mapped_us);
    return true;
}

int64_t FcuClock::offsetNs() const {
    std::scoped_lock lk(mtx_);
    return estimate_ns_;
}

size_t FcuClock::sampleCount() const {
    std::scoped_lock lk(mtx_);
    return offsets_.size();
}

uint64_t FcuClock::lastImuTimestampUs() const {
    std::scoped_lock lk(mtx_);
    return last_imu_us_;
}

void FcuClock::reset() {
    std::scoped_lock lk(mtx_);
    clearLocked();
}

}  // namespace pure_uav
