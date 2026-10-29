#include "fcu_clock.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

namespace pure_uav {

namespace {
int64_t steadyNowNs() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}
}  // namespace

void FcuClock::clearLocked() {
    offsets_.clear();
    last_host_ns_ = 0;
    estimate_ns_ = 0;
    has_estimate_ = false;
    since_recompute_ = 0;
    last_imu_us_ = 0;
}

FcuClock::AddResult FcuClock::addSample(int64_t host_ns, uint64_t imu_timestamp_us) {
    std::lock_guard<std::mutex> lk(mtx_);

    if (last_imu_us_ != 0 && imu_timestamp_us != 0) {
        // 显著回退 = 飞控重启（boot 时间从 ~0 重来）。必须清空旧样本，
        // 否则新旧两个时间基的 offset 会混在同一个 5% 分位里，映射结果直接错几百秒。
        if (imu_timestamp_us + static_cast<uint64_t>(kRollbackUs) < last_imu_us_) {
            clearLocked();
            timebase_resets_.fetch_add(1);
            last_imu_us_ = imu_timestamp_us;
            offsets_.push_back(host_ns - static_cast<int64_t>(imu_timestamp_us) * 1000);
            last_host_ns_ = host_ns;
            return AddResult::kTimebaseReset;
        }
        // 重复帧 / 轻微乱序：丢弃，不污染偏移估计（主机时钟也不回退）
        if (imu_timestamp_us <= last_imu_us_) {
            out_of_order_.fetch_add(1);
            return AddResult::kOutOfOrder;
        }
    }

    offsets_.push_back(host_ns - static_cast<int64_t>(imu_timestamp_us) * 1000);
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
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_estimate_ || offsets_.size() < kMinSamples) return false;
    return (steadyNowNs() - last_host_ns_) < kMaxAgeNs;
}

bool FcuClock::hostToFcuUs(int64_t host_ns, uint64_t& imu_us) const {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_estimate_ || offsets_.size() < kMinSamples) return false;
    if ((steadyNowNs() - last_host_ns_) >= kMaxAgeNs) return false;
    const int64_t us = (host_ns - estimate_ns_) / 1000;
    if (us < 0) return false;
    imu_us = static_cast<uint64_t>(us);
    return true;
}

int64_t FcuClock::offsetNs() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return estimate_ns_;
}

size_t FcuClock::sampleCount() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return offsets_.size();
}

uint64_t FcuClock::lastImuTimestampUs() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return last_imu_us_;
}

void FcuClock::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    clearLocked();
}

}  // namespace pure_uav
