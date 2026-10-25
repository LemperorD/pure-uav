#include "pure_uav/vio_health.hpp"

#include <algorithm>
#include <deque>

namespace pure_uav {

void HealthMonitor::pushWindow(double t, bool updated) {
    win_time_.push_back(t);
    win_updated_.push_back(updated ? 1 : 0);
    const double lo = t - cfg_.update_rate_window_s;
    size_t drop = 0;
    while (drop < win_time_.size() && win_time_[drop] < lo) ++drop;
    if (drop > 0) {
        win_time_.erase(win_time_.begin(), win_time_.begin() + drop);
        win_updated_.erase(win_updated_.begin(), win_updated_.begin() + drop);
    }
}

// 窗口没填满时返回 -1（不评判），避免开机就误判
double HealthMonitor::updateRate() const {
    if (win_time_.empty()) return -1.0;
    const double span = win_time_.back() - win_time_.front();
    if (span < 0.8 * cfg_.update_rate_window_s) return -1.0;
    int n = 0;
    for (auto v : win_updated_) n += v;
    return static_cast<double>(n) / static_cast<double>(win_updated_.size());
}

const char* toString(VioHealthState s) {
    switch (s) {
        case VioHealthState::OK: return "OK";
        case VioHealthState::DEGRADED: return "DEGRADED";
        case VioHealthState::LOST: return "LOST";
        default: return "UNKNOWN";
    }
}

HealthMonitor::HealthMonitor(const HealthConfig& cfg) : cfg_(cfg) {}

void HealthMonitor::configure(const HealthConfig& cfg) {
    cfg_ = cfg;
    reset();
}

void HealthMonitor::reset() {
    win_time_.clear();
    win_updated_.clear();
    state_ = VioHealthState::UNKNOWN;
    start_time_ = last_time_ = last_update_time_ = -1.0;
    bad_since_ = good_since_ = state_changed_at_ = -1.0;
    features_ = 0;
    updated_ = false;
    need_recover_ = false;
}

void HealthMonitor::update(double time_s, int features, bool visual_updated) {
    if (!cfg_.enabled) {
        state_ = VioHealthState::OK;
        features_ = features;
        updated_ = visual_updated;
        return;
    }

    if (start_time_ < 0.0) {
        start_time_ = time_s;
        last_time_ = time_s;
        state_ = VioHealthState::OK;
        state_changed_at_ = time_s;
        if (visual_updated) last_update_time_ = time_s;
        features_ = features;
        updated_ = visual_updated;
        return;
    }

    features_ = features;
    updated_ = visual_updated;
    last_time_ = time_s;
    if (visual_updated) {
        last_update_time_ = time_s;
    }

    pushWindow(time_s, visual_updated);
    const double rate = updateRate();
    const bool low_rate = (rate >= 0.0) && (rate < cfg_.min_update_rate);
    const bool few_features = features < cfg_.min_features;
    const double since_update = (last_update_time_ < 0.0) ? (time_s - start_time_)
                                                          : (time_s - last_update_time_);

    // 特征不足的计时
    if (few_features) {
        if (bad_since_ < 0.0) bad_since_ = time_s;
    } else {
        bad_since_ = -1.0;
    }
    const double bad_for = (bad_since_ < 0.0) ? 0.0 : (time_s - bad_since_);

    const bool should_be_lost =
        low_rate || (since_update > cfg_.lost_after_s) || (bad_for > cfg_.lost_after_s);
    const bool should_be_degraded =
        few_features || low_rate || (since_update > cfg_.degrade_after_s);

    switch (state_) {
        case VioHealthState::UNKNOWN:
            state_ = VioHealthState::OK;
            state_changed_at_ = time_s;
            break;

        case VioHealthState::OK:
            if (should_be_lost) {
                state_ = VioHealthState::LOST;
                state_changed_at_ = time_s;
                ++lost_count_;
                need_recover_ = true;
                good_since_ = -1.0;
            } else if (should_be_degraded) {
                state_ = VioHealthState::DEGRADED;
                state_changed_at_ = time_s;
            }
            break;

        case VioHealthState::DEGRADED:
            if (should_be_lost) {
                state_ = VioHealthState::LOST;
                state_changed_at_ = time_s;
                ++lost_count_;
                need_recover_ = true;
                good_since_ = -1.0;
            } else if (!should_be_degraded) {
                state_ = VioHealthState::OK;
                state_changed_at_ = time_s;
            }
            break;

        case VioHealthState::LOST: {
            // 恢复候选：特征回到 min_features_recover 以上，且近期有更新
            const bool healthy = (features >= cfg_.min_features_recover) &&
                                 (!(rate >= 0.0) || rate >= cfg_.min_update_rate_recover) &&
                                 (since_update <= cfg_.lost_after_s);
            if (!healthy) {
                good_since_ = -1.0;
                break;
            }
            if (good_since_ < 0.0) good_since_ = time_s;
            if (time_s - good_since_ >= cfg_.recover_after_s) {
                // 状态在这里先切回 OK；如果要重建估计器，上层调 notify_recovered()
                state_ = VioHealthState::OK;
                state_changed_at_ = time_s;
                bad_since_ = -1.0;
                good_since_ = -1.0;
            }
            break;
        }
    }
}

void HealthMonitor::notify_recovered() {
    ++recoveries_;
    need_recover_ = false;
    state_ = VioHealthState::OK;
}

HealthReport HealthMonitor::report(double now_s) const {
    HealthReport r;
    r.state = state_;
    r.publishable = publishable();
    r.features = features_;
    r.updated_this_frame = updated_;
    r.since_update_s = (last_update_time_ < 0.0)
                           ? (now_s - start_time_)
                           : std::max(0.0, now_s - last_update_time_);
    r.since_state_change_s = (state_changed_at_ < 0.0) ? 0.0 : (now_s - state_changed_at_);
    r.recoveries = recoveries_;
    r.lost_count = lost_count_;
    return r;
}

// 【2026-10-22】物理合理性违规：直接判 LOST 并保持，直到外部重建。
// 这是"安全阀"，不参与精度优化——它只保证不发明显错误的位姿。
void HealthMonitor::mark_implausible(double time_s) {
    if (!cfg_.enabled) return;
    state_ = VioHealthState::LOST;
    state_changed_at_ = time_s;
    last_time_ = time_s;
    updated_ = false;
    need_recover_ = true;
    // 清空更新窗口：恢复时需要重新攒满一段正常更新
    win_time_.clear();
    win_updated_.clear();
    last_update_time_ = -1.0;
}

}  // namespace pure_uav
