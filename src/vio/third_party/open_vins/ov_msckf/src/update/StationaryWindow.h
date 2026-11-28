#ifndef OV_MSCKF_STATIONARY_WINDOW_H
#define OV_MSCKF_STATIONARY_WINDOW_H

#include <algorithm>
#include <deque>

namespace ov_msckf {

// Let occasional marginal IMU intervals age out of a visually stationary window.
// A motion veto or missing visual evidence invalidates the entire window.
class StationaryWindow {
public:
  enum class ImuEvidence { quiet, marginal, motion };

  explicit StationaryWindow(int frames) : frames_(std::max(1, frames)) {}

  bool observe(bool vision_still, ImuEvidence imu) {
    if (!vision_still || imu == ImuEvidence::motion) {
      reset();
      return false;
    }

    const bool quiet = imu == ImuEvidence::quiet;
    if (size() == frames_) {
      quiet_count_ -= samples_.front();
      samples_.pop_front();
    }
    samples_.push_back(quiet);
    quiet_count_ += quiet;
    return quiet && size() == frames_ && quiet_count_ >= required_quiet();
  }

  void reset() {
    samples_.clear();
    quiet_count_ = 0;
  }

  int size() const { return static_cast<int>(samples_.size()); }
  int quiet_count() const { return quiet_count_; }
  int required_quiet() const { return frames_ - frames_ / 5; }
  int capacity() const { return frames_; }

private:
  int frames_;
  int quiet_count_ = 0;
  std::deque<bool> samples_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_STATIONARY_WINDOW_H
