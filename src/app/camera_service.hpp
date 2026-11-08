#pragma once
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>

#include "orbbec_camera.hpp"
#include "pure_uav/vio_runtime.hpp"

namespace pure_uav::app {
class CameraService final : public VioFrameSource {
  public:
    ~CameraService() override;
    bool start(const OrbbecCamera::Config& config, std::string* error, OrbbecCamera::ColorCallback color_callback = {});
    void stop();
    bool pop(VioFrame& out, int timeout_ms) override;
    VioDepthInfo depthInfo() const override;
    void discardBefore(int64_t host_ns) override;
    bool running() const;
    uint64_t dropped() const;
    int64_t lastFrameHostNs() const;
    std::string lastError() const;

  private:
    void enqueue(const cv::Mat& gray, const cv::Mat& depth, int64_t host_ns, int64_t device_us, int64_t depth_device_us);
    OrbbecCamera camera_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<VioFrame> frames_;
    VioDepthInfo depth_info_;
    bool accepting_ = false;
    uint64_t dropped_ = 0;
    int64_t discard_before_ns_ = 0, last_frame_ns_ = 0;
    std::string error_;
};
}  // namespace pure_uav::app
