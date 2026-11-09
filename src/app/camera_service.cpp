#include "camera_service.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <utility>

namespace pure_uav::app {
CameraService::~CameraService() {
    stop();
}
bool CameraService::start(const OrbbecCamera::Config& config, std::string* error, OrbbecCamera::ColorCallback color_callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (accepting_) {
            if (error) *error = "CameraService is already started";
            return false;
        }
        frames_.clear();
        depth_info_ = {};
        error_.clear();
        dropped_ = 0;
        last_frame_ns_ = 0;
        accepting_ = true;
    }
    try {
        std::string why;
        auto capture_config = config;
        capture_config.callback_device_timestamps = true;
        if (!camera_.start(capture_config, [this](const cv::Mat& gray, const cv::Mat& depth, int64_t host, int64_t device, int64_t depth_device) { enqueue(gray, depth, host, device, depth_device); }, &why, std::move(color_callback))) {
            stop();
            std::lock_guard<std::mutex> lock(mutex_);
            error_ = why;
            if (error) *error = error_;
            return false;
        }
        const auto info = camera_.depthInfo();
        VioDepthInfo d;
        d.fx = info.fx;
        d.fy = info.fy;
        d.cx = info.cx;
        d.cy = info.cy;
        d.width = info.width;
        d.height = info.height;
        std::copy(info.R_c2d, info.R_c2d + 9, d.R_c2d);
        for (int i = 0; i < 3; ++i) d.t_c2d_m[i] = info.t_mm[i] * 1e-3;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            depth_info_ = d;
        }
        if (config.use_depth && !d.valid()) {
            stop();
            std::lock_guard<std::mutex> lock(mutex_);
            error_ = "Depth requested but depth calibration is unavailable";
            if (error) *error = error_;
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        stop();
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = std::string("CameraService start: ") + e.what();
        if (error) *error = error_;
        return false;
    }
}
void CameraService::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        accepting_ = false;
        frames_.clear();
    }
    cv_.notify_all();
    camera_.stop();
}
void CameraService::enqueue(const cv::Mat& gray, const cv::Mat& depth, int64_t host, int64_t device, int64_t depth_device) {
    try {
        VioFrame frame{gray.clone(), depth.clone(), host, device, depth_device};
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!accepting_ || host < discard_before_ns_) return;
            if (frames_.size() == 5) {
                frames_.pop_front();
                ++dropped_;
            }
            frames_.push_back(std::move(frame));
            last_frame_ns_ = host;
        }
        cv_.notify_one();
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = std::string("Camera frame copy: ") + e.what();
        accepting_ = false;
        cv_.notify_all();
    }
}
bool CameraService::pop(VioFrame& out, int timeout_ms) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, std::chrono::milliseconds(std::max(0, timeout_ms)), [this] { return !frames_.empty() || !accepting_; });
    if (frames_.empty()) return false;
    out = std::move(frames_.front());
    frames_.pop_front();
    return true;
}
void CameraService::discardBefore(int64_t stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    discard_before_ns_ = std::max(discard_before_ns_, stamp);
    while (!frames_.empty() && frames_.front().host_ns < discard_before_ns_) {
        frames_.pop_front();
        ++dropped_;
    }
}
VioDepthInfo CameraService::depthInfo() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return depth_info_;
}
bool CameraService::running() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return accepting_ && camera_.running();
}
uint64_t CameraService::dropped() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_;
}
int64_t CameraService::lastFrameHostNs() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_frame_ns_;
}
std::string CameraService::lastError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
}
}  // namespace pure_uav::app
