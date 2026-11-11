#include "shm_camera_source.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <filesystem>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace pure_uav {
namespace {
void require(bool condition, const std::string& error) {
    if (!condition) throw std::runtime_error(error);
}
}  // namespace

ShmCameraSource::~ShmCameraSource() {
    stop();
}

bool ShmCameraSource::start(const Config& config, std::string* error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (accepting_) {
        if (error) *error = "VIO camera channel is already started";
        return false;
    }
    reader_.close();
    depth_info_ = {};
    error_.clear();
    last_seq_ = dropped_ = 0;
    discard_before_ns_ = last_frame_ns_ = 0;
    try {
        Config resolved = config;
        if (!resolved.config_dir.empty()) {
            const std::filesystem::path dir(resolved.config_dir);
            cv::FileStorage estimator((dir / resolved.estimator_config).string(), cv::FileStorage::READ);
            require(estimator.isOpened(), "Cannot read VIO estimator configuration");
            std::string chain;
            estimator["relative_config_imucam"] >> chain;
            require(!chain.empty(), "Missing relative_config_imucam in estimator configuration");
            cv::FileStorage calibration((dir / chain).string(), cv::FileStorage::READ);
            require(calibration.isOpened(), "Cannot read VIO camera calibration");
            std::vector<int> resolution;
            calibration["cam0"]["resolution"] >> resolution;
            if (resolution.size() == 2 && resolved.width == 0 && resolved.height == 0) {
                resolved.width = resolution[0];
                resolved.height = resolution[1];
            }
            require(resolution.size() == 2 && resolution[0] == resolved.width && resolution[1] == resolved.height, "VIO width/height do not match calibrated cam0.resolution");
        }
        require(resolved.width > 0 && resolved.height > 0 && resolved.fps > 0 && resolved.timeout_ms >= 0, "Invalid VIO camera dimensions, fps or timeout");
        std::string why;
        if (!reader_.open(resolved.channel, camera::ChannelType::vio, resolved.timeout_ms, &why)) throw std::runtime_error(why);
        require(!reader_.closing() && reader_.writerAlive(), "camera_server is stopped or its heartbeat has expired");
        const auto& stream = reader_.streamMeta();
        require(stream.width == static_cast<uint32_t>(resolved.width) && stream.height == static_cast<uint32_t>(resolved.height) && stream.fps == static_cast<uint32_t>(resolved.fps), "camera_server resolution/fps differ from VIO expectations; check the service camera.yaml and VIO calibration");
        VioDepthInfo depth;
        if (stream.has_depth) {
            depth.fx = stream.fx;
            depth.fy = stream.fy;
            depth.cx = stream.cx;
            depth.cy = stream.cy;
            depth.width = static_cast<int>(stream.depth_width);
            depth.height = static_cast<int>(stream.depth_height);
            std::copy(std::begin(stream.R_c2d), std::end(stream.R_c2d), depth.R_c2d);
            for (int i = 0; i < 3; ++i) depth.t_c2d_m[i] = stream.t_mm[i] * 1e-3;
        }
        require(!resolved.use_depth || (stream.has_depth && depth.valid()), "Depth requested but camera_server depth calibration is unavailable or invalid");
        depth_info_ = depth;
        config_ = resolved;
        session_ = reader_.session();
        generation_ = reader_.generation();
        accepting_ = true;
        if (error) error->clear();
        return true;
    } catch (const std::exception& e) {
        reader_.close();
        failLocked(std::string("VIO camera channel: ") + e.what());
        if (error) *error = error_;
        return false;
    }
}

void ShmCameraSource::failLocked(const std::string& error) const {
    error_ = error;
    accepting_ = false;
}

bool ShmCameraSource::aliveLocked() const {
    if (!accepting_) return false;
    if (reader_.closing() || !reader_.writerAlive()) {
        failLocked("camera_server stopped or camera frames timed out; restart the VIO consumer after the server");
    } else if (reader_.session() != session_ || reader_.generation() != generation_) {
        failLocked("camera_server session/generation changed; restart the VIO consumer to reinitialize its timebase");
    }
    return accepting_;
}

void ShmCameraSource::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        accepting_ = false;
        reader_.close();
        payload_.clear();
    }
    cv_.notify_all();
}

bool ShmCameraSource::pop(VioFrame& out, int timeout_ms) {
    using namespace std::chrono;
    const auto deadline = steady_clock::now() + milliseconds(std::max(0, timeout_ms));
    std::unique_lock<std::mutex> lock(mutex_);
    while (aliveLocked()) {
        camera::FrameMeta meta;
        uint64_t session = 0, generation = 0;
        if (reader_.readLatest(meta, payload_, session, generation)) {
            if (session != session_ || generation != generation_ || !aliveLocked()) {
                failLocked("camera_server session/generation changed while reading a frame");
                return false;
            }
            if (last_seq_ > 0 && meta.seq > last_seq_ + 1) dropped_ += meta.seq - last_seq_ - 1;
            last_seq_ = meta.seq;
            const uint64_t gray_bytes = static_cast<uint64_t>(config_.width) * config_.height;
            const uint64_t depth_bytes = static_cast<uint64_t>(depth_info_.width) * depth_info_.height * sizeof(uint16_t);
            if (meta.width != static_cast<uint32_t>(config_.width) || meta.height != static_cast<uint32_t>(config_.height) || meta.gray_bytes != gray_bytes || meta.color_bytes != 0 || meta.host_ns <= 0 ||
                (meta.depth_bytes != 0 && (!reader_.streamMeta().has_depth || !depth_info_.valid() || meta.depth_bytes != depth_bytes || !(meta.flags & camera::kFrameHasDepth)))) {
                failLocked("Malformed camera_server VIO frame dimensions, lengths or depth metadata");
                return false;
            }
            if (meta.host_ns < discard_before_ns_) {
                ++dropped_;
            } else {
                try {
                    VioFrame frame;
                    frame.gray = cv::Mat(config_.height, config_.width, CV_8UC1, payload_.data()).clone();
                    if (config_.use_depth && meta.depth_bytes != 0) frame.depth_mm = cv::Mat(depth_info_.height, depth_info_.width, CV_16UC1, payload_.data() + meta.gray_bytes).clone();
                    frame.host_ns = meta.host_ns;
                    frame.device_us = meta.device_us;
                    frame.depth_device_us = meta.depth_device_us;
                    frame.system_us = meta.sys_us;
                    last_frame_ns_ = frame.host_ns;
                    out = std::move(frame);
                    return true;
                } catch (const std::exception& e) {
                    failLocked(std::string("Camera frame copy: ") + e.what());
                    return false;
                }
            }
        }
        if (steady_clock::now() >= deadline) break;
        cv_.wait_until(lock, std::min(deadline, steady_clock::now() + milliseconds(1)));
    }
    return false;
}

void ShmCameraSource::discardBefore(int64_t stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    discard_before_ns_ = std::max(discard_before_ns_, stamp);
}
VioDepthInfo ShmCameraSource::depthInfo() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return depth_info_;
}
bool ShmCameraSource::running() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return aliveLocked();
}
uint64_t ShmCameraSource::dropped() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_;
}
int64_t ShmCameraSource::lastFrameHostNs() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_frame_ns_;
}
std::string ShmCameraSource::lastError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
}
}  // namespace pure_uav
