#include "frame_publisher.hpp"

#include <cstring>
#include <utility>

namespace pure_uav::camera {

FramePublisher::~FramePublisher() {
    stop();
}

bool FramePublisher::create(const PublisherConfig& cfg, std::string* error) {
    stop();
    if (cfg.color_max_hz < 1 || cfg.color_max_hz > 60 || cfg.slots < 2) {
        if (error != nullptr) {
            *error = "camera publisher: color_max_hz must be 1..60 and slots >= 2";
        }
        return false;
    }
    cfg_ = cfg;
    const StreamMeta placeholder{};  // 深度内参等相机启动后由 activate() 补齐
    if (!vio_.create(cfg.vio_name, ChannelType::vio, placeholder, cfg.vio_payload_bytes, cfg.slots, error)) {
        return false;
    }
    if (!color_.create(cfg.color_name, ChannelType::color, placeholder, cfg.color_payload_bytes, cfg.slots, error)) {
        vio_.close();
        return false;
    }
    last_color_ns_ = 0;
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

void FramePublisher::activate(const StreamMeta& meta) noexcept {
    vio_.setStreamMeta(meta);
    color_.setStreamMeta(meta);
    setSession(1, 0);
    active_.store(true, std::memory_order_release);
    vio_.markReady();
    color_.markReady();
}

void FramePublisher::publish(const OrbbecCamera::CameraFrames& frames) noexcept {
    if (!active_.load(std::memory_order_acquire)) {
        return;
    }
    try {
        // 共享内存只接受连续字节；SDK 解码结果通常连续，非连续时才 clone 一次。
        cv::Mat gray_copy;
        cv::Mat depth_copy;
        cv::Mat color_copy;
        const cv::Mat* gray = &frames.gray;
        const cv::Mat* depth = &frames.depth_mm;
        const cv::Mat* color = &frames.color_encoded;
        if (!gray->empty() && !gray->isContinuous()) {
            gray_copy = gray->clone();
            gray = &gray_copy;
        }
        if (!depth->empty() && !depth->isContinuous()) {
            depth_copy = depth->clone();
            depth = &depth_copy;
        }
        if (!color->empty() && !color->isContinuous()) {
            color_copy = color->clone();
            color = &color_copy;
        }

        const size_t gray_bytes = gray->empty() ? 0 : gray->total() * gray->elemSize();
        const size_t depth_bytes = depth->empty() ? 0 : depth->total() * depth->elemSize();
        const size_t color_bytes = color->empty() ? 0 : color->total() * color->elemSize();

        FrameMeta vio_meta;
        vio_meta.seq = frames.seq;
        vio_meta.host_ns = frames.host_ns;
        vio_meta.device_us = frames.device_us;
        vio_meta.sys_us = frames.sys_us;
        vio_meta.depth_device_us = frames.depth_device_us;
        vio_meta.gray_bytes = static_cast<uint32_t>(gray_bytes);
        vio_meta.depth_bytes = static_cast<uint32_t>(depth_bytes);
        vio_meta.width = static_cast<uint32_t>(frames.width);
        vio_meta.height = static_cast<uint32_t>(frames.height);
        vio_meta.flags = frames.has_depth ? kFrameHasDepth : 0U;
        if (gray_bytes == 0 || !vio_.publish(vio_meta, gray->data, depth_bytes != 0 ? depth->data : nullptr, nullptr)) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
        } else {
            vio_frames_.fetch_add(1, std::memory_order_relaxed);
        }

        // 彩色通道按 max_hz 限频：只复制被选中的编码字节，通常 MJPG 无需先解成 BGR。
        const int64_t period_ns = 1000000000LL / cfg_.color_max_hz;
        if (last_color_ns_ != 0 && frames.host_ns - last_color_ns_ < period_ns) {
            color_rate_limited_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (color_bytes == 0) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        FrameMeta color_meta;
        color_meta.seq = frames.seq;
        color_meta.host_ns = frames.host_ns;
        color_meta.device_us = frames.device_us;
        color_meta.sys_us = frames.sys_us;
        color_meta.depth_device_us = frames.depth_device_us;
        color_meta.color_bytes = static_cast<uint32_t>(color_bytes);
        color_meta.color_encoding = static_cast<uint32_t>(frames.encoding);
        color_meta.width = static_cast<uint32_t>(frames.width);
        color_meta.height = static_cast<uint32_t>(frames.height);
        if (!color_.publish(color_meta, nullptr, nullptr, color->data)) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        last_color_ns_ = frames.host_ns;
        color_frames_.fetch_add(1, std::memory_order_relaxed);
    } catch (...) {
        // 采集回调绝不能因一帧异常退出；记数后继续。
        dropped_.fetch_add(1, std::memory_order_relaxed);
    }
}

void FramePublisher::setSession(uint64_t session, uint64_t generation) noexcept {
    vio_.setSession(session, generation);
    color_.setSession(session, generation);
}

void FramePublisher::markClosing() noexcept {
    vio_.markClosing();
    color_.markClosing();
}

bool FramePublisher::active() const noexcept {
    return active_.load(std::memory_order_acquire);
}

PublisherStats FramePublisher::stats() const noexcept {
    PublisherStats out;
    out.vio_frames = vio_frames_.load(std::memory_order_relaxed);
    out.color_frames = color_frames_.load(std::memory_order_relaxed);
    out.color_rate_limited = color_rate_limited_.load(std::memory_order_relaxed);
    out.dropped = dropped_.load(std::memory_order_relaxed);
    return out;
}

uint64_t FramePublisher::channelDropped() const noexcept {
    return vio_.dropped() + color_.dropped();
}

void FramePublisher::stop() noexcept {
    active_.store(false, std::memory_order_release);
    markClosing();
    vio_.close();
    color_.close();
}

}  // namespace pure_uav::camera
