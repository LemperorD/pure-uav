// 相机服务侧发布器：把一份完整帧集拆成 VIO / 彩色两条独立共享内存通道。
//
// 分开的理由：VIO 每帧约 6 MB（1920x1080 灰度 + CV_16U 深度，30 fps），
// 检测通道携带同帧深度与 MJPG，限频 10 Hz；两个消费者互不占用对方槽位。
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "camera_shm.hpp"
#include "orbbec_camera.hpp"

namespace pure_uav::camera {

struct PublisherConfig {
    std::string vio_name = kVioChannel;
    std::string color_name = kColorChannel;
    int slots = kDefaultSlotCount;
    size_t vio_payload_bytes = kDefaultVioPayloadBytes;
    size_t color_payload_bytes = kDefaultColorPayloadBytes;
    int color_max_hz = 10;
};

struct PublisherStats {
    uint64_t vio_frames = 0;
    uint64_t color_frames = 0;
    uint64_t color_rate_limited = 0;
    uint64_t dropped = 0;
};

class FramePublisher {
  public:
    FramePublisher() = default;
    ~FramePublisher();
    FramePublisher(const FramePublisher&) = delete;
    FramePublisher& operator=(const FramePublisher&) = delete;

    /// 创建两条通道（此时 ready=0，消费者会在 open() 中等待）。
    bool create(const PublisherConfig& cfg, std::string* error);

    /// 相机启动、拿到 SDK 深度内参后调用：写静态元数据 → 置初始会话 → markReady。
    void activate(const StreamMeta& meta) noexcept;

    /// 采集回调内同步调用：只做 memcpy + 原子操作，禁止推理/日志/阻塞。
    void publish(const OrbbecCamera::CameraFrames& frames) noexcept;

    void setSession(uint64_t session, uint64_t generation) noexcept;
    void markClosing() noexcept;
    [[nodiscard]] bool active() const noexcept;
    [[nodiscard]] PublisherStats stats() const noexcept;
    [[nodiscard]] uint64_t channelDropped() const noexcept;
    void stop() noexcept;

  private:
    PublisherConfig cfg_;
    ShmChannelWriter vio_;
    ShmChannelWriter color_;
    std::atomic<bool> active_{false};
    std::atomic<uint64_t> vio_frames_{0};
    std::atomic<uint64_t> color_frames_{0};
    std::atomic<uint64_t> color_rate_limited_{0};
    std::atomic<uint64_t> dropped_{0};
    int64_t last_color_ns_ = 0;  // 仅采集线程访问
};

}  // namespace pure_uav::camera
