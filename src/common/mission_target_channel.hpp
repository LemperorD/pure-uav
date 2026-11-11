#pragma once

#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace pure_uav::app {

// 只在同机同构建的父子进程间传输。采集时刻是 CLOCK_MONOTONIC；不解析 stdout。
struct MissionTargetPacket {
    std::uint32_t version = 1;
    std::uint32_t present = 0;
    std::int64_t captured_ns = 0;
    std::uint64_t sequence = 0;
    double u_px = 0, v_px = 0, w_px = 0, h_px = 0, confidence = 0;
    std::int32_t class_id = -1;
    std::uint32_t has_range = 0;
    double range_m = 0;  // 后续测距适配器填写；当前纯检测不伪造距离。
};
static_assert(std::is_trivially_copyable_v<MissionTargetPacket>);

inline bool publishMissionTarget(int fd, const MissionTargetPacket& packet) {
    // 队列满直接丢本帧，不能阻塞推理线程；父进程按采样年龄丢旧帧。
    return send(fd, &packet, sizeof(packet), MSG_DONTWAIT | MSG_NOSIGNAL) == sizeof(packet);
}

class MissionTargetChannel {
  public:
    ~MissionTargetChannel() {
        if (receive_fd_ >= 0) {
            close(receive_fd_);
        }
        if (publish_fd_ >= 0) {
            close(publish_fd_);
        }
    }
    MissionTargetChannel() = default;
    MissionTargetChannel(const MissionTargetChannel&) = delete;
    MissionTargetChannel& operator=(const MissionTargetChannel&) = delete;
    bool open() {
        if (receive_fd_ >= 0) {
            return true;
        }
        int pair[2];
        if (socketpair(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0, pair) != 0) {
            return false;
        }
        receive_fd_ = pair[0];
        publish_fd_ = pair[1];
        return true;
    }
    int publisherFd() const {
        return publish_fd_;
    }
    bool latest(MissionTargetPacket& out, std::int64_t now_ns, std::int64_t max_age_ns) {
        bool found = false;
        // 每拍工作量有上限；只交付新鲜、完整、合法的最近一帧。
        for (int count = 0; count < 64; ++count) {
            MissionTargetPacket packet{};
            const auto length = recv(receive_fd_, &packet, sizeof(packet), MSG_DONTWAIT | MSG_TRUNC);
            if (length < 0) {
                if (errno == EINTR) {
                    continue;
                }
                break;
            }
            if (length != sizeof(packet) || packet.version != 1 || packet.present > 1 || packet.has_range > 1 || packet.captured_ns <= last_captured_ns_ || packet.captured_ns > now_ns || now_ns - packet.captured_ns > max_age_ns) {
                continue;
            }
            if (packet.present != 0 &&
                (!std::isfinite(packet.u_px) || !std::isfinite(packet.v_px) || !std::isfinite(packet.w_px) || !std::isfinite(packet.h_px) || packet.w_px <= 0 || packet.h_px <= 0 || !std::isfinite(packet.confidence) || packet.confidence < 0 || packet.confidence > 1 || packet.class_id < 0)) {
                continue;
            }
            if (packet.has_range != 0 && (!std::isfinite(packet.range_m) || packet.range_m <= 0)) {
                continue;
            }
            last_captured_ns_ = packet.captured_ns;
            out = packet;
            found = true;
        }
        return found;
    }

  private:
    int receive_fd_ = -1, publish_fd_ = -1;
    std::int64_t last_captured_ns_ = 0;
};

}  // namespace pure_uav::app
