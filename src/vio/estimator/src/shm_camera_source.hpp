#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

#include "camera_shm.hpp"
#include "pure_uav/vio_runtime.hpp"

namespace pure_uav {
// camera_server V1 通道的只读 VIO 输入；相机配置由服务端管理。
class ShmCameraSource final : public VioFrameSource {
  public:
    struct Config {
        std::string channel = camera::kVioChannel;
        int timeout_ms = 5000;
        int width = 1920, height = 1080, fps = 30;  // width/height 都为 0 时读取标定
        bool use_depth = false;
        std::string config_dir;  // 非空时校验通道分辨率与标定链一致
        std::string estimator_config = "estimator_config.yaml";
    };

    ~ShmCameraSource() override;
    bool start(const Config& config, std::string* error);
    void stop();
    bool pop(VioFrame& out, int timeout_ms) override;
    VioDepthInfo depthInfo() const override;
    void discardBefore(int64_t host_ns) override;
    bool running() const;
    uint64_t dropped() const;
    int64_t lastFrameHostNs() const;
    std::string lastError() const;

  private:
    bool aliveLocked() const;
    void failLocked(const std::string& error) const;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    camera::ShmChannelReader reader_;
    std::vector<uint8_t> payload_;
    Config config_;
    VioDepthInfo depth_info_;
    mutable bool accepting_ = false;
    mutable std::string error_;
    uint64_t session_ = 0, generation_ = 0, last_seq_ = 0, dropped_ = 0;
    int64_t discard_before_ns_ = 0, last_frame_ns_ = 0;
};
}  // namespace pure_uav
