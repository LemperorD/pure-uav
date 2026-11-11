#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>

#include "pure_uav/vio_estimator.hpp"

namespace pure_uav {

struct VioFrame {
    cv::Mat gray;
    cv::Mat depth_mm;
    int64_t host_ns = 0;
    int64_t device_us = 0;
    int64_t depth_device_us = 0;
    int64_t system_us = 0;  // SDK systemTimeStampUs，供 vio_live 保持既有相机平滑时基
};

// Depth is in the native depth camera coordinates, not aligned RGB coordinates.
struct VioDepthInfo {
    double fx = 0, fy = 0, cx = 0, cy = 0;
    int width = 0, height = 0;
    double R_c2d[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double t_c2d_m[3] = {0, 0, 0};
    bool valid() const;
};

class VioFrameSource {
  public:
    virtual ~VioFrameSource() = default;
    virtual bool pop(VioFrame& out, int timeout_ms) = 0;
    virtual VioDepthInfo depthInfo() const {
        return {};
    }
    virtual void discardBefore(int64_t /*host_ns*/) {
    }
};

class VioImuSource {
  public:
    struct TimedImu {
        ImuSample imu;        // 原始飞控采样微秒；由 VioRuntime 换算为 Unix 后送入估计器
        int64_t host_ns = 0;  // steady_clock，IMU 到达主机的时刻
        uint64_t session = 0;
    };
    virtual ~VioImuSource() = default;
    virtual bool pop(TimedImu& out) = 0;
    virtual uint64_t session() const = 0;
    virtual bool armed() const = 0;
    virtual uint64_t dropped() const {
        return 0;
    }
};

struct VioPoseSnapshot {
    bool valid = false;
    VisionPose pose;  // timestamp_us 为主机 Unix 微秒
    uint64_t session = 0;
    uint64_t timebase_generation = 0;
    int64_t produced_host_ns = 0;
    int64_t sample_host_ns = 0;  // also reject old frames that finished processing recently
    int health = static_cast<int>(VioHealthState::UNKNOWN);
    int features = 0;
    long long rebuilds = 0;
    int quality = 0;
    float pos_sigma_m = 0;
    float vel_sigma_mps = 0;
};

struct VioPublicationPolicy {
    int quality_ok = 90, quality_degraded = 50;
    float pos_sigma_ok_m = 0.15f, vel_sigma_ok_mps = 0.25f;
    float pos_sigma_degraded_m = 0.5f, vel_sigma_degraded_mps = 0.8f;
};

class VioRuntime {
  public:
    VioRuntime(const VioEstimator::Options& opt, VioImuSource& imu, VioFrameSource& frames, const VioPublicationPolicy& policy = {});
    ~VioRuntime();
    VioRuntime(const VioRuntime&) = delete;
    VioRuntime& operator=(const VioRuntime&) = delete;
    bool start(std::string* error);
    void stop();
    bool latestPose(VioPoseSnapshot& out) const;
    bool publishable() const;
    bool clockValid() const;
    uint64_t timebaseGeneration() const;
    bool running() const;

    struct Stats {
        uint64_t imu_popped = 0, imu_dropped = 0, imu_out_of_order = 0;
        uint64_t frames_fed = 0, frames_dropped = 0;
        uint64_t poses = 0, blocked = 0, timebase_resets = 0;
        uint64_t depth_frames_used = 0, depth_frames_rejected = 0;
        uint64_t zupt_changes = 0;
        bool zupt_enabled = false;
        double imu_hz = 0;
        std::string last_error;
    };
    Stats stats() const;

    // Hold through the final gate and the nonblocking serial write. A reset/health
    // invalidation cannot overtake a checked snapshot before it is sent.
    class PublicationGuard {
      public:
        PublicationGuard(PublicationGuard&&) noexcept = default;
        const VioPoseSnapshot& snapshot() const;
        bool publishable() const;
        bool clockValid() const;
        uint64_t generation() const;

      private:
        friend class VioRuntime;
        explicit PublicationGuard(const VioRuntime& runtime);
        const VioRuntime& runtime_;
        std::unique_lock<std::mutex> lock_;
    };
    PublicationGuard publication() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace pure_uav
