#pragma once
// Orbbec 彩色采集：仅彩色流，统一输出 BGR CV_8UC3。
// 编码回退、时间戳与线程约束见同目录 README。

#include <cstdint>
#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <string>

namespace pure_uav::detection {

/// 相机采集配置；分辨率/帧率必须与设备支持的 profile 匹配。
struct CameraConfig {
    int width = 640;
    int height = 480;
    int fps = 60;
    bool verbose = false;  // true 时放开 OrbbecSDK 日志
};

/// 采集线程回调的一帧。
struct CameraFrame {
    cv::Mat bgr;                    // CV_8UC3
    uint64_t host_us = 0;           // 帧到达主机的 Unix µs
    uint64_t device_us = 0;         // 相机设备时间戳（仅诊断）
    int64_t host_monotonic_ns = 0;  // 共享采集路径的 steady_clock 时刻；旧采集器为 0
    uint64_t seq = 0;               // 相机帧序号，从 1 开始单调递增
};

/// Orbbec 彩色流采集器；回调在采集线程执行。
class CameraSource {
  public:
    /// 采集回调；在采集线程同步调用，不得阻塞。
    using FrameCallback = std::function<void(const CameraFrame&)>;

    CameraSource();
    ~CameraSource();

    CameraSource(const CameraSource&) = delete;
    CameraSource& operator=(const CameraSource&) = delete;

    /// 打开彩色流并启动采集线程。
    /// @param[out] error 失败原因（含可用 profile 列表）
    /// @return 成功为 true；已在运行时直接返回 true。
    static bool start(const CameraConfig& cfg, FrameCallback cb, std::string* error);

    /// 停止采集并释放相机；可重复调用。
    void stop();

    [[nodiscard]] bool running() const;

    /// 已打开的实际 profile（"WxH@FPS fmtF"）；未启动时为空。
    static [[nodiscard]] std::string activeProfile();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace pure_uav::detection
