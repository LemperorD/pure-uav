#pragma once
// =============================================================================
//  Orbbec 彩色相机采集（仅彩色流，统一输出 BGR CV_8UC3）
//
//  · 编码优先级：OB_FORMAT_MJPG → OB_FORMAT_RGB → OB_FORMAT_YUYV；
//    同分辨率/帧率都不支持时 start() 返回 false，错误信息带可用 profile 列表。
//  · host_us 为帧到达主机时的 Unix 微秒（system_clock）；device_us 为
//    SDK timeStampUs()，仅作诊断。host_us 不是曝光中点、不是飞控 boot 时钟。
//  · 不打开深度流，不锁定曝光/增益（检测依赖相机默认自动曝光）。
//  · start()/stop() 可重复调用，stop() 幂等。
// =============================================================================

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace pure_uav {
namespace detection {

struct CameraConfig {
    int width = 640;
    int height = 480;
    int fps = 60;
    bool verbose = false;   // true 时放开 SDK 日志（默认压到 ERROR）
};

struct CameraFrame {
    cv::Mat bgr;            // CV_8UC3
    uint64_t host_us = 0;   // 到达主机的 Unix 微秒
    uint64_t device_us = 0; // 相机设备时间戳（诊断）
    uint64_t seq = 0;       // 相机帧序号（从 1 开始单调递增）
};

class CameraSource {
public:
    // 在采集线程中同步调用，回调内不要阻塞。
    using FrameCallback = std::function<void(const CameraFrame&)>;

    CameraSource();
    ~CameraSource();

    CameraSource(const CameraSource&) = delete;
    CameraSource& operator=(const CameraSource&) = delete;

    bool start(const CameraConfig& cfg, FrameCallback cb, std::string* error);
    void stop();
    bool running() const;

    // 已打开的实际 profile（"WxH@FPS fmtF"）；未启动时为空。
    std::string activeProfile() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace detection
}  // namespace pure_uav
