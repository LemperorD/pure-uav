#pragma once

#include <memory>
#include <string>

namespace pure_uav::app {

struct GuidanceAppParams;

enum class CameraLoadResult {
    kOk,
    kNotCalibrated,
    kError,
};

/// guidance 的 app 层胶水：只负责参数/相机标定加载与校验。
/// 不构造 GuidanceLoop、不启动线程；guidance 核本身不依赖 driver/YAML。
class GuidanceSession {
  public:
    GuidanceSession();
    ~GuidanceSession();

    GuidanceSession(const GuidanceSession&) = delete;
    GuidanceSession& operator=(const GuidanceSession&) = delete;

    /// 一次加载制导律/loop 参数和彩色相机标定。
    /// calibrated=false 不是致命错误，返回 true 但 cameraOk()==false。
    bool init(const std::string& guidance_path, const std::string& camera_path, std::string& error);
    bool initParams(const std::string& path, std::string& error);
    CameraLoadResult initCamera(const std::string& path, std::string& error);

    [[nodiscard]] bool paramsOk() const;
    [[nodiscard]] bool cameraOk() const;
    [[nodiscard]] bool cameraFatal() const;
    [[nodiscard]] bool readyForGuidance() const;
    [[nodiscard]] const std::string& cameraReason() const;
    /// 已加载的制导参数；未成功 initParams 时为默认值。
    [[nodiscard]] const GuidanceAppParams& params() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace pure_uav::app
