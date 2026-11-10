#pragma once

#include <memory>
#include <string>

namespace pure_uav {
class Communicator;
}

namespace pure_uav::app {

enum class CameraLoadResult {
    kOk,
    kNotCalibrated,
    kError,
};

/// guidance 的 app 层胶水：参数/相机标定加载、CommInsSource、GuidanceLoop 初始化。
/// 只做 init，不启动 guidance 线程；guidance 核本身不依赖 driver/YAML。
class GuidanceSession {
  public:
    GuidanceSession();
    ~GuidanceSession();

    GuidanceSession(const GuidanceSession&) = delete;
    GuidanceSession& operator=(const GuidanceSession&) = delete;

    bool initParams(const std::string& path, std::string& error);
    CameraLoadResult initCamera(const std::string& path, std::string& error);
    bool attach(Communicator& comm, std::string& error);

    [[nodiscard]] bool paramsOk() const;
    [[nodiscard]] bool cameraOk() const;
    [[nodiscard]] bool cameraFatal() const;
    [[nodiscard]] bool readyForGuidance() const;
    [[nodiscard]] const std::string& cameraReason() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace pure_uav::app
