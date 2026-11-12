#pragma once

#include <functional>
#include <memory>

#include "abstract_factory.hpp"
#include "guidance_loop.hpp"

// 每次创建检测产品时产生独立后端；不设置时使用构建所选的 YOLO NPU/CPU 后端。
struct DetectionProductConfig {
    pure_uav::detection::RuntimeConfig runtime;
    std::function<std::unique_ptr<pure_uav::detection::InferenceBackend>()> create_backend;
};

struct VioProductConfig {
    pure_uav::VioEstimator::Options options;
    pure_uav::VioPublicationPolicy policy;
    std::shared_ptr<pure_uav::VioImuSource> imu;
    std::shared_ptr<pure_uav::VioFrameSource> frames;
};

struct GuidanceProductConfig {
    pure_uav::GuidanceParams params;
    pure_uav::guidance::flight::GuidanceLoopConfig loop;
    std::shared_ptr<pure_uav::guidance::flight::GuidanceInsSource> ins;
    std::shared_ptr<pure_uav::guidance::flight::GuidanceTargetSource> target;
    std::shared_ptr<pure_uav::guidance::flight::GuidanceCommandSink> sink;
};

class YoloA733Detector final : public IDetector {
  public:
    explicit YoloA733Detector(const DetectionProductConfig& config);
    ~YoloA733Detector() override;
    void init() override;
    void stop() override;
    [[nodiscard]] bool initialized() const override;
    bool submit(const pure_uav::EncodedColorFrame& frame) override;
    void setEpoch(uint64_t session, uint64_t generation) override;
    bool detect(pure_uav::detection::TargetSnapshot& out, int64_t now_ns) override;

  private:
    pure_uav::detection::DetectionRuntime runtime_;
    bool initialized_ = false;
};

class OpenVINS final : public IVIO {
  public:
    explicit OpenVINS(VioProductConfig config);
    ~OpenVINS() override;
    void init() override;
    void stop() override;
    [[nodiscard]] bool initialized() const override;
    bool update(pure_uav::VioPoseSnapshot& out) override;
    pure_uav::VioRuntime::PublicationGuard publication() const override;

  private:
    // runtime 借用数据源；先析构 runtime，再释放数据源。
    VioProductConfig config_;
    std::unique_ptr<pure_uav::VioRuntime> runtime_;
    bool initialized_ = false;
};

class PNGuidance final : public IGuidance {
  public:
    explicit PNGuidance(GuidanceProductConfig config);
    ~PNGuidance() override;
    void init() override;
    void stop() override;
    [[nodiscard]] bool initialized() const override;
    pure_uav::guidance::flight::GuidanceLoopStatus guide(pure_uav::guidance::flight::GuidanceOutput* out = nullptr) override;

  private:
    GuidanceProductConfig config_;
    std::unique_ptr<pure_uav::guidance::flight::GuidanceLoop> loop_;
};
