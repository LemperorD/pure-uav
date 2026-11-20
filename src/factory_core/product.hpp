#pragma once

#include <memory>

#include "abstract_factory.hpp"
#include "detection_supervisor.hpp"
#include "glue/mission_service.hpp"
#include "startup/app_startup.hpp"

namespace pure_uav::app {

struct DetectionProductConfig {
    AppOptions options;
};

struct VioProductConfig {
    VioEstimator::Options options;
    VioImuSource& imu;
    VioFrameSource& frames;
    VioPublicationPolicy publication;
};

struct GuidanceProductConfig {
    AppOptions options;
    AppStartupConfig settings;
    Communicator& comm;
};

class OpenVINS final : public IVIO {
  public:
    explicit OpenVINS(const VioProductConfig& config);
    ~OpenVINS() override;
    void init() override;
    void stop() override;
    bool initialized() const override;
    bool update(VioPoseSnapshot& out) const override;
    bool running() const override;
    VioRuntime::Stats stats() const override;
    VioRuntime::PublicationGuard publication() const override;

  private:
    VioRuntime runtime_;
    bool initialized_ = false;
};

class PNGuidance final : public IGuidance {
  public:
    explicit PNGuidance(GuidanceProductConfig config);
    void setVIO(std::shared_ptr<IVIO> vio) override;
    ~PNGuidance() override;
    void init() override;
    void stop() override;
    bool initialized() const override;
    const mission::RuntimeStatus& guide(bool return_requested) override;
    int targetPublisherFd() const override;

  private:
    GuidanceProductConfig config_;
    std::shared_ptr<IVIO> vio_;
    std::unique_ptr<MissionService> service_;
    bool initialized_ = false;
};

class YoloProcessDetector final : public IDetector {
  public:
    explicit YoloProcessDetector(DetectionProductConfig config);
    void setGuidance(std::shared_ptr<IGuidance> guidance) override;
    ~YoloProcessDetector() override;
    void init() override;
    void stop() override;
    bool initialized() const override;
    bool detect(SuperviseResult& result) const override;

  private:
    AppOptions options_;
    std::shared_ptr<IGuidance> guidance_;
    std::unique_ptr<DetectionSupervisor> supervisor_;
};

}  // namespace pure_uav::app
