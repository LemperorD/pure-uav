#pragma once

#include <utility>

#include "product.hpp"

namespace pure_uav::app {

class YoloProcessDetectFactory final : public IDetectFactory {
  public:
    explicit YoloProcessDetectFactory(DetectionProductConfig config = {}) : config_(std::move(config)) {
    }
    std::shared_ptr<IDetector> createDetector() override {
        return std::make_shared<YoloProcessDetector>(config_);
    }

  private:
    DetectionProductConfig config_;
};

class OpenVINSFactory final : public IVIOFactory {
  public:
    explicit OpenVINSFactory(VioProductConfig config) : config_(std::move(config)) {
    }
    std::shared_ptr<IVIO> createVIO() override {
        return std::make_shared<OpenVINS>(config_);
    }

  private:
    VioProductConfig config_;
};

class PNGuidanceFactory final : public IGuidanceFactory {
  public:
    explicit PNGuidanceFactory(GuidanceProductConfig config) : config_(std::move(config)) {
    }
    std::shared_ptr<IGuidance> createGuidance() override {
        return std::make_shared<PNGuidance>(config_);
    }

  private:
    GuidanceProductConfig config_;
};

class PureUavFactory final : public IPureUavFactory {
  public:
    PureUavFactory(DetectionProductConfig detection, VioProductConfig vio, GuidanceProductConfig guidance) : detection_(std::move(detection)), vio_(std::move(vio)), guidance_(std::move(guidance)) {
    }
    std::shared_ptr<IDetectFactory> createDetectorFactory() override {
        return std::make_shared<YoloProcessDetectFactory>(detection_);
    }
    std::shared_ptr<IVIOFactory> createVIOFactory() override {
        return std::make_shared<OpenVINSFactory>(vio_);
    }
    std::shared_ptr<IGuidanceFactory> createGuidanceFactory() override {
        return std::make_shared<PNGuidanceFactory>(guidance_);
    }

  private:
    DetectionProductConfig detection_;
    VioProductConfig vio_;
    GuidanceProductConfig guidance_;
};

}  // namespace pure_uav::app
