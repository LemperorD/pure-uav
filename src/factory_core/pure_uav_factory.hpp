#pragma once

#include "abstract_factory.hpp"

class YoloA733DetectFactory final : public IDetectFactory {
  public:
    std::shared_ptr<IDetector> createDetector() override {
        return std::make_shared<YoloA733Detector>();
    }
};

class OpenVINSFactory final : public IVIOFactory {
  public:
    std::shared_ptr<IVIO> createVIO() override {
        return std::make_shared<OpenVINS>();
    }
};

class PNGuidanceFactory final : public IGuidanceFactory {
  public:
    std::shared_ptr<IGuidance> createGuidance() override {
        return std::make_shared<PNGuidance>();
    }
};

class PureUavFactory final : public IPureUavFactory {
  public:
    std::shared_ptr<IDetectFactory> createDetectorFactory() override {
        return std::make_shared<YoloA733DetectFactory>();
    }

    std::shared_ptr<IVIOFactory> createVIOFactory() override {
        return std::make_shared<OpenVINSFactory>();
    }

    std::shared_ptr<IGuidanceFactory> createGuidanceFactory() override {
        return std::make_shared<PNGuidanceFactory>();
    }
};
