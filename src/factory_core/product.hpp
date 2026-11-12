#include "abstract_factory.hpp"

class YoloA733Detector final : public IDetector {
  public:
    std::shared_ptr<IDetector> createDetector() override {
        return std::make_shared<YoloA733Detector>();
    }
};

class OpenVINS final : public IVIO {
  public:
    std::shared_ptr<IVIO> createVIO() override {
        return std::make_shared<OpenVINS>();
    }
};

class PNGuidance final : public IGuidance {
  public:
    std::shared_ptr<IGuidance> createGuidance() override {
        return std::make_shared<PNGuidance>();
    }
};
