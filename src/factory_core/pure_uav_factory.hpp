#pragma once

#include <utility>

#include "product.hpp"

class YoloA733DetectFactory final : public IDetectFactory {
  public:
    explicit YoloA733DetectFactory(DetectionProductConfig config = {}) : config_(std::move(config)) {
    }
    std::shared_ptr<IDetector> createDetector() override {
        return std::make_shared<YoloA733Detector>(config_);
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

// 只创建对象，不打开相机、串口，也不启动线程；宿主显式调用产品 init。
// 每个产品拥有独立运行时。配置中的共享数据源应由宿主分配消费权限，
// 不应让多个运行中的产品同时消费同一个破坏性读取队列或发送到同一出口。
class PureUavFactory final : public IPureUavFactory {
  public:
    PureUavFactory(DetectionProductConfig detection, VioProductConfig vio, GuidanceProductConfig guidance) : detection_(std::move(detection)), vio_(std::move(vio)), guidance_(std::move(guidance)) {
    }
    std::shared_ptr<IDetectFactory> createDetectorFactory() override {
        return std::make_shared<YoloA733DetectFactory>(detection_);
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
