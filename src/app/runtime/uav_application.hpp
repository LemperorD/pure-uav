#pragma once

#include <memory>

#include "startup/app_options.hpp"

namespace pure_uav::app {

class IPureUavFactory;
class IVIO;
class IGuidance;
class IDetector;

// 宿主拥有设备与产品，负责接线、节拍监督和有序退出。
class UavApplication {
  public:
    explicit UavApplication(AppOptions options);
    ~UavApplication();
    UavApplication(const UavApplication&) = delete;
    UavApplication& operator=(const UavApplication&) = delete;

    std::unique_ptr<IPureUavFactory> prepare();
    void init(std::shared_ptr<IVIO> vio, std::shared_ptr<IGuidance> guidance, std::shared_ptr<IDetector> detector);
    int run();
    int result() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace pure_uav::app
