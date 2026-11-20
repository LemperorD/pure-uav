#pragma once

#include <memory>

#include "pure_uav/vio_runtime.hpp"

namespace pure_uav::app {

struct SuperviseResult;
namespace mission {
struct RuntimeStatus;
}

// 创建不启动；init 失败抛异常，stop 可重复调用，析构自动停止。
class IProduct {
  public:
    virtual ~IProduct() = default;
    virtual void init() = 0;
    virtual void stop() = 0;
    [[nodiscard]] virtual bool initialized() const = 0;
};

class IGuidance;

class IDetector : public IProduct {
  public:
    virtual void setGuidance(std::shared_ptr<IGuidance> guidance) = 0;
    virtual bool detect(SuperviseResult& result) const = 0;
};

class IVIO : public IProduct {
  public:
    virtual bool update(VioPoseSnapshot& out) const = 0;
    virtual bool running() const = 0;
    virtual VioRuntime::Stats stats() const = 0;
    virtual VioRuntime::PublicationGuard publication() const = 0;
};

class IGuidance : public IProduct {
  public:
    virtual void setVIO(std::shared_ptr<IVIO> vio) = 0;
    virtual const mission::RuntimeStatus& guide(bool return_requested) = 0;
    virtual int targetPublisherFd() const = 0;
};

class IDetectFactory {
  public:
    virtual ~IDetectFactory() = default;
    virtual std::shared_ptr<IDetector> createDetector() = 0;
};

class IVIOFactory {
  public:
    virtual ~IVIOFactory() = default;
    virtual std::shared_ptr<IVIO> createVIO() = 0;
};

class IGuidanceFactory {
  public:
    virtual ~IGuidanceFactory() = default;
    virtual std::shared_ptr<IGuidance> createGuidance() = 0;
};

class IPureUavFactory {
  public:
    virtual ~IPureUavFactory() = default;
    virtual std::shared_ptr<IDetectFactory> createDetectorFactory() = 0;
    virtual std::shared_ptr<IVIOFactory> createVIOFactory() = 0;
    virtual std::shared_ptr<IGuidanceFactory> createGuidanceFactory() = 0;
};

}  // namespace pure_uav::app
