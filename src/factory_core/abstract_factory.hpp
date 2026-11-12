#pragma once

#include <memory>

#include "detection_runtime.hpp"
#include "guidance_flight_types.hpp"
#include "pure_uav/vio_runtime.hpp"

// init/stop 与单步调用由宿主串行调度；内部工作线程由原模块管理。
// init 失败抛出异常；stop 可重复调用，析构自动停止。
class IProduct {
  public:
    virtual ~IProduct() = default;
    virtual void init() = 0;
    virtual void stop() = 0;
    [[nodiscard]] virtual bool initialized() const = 0;
};

class IDetector : public IProduct {
  public:
    virtual bool submit(const pure_uav::EncodedColorFrame& frame) = 0;
    virtual void setEpoch(uint64_t session, uint64_t generation) = 0;
    // 读取仍在有效期内的最新快照；同一快照可重复读取。
    virtual bool detect(pure_uav::detection::TargetSnapshot& out, int64_t now_ns) = 0;
};

class IVIO : public IProduct {
  public:
    // 读取最新定位快照，不直接向飞控发布；发布仍需宿主的时间基/健康门控。
    virtual bool update(pure_uav::VioPoseSnapshot& out) = 0;
    virtual pure_uav::VioRuntime::PublicationGuard publication() const = 0;
};

class IGuidance : public IProduct {
  public:
    // 执行一拍；宿主负责按配置频率调度，并处理原模块的状态码。
    virtual pure_uav::guidance::flight::GuidanceLoopStatus guide(pure_uav::guidance::flight::GuidanceOutput* out = nullptr) = 0;
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
