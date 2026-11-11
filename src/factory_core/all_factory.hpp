#pragma once

#include <memory>

class IProduct {
  public:
    virtual void init() = 0;
};

class IDetector : public IProduct {
  public:
    virtual ~IDetector() = default;
    virtual void detect() = 0;
};

class IVIO : public IProduct {
  public:
    virtual ~IVIO() = default;
    virtual void update() = 0;
};

class IGuidance : public IProduct {
  public:
    virtual ~IGuidance() = default;
    virtual void guide() = 0;
};

class IDetectFactory {
  public:
    virtual ~IDetectFactory() = default;
    virtual IDetector* createDetector() = 0;
};

class IVIOFactory {
  public:
    virtual ~IVIOFactory() = default;
    virtual IVIO* createVIO() = 0;
};

class IGuidanceFactory {
  public:
    virtual ~IGuidanceFactory() = default;
    virtual IGuidance* createGuidance() = 0;
};

class IPureUavFactory {
  public:
    virtual ~IPureUavFactory() = default;
    virtual std::shared_ptr<IDetectFactory> createDetectorFactory() = 0;
    virtual std::shared_ptr<IVIOFactory> createVIOFactory() = 0;
    virtual std::shared_ptr<IGuidanceFactory> createGuidanceFactory() = 0;
};
