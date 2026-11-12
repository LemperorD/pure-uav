#include <thread>

#include "core/all_factory.hpp"

int main() {
    std::shared_ptr<IPureUavFactory> factory = std::make_shared<IPureUavFactory>();
    std::shared_ptr<IDetectFactory> detectFactory = factory->createDetectorFactory();
    std::shared_ptr<IVIOFactory> vioFactory = factory->createVIOFactory();
    std::shared_ptr<IGuidanceFactory> guidanceFactory = factory->createGuidanceFactory();

    IDetector* detector = detectFactory->createDetector();
    IVIO* vio = vioFactory->createVIO();
    IGuidance* guidance = guidanceFactory->createGuidance();

    detector->init();
    vio->init();
    guidance->init();

    std::thread detectThread([&]() { detector->detect(); });

    std::thread vioThread([&]() { vio->update(); });

    std::thread guidanceThread([&]() { guidance->guide(); });

    return 0;
}
