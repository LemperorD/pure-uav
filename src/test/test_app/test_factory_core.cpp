#include <fcntl.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <thread>

#include "pure_uav_factory.hpp"
#include "runtime/vio_pose_publisher.hpp"
#include "startup/guidance_application.hpp"

using namespace pure_uav;
using namespace pure_uav::app;

namespace {
void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}
template <class Operation>
void mustThrow(Operation operation, const char* message) {
    try {
        operation();
    } catch (const std::logic_error&) {
        return;
    }
    throw std::runtime_error(message);
}
class EmptyImu final : public VioImuSource {
  public:
    bool pop(TimedImu& /*out*/) override {
        return false;
    }
    uint64_t session() const override {
        return 0;
    }
    bool armed() const override {
        return false;
    }
    uint64_t dropped() const override {
        return 0;
    }
};
class EmptyFrames final : public VioFrameSource {
  public:
    bool pop(VioFrame& /*out*/, int /*timeout_ms*/) override {
        return false;
    }
};
class FakeVio final : public IVIO {
  public:
    FakeVio(EmptyImu& imu, EmptyFrames& frames) : runtime_(VioEstimator::Options{}, imu, frames) {
    }
    void init() override {
        initialized_ = true;
    }
    void stop() override {
        initialized_ = false;
    }
    bool initialized() const override {
        return initialized_;
    }
    bool update(VioPoseSnapshot& out) const override {
        return runtime_.latestPose(out);
    }
    bool running() const override {
        return false;
    }
    VioRuntime::Stats stats() const override {
        return runtime_.stats();
    }
    VioRuntime::PublicationGuard publication() const override {
        return runtime_.publication();
    }

  private:
    VioRuntime runtime_;
    bool initialized_ = false;
};
}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--factory-detector-child") {
        return argc == 4 && std::string(argv[2]) == "--mission-target-fd" && fcntl(std::stoi(argv[3]), F_GETFD) >= 0 ? 0 : 3;
    }
    try {
        Communicator comm;
        EmptyImu imu;
        EmptyFrames frames;
        AppOptions options;
        options.mission = true;
        options.no_send = true;
        options.detect_bin = std::filesystem::read_symlink("/proc/self/exe").string();
        options.detect_args = {"--factory-detector-child"};
        AppStartupConfig settings;
        settings.guidance = guidance_default_params();
        settings.mission.send_enabled = false;

        std::unique_ptr<IPureUavFactory> factory = std::make_unique<PureUavFactory>(DetectionProductConfig{options}, VioProductConfig{settings.app.vio, imu, frames, settings.app.publication}, GuidanceProductConfig{options, settings, comm});
        auto vio_factory = factory->createVIOFactory();
        auto guidance_factory = factory->createGuidanceFactory();
        auto detection_factory = factory->createDetectorFactory();
        auto first_vio = vio_factory->createVIO();
        auto second_vio = vio_factory->createVIO();
        check(first_vio != second_vio && !first_vio->initialized() && !second_vio->running(), "factory started or shared a VIO runtime");
        first_vio->stop();
        first_vio->stop();
        check(!comm.isConnected(), "factory opened the flight controller");
        auto unbound_guidance = guidance_factory->createGuidance();
        mustThrow([&] { unbound_guidance->setVIO(nullptr); }, "null VIO was accepted");
        mustThrow([&] { unbound_guidance->init(); }, "guidance accepted missing VIO");
        auto unbound_detector = detection_factory->createDetector();
        mustThrow([&] { unbound_detector->init(); }, "mission detection accepted missing guidance");

        auto vio = std::make_shared<FakeVio>(imu, frames);
        auto guidance = guidance_factory->createGuidance();
        guidance->setVIO(vio);
        mustThrow([&] { guidance->init(); }, "guidance initialized before VIO");
        mustThrow([&] { guidance->guide(false); }, "guidance ran before initialization");
        vio->init();
        guidance->init();
        mustThrow([&] { guidance->setVIO(vio); }, "guidance allowed live VIO replacement");
        const int fd = guidance->targetPublisherFd();
        guidance->init();
        check(fd >= 0 && guidance->targetPublisherFd() == fd, "guidance init was not idempotent");
        const auto& status = guidance->guide(false);
        check(status.setpoints_sent == 0 && status.requests_sent == 0, "disconnected guidance sent a flight command");
        VioPosePublishState state;
        check(!publishVioPose(*vio, comm, false, 100, state), "publisher ignored stopped abstract VIO");

        auto detector = detection_factory->createDetector();
        detector->setGuidance(guidance);
        check(!detector->initialized(), "factory started detection during creation");
        factory.reset();
        vio_factory.reset();
        guidance_factory.reset();
        detection_factory.reset();
        std::weak_ptr<IGuidance> weak_guidance = guidance;
        std::weak_ptr<IVIO> weak_vio = vio;
        guidance.reset();
        vio.reset();
        check(!weak_guidance.expired() && !weak_vio.expired(), "products lost their dependencies with the factory");

        bool detection_available = true;
        try {
            detector->init();
        } catch (const std::runtime_error& error) {
            if (std::string(error.what()).find("当前构建缺少 detect_rt") == std::string::npos) {
                throw;
            }
            detection_available = false;
        }
        if (detection_available) {
            detector->init();
            mustThrow([&] { detector->setGuidance(nullptr); }, "detection allowed live target-channel replacement");
            SuperviseResult result;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
            while (!detector->detect(result) && std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            check(detector->detect(result) && result.status == SuperviseStatus::exited && result.last_exit_code == 0, "detector failed to pass the target fd or supervise its child");
        }
        detector->stop();
        detector->stop();
        check(!detector->initialized(), "detector stayed initialized after stop");
        auto retained_guidance = weak_guidance.lock();
        retained_guidance->stop();
        retained_guidance->stop();
        check(!retained_guidance->initialized() && fcntl(fd, F_GETFD) >= 0, "guidance stop closed a channel still owned by detection");
        mustThrow([&] { retained_guidance->guide(false); }, "stopped guidance resumed sending");
        retained_guidance->init();
        check(retained_guidance->initialized(), "guidance failed to restart");
        retained_guidance.reset();
        detector.reset();
        check(weak_guidance.expired() && weak_vio.expired(), "product dependencies leaked");
        std::puts("PASS: abstract factories, lifecycle, dependency ownership and detection IPC; no devices opened");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
