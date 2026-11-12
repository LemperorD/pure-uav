#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <type_traits>

#include "pure_uav_factory.hpp"
#include "pure_uav_factory.hpp"  // 验证公开头可重复包含。

namespace {
using namespace pure_uav;
using namespace pure_uav::detection;
using namespace pure_uav::guidance::flight;

void check(bool value, const char* message) {
    if (!value) {
        throw std::runtime_error(message);
    }
}
template <class Exception, class Function>
void expectThrow(Function function) {
    try {
        function();
    } catch (const Exception&) {
        return;
    }
    throw std::runtime_error("Expected exception was not thrown");
}
int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct BackendState {
    int loads = 0;
    int closes = 0;
    bool fail = false;
};
class FakeBackend final : public InferenceBackend {
  public:
    explicit FakeBackend(std::shared_ptr<BackendState> state) : state_(std::move(state)) {
    }
    bool load(const RuntimeConfig&, std::string& error) override {
        ++state_->loads;
        error = state_->fail ? "test load failure" : "";
        return !state_->fail;
    }
    bool infer(const cv::Mat&, FrameResult& out, std::string&) override {
        out = {};
        out.present = true;
        out.count = 1;
        out.boxes[0] = {1, 1, 6, 6, 0.9f, 0};
        return true;
    }
    void close() noexcept override {
        ++state_->closes;
    }

  private:
    std::shared_ptr<BackendState> state_;
};
class EmptyImu final : public VioImuSource {
  public:
    bool pop(TimedImu&) override {
        return false;
    }
    uint64_t session() const override {
        return 1;
    }
    bool armed() const override {
        return false;
    }
};
class EmptyFrames final : public VioFrameSource {
  public:
    bool pop(VioFrame&, int) override {
        return false;
    }
};
class InsSource final : public GuidanceInsSource {
  public:
    bool poll(GuidanceInsSample& out) override {
        out = {};
        out.boot_us = 1000000;
        out.has_position = true;
        out.has_attitude = true;
        out.q[0] = 1;
        return true;
    }
    bool alive() const override {
        return true;
    }
};
class TargetSource final : public GuidanceTargetSource {
  public:
    bool poll(GuidanceTargetSample& out) override {
        out = {};
        out.boot_us = 1000000;
        return true;  // 无目标也应按原模块策略输出搜索/悬停指令。
    }
    bool alive() const override {
        return true;
    }
};
class Sink final : public GuidanceCommandSink {
  public:
    int sent = 0;
    bool accept = true;
    bool send(const GuidanceOutput&) override {
        ++sent;
        return accept;
    }
    bool alive() const override {
        return true;
    }
};

void detectionLifecycle() {
    auto state = std::make_shared<BackendState>();
    DetectionProductConfig config;
    config.runtime.max_age_ms = 2000;
    config.create_backend = [state] { return std::make_unique<FakeBackend>(state); };
    YoloA733DetectFactory factory(config);
    auto detector = factory.createDetector();
    check(state->loads == 0, "Creation must not start backend");
    TargetSnapshot snapshot;
    snapshot.valid = true;
    check(!detector->detect(snapshot, nowNs()) && !snapshot.valid, "Uninitialized detection clears snapshot");
    detector->init();
    detector->init();
    check(state->loads == 1 && detector->initialized(), "Init must be idempotent");
    detector->setEpoch(7, 3);
    EncodedColorFrame frame;
    frame.encoding = ColorEncoding::bgr;
    frame.width = frame.height = 8;
    frame.bytes = cv::Mat::zeros(1, 8 * 8 * 3, CV_8UC1);
    frame.seq = 1;
    frame.host_ns = nowNs();
    check(detector->submit(frame), "Frame submission failed");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!detector->detect(snapshot, nowNs()) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(snapshot.valid && snapshot.session == 7 && snapshot.generation == 3 && snapshot.result.count == 1, "Runtime result/epoch not forwarded");
    detector->setEpoch(8, 0);
    check(!detector->detect(snapshot, nowNs()) && !snapshot.valid, "Epoch change must invalidate result");
    detector->stop();
    detector->stop();
    check(!detector->initialized() && !detector->submit(frame), "Stopped product accepted a frame");
    detector->init();
    check(state->loads == 2, "Detection restart failed");
    std::shared_ptr<IProduct> base = detector;
    detector.reset();
    base.reset();
    check(state->closes == 2, "Destruction through base must stop backend");

    state->fail = true;
    auto failing = factory.createDetector();
    expectThrow<std::runtime_error>([&] { failing->init(); });
    check(!failing->initialized(), "Failed init must not mark initialized");
    state->fail = false;
    failing->init();
    failing->stop();
    config.create_backend = [] { return std::unique_ptr<InferenceBackend>{}; };
    expectThrow<std::invalid_argument>([&] { YoloA733Detector invalid(config); });
}

void factoriesAndPorts() {
    static_assert(std::has_virtual_destructor<IProduct>::value);
    static_assert(!std::is_abstract<YoloA733Detector>::value);
    static_assert(!std::is_abstract<OpenVINS>::value);
    static_assert(!std::is_abstract<PNGuidance>::value);
    expectThrow<std::invalid_argument>([] { OpenVINS invalid({}); });
    expectThrow<std::invalid_argument>([] { PNGuidance invalid({}); });
    VioProductConfig vio;
    vio.imu = std::make_shared<EmptyImu>();
    vio.frames = std::make_shared<EmptyFrames>();
    vio.options.config_dir = "/nonexistent/factory_core_test";
    GuidanceProductConfig guidance;
    guidance.ins = std::make_shared<InsSource>();
    guidance.target = std::make_shared<TargetSource>();
    auto sink = std::make_shared<Sink>();
    guidance.sink = sink;
    guidance.loop.cam.fx = guidance.loop.cam.fy = 100;

    PureUavFactory factory({}, vio, guidance);
    auto vio_factory = factory.createVIOFactory();
    auto product = vio_factory->createVIO();
    vio = {};
    VioPoseSnapshot pose;
    pose.valid = true;
    check(!product->update(pose) && !pose.valid, "Uninitialized VIO must clear snapshot");
    {
        auto guard = product->publication();
        check(!guard.publishable() && !guard.clockValid(), "Uninitialized VIO must block publication");
    }
    expectThrow<std::runtime_error>([&] { product->init(); });
    check(!product->initialized(), "Failed VIO init must not mark initialized");
    product->stop();
    product->stop();

    auto guidance_factory = factory.createGuidanceFactory();
    auto first = guidance_factory->createGuidance();
    auto second = guidance_factory->createGuidance();
    check(first != second, "Factories must create independent products");
    check(first->guide() == GuidanceLoopStatus::kNotInitialized, "Guide must require init");
    first->init();
    first->init();
    check(first->guide() == GuidanceLoopStatus::kOk && sink->sent == 1, "Guidance did not execute through sink");
    sink->accept = false;
    check(first->guide() == GuidanceLoopStatus::kSinkError, "Guidance sink failure was lost");
    first->stop();
    first->stop();
    check(first->guide() == GuidanceLoopStatus::kNotInitialized, "Stopped guidance executed");
    sink->accept = true;
    first->init();
    check(first->guide() == GuidanceLoopStatus::kOk, "Guidance restart retained stop flag");
    first->stop();
    guidance.loop.cam.fx = 0;
    PNGuidance invalid_camera(guidance);
    expectThrow<std::runtime_error>([&] { invalid_camera.init(); });
    check(!invalid_camera.initialized(), "Invalid camera must not initialize guidance");
}

void vioLifecycle(const char* config_dir) {
    std::shared_ptr<IVIO> product;
    std::weak_ptr<VioImuSource> imu_lifetime;
    {
        VioProductConfig config;
        config.options.config_dir = config_dir;
        config.options.verbose = false;
        config.options.quiet_runtime = true;
        config.imu = std::make_shared<EmptyImu>();
        config.frames = std::make_shared<EmptyFrames>();
        imu_lifetime = config.imu;
        OpenVINSFactory factory(config);
        product = factory.createVIO();
    }
    check(!imu_lifetime.expired(), "Product must retain sources after factory destruction");
    product->init();
    product->init();
    check(product->initialized(), "VIO init failed");
    {
        auto guard = product->publication();
        check(!guard.publishable() && !guard.clockValid(), "VIO without samples must block publication");
    }
    product->stop();
    product->stop();
    check(!product->initialized(), "VIO stop failed");
    product->init();
    check(product->initialized(), "VIO restart failed");
    product.reset();
    check(imu_lifetime.expired(), "Product destruction must release sources after worker stops");
}
}  // namespace

int main(int argc, char** argv) {
    try {
        check(argc == 2, "Expected VIO calibration directory argument");
        detectionLifecycle();
        factoriesAndPorts();
        vioLifecycle(argv[1]);
        std::cout << "factory_core: all checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
