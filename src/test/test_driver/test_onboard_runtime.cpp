// 无硬件回归：共享采集的最新帧交付、线程退出、队列界限及飞控端口换算。
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "guidance_loop.hpp"
#include "pure_uav/detection/detector.hpp"
#include "pure_uav/detection/yolo_cpu.hpp"
#include "runtime/bounded_queue.hpp"
#include "runtime/guidance_ports.hpp"
#include "startup/guidance_application.hpp"

namespace {
void check(bool ok) {
    if (!ok) throw std::runtime_error("onboard runtime regression failed");
}
std::mutex gate_mutex;
std::condition_variable gate;
bool entered = false, released = false;
}  // namespace

// 替代设备和推理驱动；编译真实 Detector 线程与后处理，不加载 SDK/ONNX Runtime。
namespace pure_uav::detection {
struct CameraSource::Impl {};
CameraSource::CameraSource() = default;
CameraSource::~CameraSource() = default;
bool CameraSource::start(const CameraConfig&, FrameCallback, std::string*) {
    throw std::runtime_error("external mode must not open a camera");
}
void CameraSource::stop() {
}
std::string CameraSource::activeProfile() const {
    return {};
}
struct YoloCpu::Impl {
    std::string info = "test";
};
YoloCpu::YoloCpu() : impl_(std::make_unique<Impl>()) {
}
YoloCpu::~YoloCpu() = default;
bool YoloCpu::load(const std::string&, int, std::string&) {
    return true;
}
bool YoloCpu::infer(const cv::Mat& image, YoloOutputView& out, std::string&) {
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        entered = true;
        gate.notify_all();
        if (!gate.wait_for(lock, std::chrono::seconds(3), [] { return released; })) return false;
    }
    check(!image.empty() && image.at<cv::Vec3b>(0, 0)[0] == 7);
    static const float output[] = {320, 240, 40, 20, 0.9f, 0.1f};
    out.data = output;
    out.channels = 6;
    out.anchors = 1;
    return true;
}
const std::string& YoloCpu::info() const {
    return impl_->info;
}
void YoloCpu::release() {
}
}  // namespace pure_uav::detection

namespace {
void testQueue() {
    pure_uav::app::BoundedQueue<int> queue(2);
    queue.push(1);
    queue.push(2);
    queue.push(3);
    int value = 0;
    check(queue.dropped() == 1 && queue.pop(value) && value == 2);
    check(queue.pop(value) && value == 3 && !queue.pop(value));
    std::atomic<bool> done{false};
    std::thread producer([&] {
        for (int i = 1; i <= 10000; ++i) queue.push(i);
        done = true;
    });
    int previous = 0, consumed = 0;
    while (!done.load()) {
        if (queue.pop(value)) {
            check(value > previous);
            previous = value;
            ++consumed;
        }
    }
    producer.join();
    while (queue.pop(value)) {
        check(value > previous);
        previous = value;
        ++consumed;
    }
    check(previous == 10000 && consumed + queue.dropped() - 1 == 10000);
    queue.push(9);
    queue.clear();
    check(!queue.pop(value));
}

void testExternalDetector() {
    using namespace pure_uav::detection;
    Detector detector;
    DetectorConfig config;
    config.external_camera = true;
    std::string error;
    check(detector.start(config, error));
    cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(7, 7, 7));
    detector.submitFrame(frame, 100, 200, 1, 300);
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        check(gate.wait_for(lock, std::chrono::seconds(3), [] { return entered; }));
    }
    detector.submitFrame(frame, 101, 201, 2, 301);
    detector.submitFrame(frame, 102, 202, 3, 302);
    frame.release();
    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        released = true;
    }
    gate.notify_all();
    FrameResult result{};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
        detector.poll(result);
        if (result.seq == 3) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(result.seq == 3 && result.host_us == 102 && result.device_us == 202 && result.host_monotonic_ns == 302);
    check(result.present && result.count == 1 && result.u_px == 320 && result.v_px == 240);
    check(detector.stats().capture_frames == 3 && detector.stats().process_frames == 2);
    detector.stop();
    detector.stop();
    check(!detector.running() && !detector.alive());
    check(detector.start(config, error));
    detector.stop();
}

void testGuidancePorts() {
    using namespace pure_uav;
    Attitude attitude;
    LocalPosition position;
    guidance::flight::GuidanceInsSample sample;
    check(!app::makeGuidanceIns(attitude, position, 0.1, sample));
    attitude.time_boot_ms = 1000;
    position.time_boot_ms = 1100;
    position.vx = 3;
    position.y = 4;
    check(app::makeGuidanceIns(attitude, position, 0.1, sample));
    check(sample.boot_us == 1000000 && sample.v.x == 3 && sample.p.y == 4 && sample.q[0] == 1);
    position.time_boot_ms = 1101;
    check(!app::makeGuidanceIns(attitude, position, 0.1, sample));
    guidance::flight::GuidanceOutput output;
    output.a_cmd_ned = Vec3{1, 2, 3};
    auto cmd = app::makeGuidanceCmd(output);
    check(cmd.mode == GuidanceMode::ACCELERATION && cmd.afx == 1 && cmd.afy == 2 && cmd.afz == 3);
    output.ctrl = guidance::flight::GuidanceControlMode::kVelocity;
    output.v_cmd_ned = Vec3{-1, -2, -3};
    cmd = app::makeGuidanceCmd(output);
    check(cmd.mode == GuidanceMode::VELOCITY && cmd.vx == -1 && cmd.vy == -2 && cmd.vz == -3);
}

void testPixelOnlyGuidance() {
    using namespace pure_uav;
    namespace flight = guidance::flight;
    struct Ins : flight::GuidanceInsSource {
        uint64_t tick = 1000000;
        bool poll(flight::GuidanceInsSample& sample) override {
            sample.boot_us = tick;
            tick += 20000;
            sample.has_attitude = sample.has_position = true;
            sample.v = Vec3{5, 0, 0};
            return true;
        }
        bool alive() const override {
            return true;
        }
    } ins;
    struct Target : flight::GuidanceTargetSource {
        uint64_t tick = 1000000;
        double u = 320;
        bool poll(flight::GuidanceTargetSample& sample) override {
            sample.boot_us = tick;
            tick += 20000;
            sample.present = true;
            sample.u_px = u;
            u += 10;
            sample.v_px = 240;
            return true;
        }
        bool alive() const override {
            return true;
        }
    } target;
    auto params = app::guidance_default_params();
    const auto loaded = app::guidance_load_yaml_file("config/onboard-guidance.yaml", &params);
    check(loaded.ok && app::guidance_validate_params(params).ok);
    params.loop.cam.fx = params.loop.cam.fy = 500;
    params.loop.cam.cx = 320;
    params.loop.cam.cy = 240;
    // 相机 Z 向前，机体 X 向前。
    const double rotation[] = {0, 0, 1, 1, 0, 0, 0, 1, 0};
    for (int i = 0; i < 9; ++i) params.loop.cam.R_body_cam[i] = rotation[i];
    flight::GuidanceNullSink sink;
    flight::GuidanceLoop loop(params.guidance, params.loop, ins, target, sink);
    check(loop.init());
    flight::GuidanceOutput out;
    check(loop.run_once(&out) == flight::GuidanceLoopStatus::kOk);
    check(loop.run_once(&out) == flight::GuidanceLoopStatus::kOk);
    check(out.valid && out.mode == PureGuidanceMode::PN && out.a_cmd_norm > 0);
}
}  // namespace

int main() {
    try {
        testQueue();
        testExternalDetector();
        testGuidancePorts();
        testPixelOnlyGuidance();
        std::puts("onboard runtime: PASS");
        return 0;
    } catch (const std::exception& error) {
        {
            std::lock_guard<std::mutex> lock(gate_mutex);
            released = true;
        }
        gate.notify_all();
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
