#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <thread>
#include <vector>

#include "detection_runtime.hpp"

using namespace pure_uav;
using namespace pure_uav::detection;
namespace {
int checks = 0;
int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void check(bool condition, const char* reason) {
    ++checks;
    if (!condition) {
        throw std::runtime_error(reason);
    }
}
template <class Predicate>
void waitFor(Predicate predicate, const char* reason) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(predicate(), reason);
}
EncodedColorFrame makeFrame(uint64_t seq, unsigned char value = 10) {
    EncodedColorFrame frame;
    frame.width = 16;
    frame.height = 16;
    frame.encoding = ColorEncoding::bgr;
    frame.bytes = cv::Mat(1, 16 * 16 * 3, CV_8UC1, cv::Scalar(value));
    frame.host_ns = nowNs();
    frame.seq = seq;
    frame.device_us = 7000;
    frame.depth_device_us = 7000;
    frame.depth_mm = cv::Mat(16, 16, CV_16UC1, cv::Scalar(2000));
    return frame;
}
DepthRangeCalibration rangeCalibration() {
    DepthRangeCalibration c;
    c.color.width = c.depth.width = c.color.height = c.depth.height = 16;
    c.color.fx = c.depth.fx = c.color.fy = c.depth.fy = 10;
    c.color.cx = c.depth.cx = 5;
    c.color.cy = c.depth.cy = 6;
    return c;
}
struct EngineState {
    std::mutex mutex;
    std::condition_variable cv;
    bool hold = false, entered = false, released = false;
    std::atomic<bool> fail{false}, empty{false};
    std::atomic<int> pixel{0}, closed{0};
};
class TestBackend final : public InferenceBackend {
  public:
    explicit TestBackend(std::shared_ptr<EngineState> state) : state_(std::move(state)) {
    }
    bool load(const RuntimeConfig& /*config*/, std::string& /*error*/) override {
        return true;
    }
    bool infer(const cv::Mat& bgr, FrameResult& result, std::string& error) override {
        {
            std::unique_lock<std::mutex> lock(state_->mutex);
            state_->entered = true;
            state_->cv.notify_all();
            state_->cv.wait_for(lock, std::chrono::seconds(1), [&] { return !state_->hold || state_->released; });
        }
        state_->pixel.store(bgr.at<cv::Vec3b>(0, 0)[0]);
        if (state_->fail.load()) {
            error = "injected inference failure";
            return false;
        }
        result = {};
        result.infer_ms = 1;
        result.present = !state_->empty.load();
        result.count = result.present ? 1 : 0;
        result.boxes[0] = {2, 3, 8, 9, 0.9f, 0};
        return true;
    }
    void close() noexcept override {
        ++state_->closed;
    }

  private:
    std::shared_ptr<EngineState> state_;
};
void hold(const std::shared_ptr<EngineState>& state) {
    std::scoped_lock lock(state->mutex);
    state->hold = true;
    state->entered = false;
    state->released = false;
}
bool entered(const std::shared_ptr<EngineState>& state) {
    std::scoped_lock lock(state->mutex);
    return state->entered;
}
void release(const std::shared_ptr<EngineState>& state) {
    std::scoped_lock lock(state->mutex);
    state->released = true;
    state->hold = false;
    state->cv.notify_all();
}
void pause() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}
}  // namespace
int main() {
    try {
        auto frame = makeFrame(1, 20);
        check(decodeColorFrame(frame).at<cv::Vec3b>(0, 0)[0] == 20, "BGR decoder");
        frame.encoding = ColorEncoding::rgb;
        frame.bytes.at<unsigned char>(0, 0) = 33;
        frame.bytes.at<unsigned char>(0, 2) = 77;
        check(decodeColorFrame(frame).at<cv::Vec3b>(0, 0)[0] == 77, "RGB channel order");
        const cv::Mat image(16, 16, CV_8UC3, cv::Scalar(80, 80, 80));
        std::vector<unsigned char> jpeg;
        cv::imencode(".jpg", image, jpeg);
        frame.bytes = cv::Mat(1, static_cast<int>(jpeg.size()), CV_8UC1, jpeg.data()).clone();
        frame.encoding = ColorEncoding::jpeg;
        check(decodeColorFrame(frame).size() == image.size(), "JPEG decoder");
        frame.encoding = ColorEncoding::yuyv;
        frame.bytes = cv::Mat(1, 16 * 16 * 2, CV_8UC1, cv::Scalar(128));
        check(decodeColorFrame(frame).type() == CV_8UC3, "YUYV decoder");
        frame.bytes = cv::Mat::zeros(1, 3, CV_8UC1);
        bool rejected = false;
        try {
            decodeColorFrame(frame);
        } catch (const std::exception&) {
            rejected = true;
        }
        check(rejected, "truncated raw color input must be rejected");

        RuntimeConfig config;
        config.max_hz = 60;
        config.max_age_ms = 500;
        std::string error;
        auto invalid = config;
        invalid.target_class = 2;
        check(!validateRuntimeConfig(invalid, error), "invalid target class accepted");
        DetectionRuntime unavailable(config, {});
        check(!unavailable.start(&error) && !error.empty(), "missing backend must not fabricate a detector");
        auto state = std::make_shared<EngineState>();
        DetectionRuntime runtime(config, std::make_unique<TestBackend>(state), rangeCalibration());
        check(runtime.start(&error), "worker start");
        check(!runtime.start(&error), "duplicate start");
        runtime.setEpoch(3, 4);
        check(runtime.submit(makeFrame(1)), "first submitted frame");
        waitFor([&] { return runtime.stats().processed == 1; }, "first result");
        TargetSnapshot target;
        check(runtime.latest(target, nowNs()) && target.valid && target.result.present, "target snapshot");
        check(target.result.u_px == 5 && target.result.v_px == 6 && target.result.w_px == 6, "primary target not recomputed from validated boxes");
        check(target.session == 3 && target.generation == 4 && target.result.device_us == 7000, "frame identity or epoch lost");
        check(target.result.has_range && std::abs(target.result.range_m - 2.0) < 1e-6, "worker did not use matching depth");
        check(target.result.host_us == 0, "monotonic timestamp must not masquerade as Unix time");
        check(runtime.latest(target, target.captured_host_ns + 500000000LL), "inclusive age boundary");
        const auto captured = target.captured_host_ns;
        check(!runtime.latest(target, captured + 500000001LL) && !target.result.present, "expired target must be cleared");
        const auto duplicate = makeFrame(1);
        check(!runtime.submit(duplicate), "duplicate frame accepted");
        frame = makeFrame(2);
        frame.host_ns = nowNs() + 1000000000LL;
        check(!runtime.submit(frame), "future capture accepted");
        frame = makeFrame(2);
        frame.host_ns = nowNs() - 600000000LL;
        check(!runtime.submit(frame), "stale capture accepted");

        pause();
        state->empty.store(true);
        check(runtime.submit(makeFrame(2)), "empty result frame submitted");
        waitFor([&] { return runtime.stats().processed == 2; }, "empty result processed");
        check(runtime.latest(target, nowNs()) && !target.result.present && target.result.count == 0 && target.result.class_id == -1, "no target did not replace previous detection");

        state->empty.store(false);
        hold(state);
        pause();
        check(runtime.submit(makeFrame(3)), "blocked inference submission");
        waitFor([&] { return entered(state); }, "backend did not enter");
        pause();
        check(runtime.submit(makeFrame(4, 40)), "queued frame submission");
        pause();
        frame = makeFrame(5, 50);
        frame.depth_mm.setTo(3000);
        check(runtime.submit(frame), "newest queued frame submission");
        frame.depth_mm.setTo(9000);
        frame.bytes.setTo(cv::Scalar(99));
        check(!runtime.submit(makeFrame(6)), "rate limiter failed");
        release(state);
        waitFor([&] { return runtime.stats().processed >= 4; }, "latest replacement not processed");
        check(runtime.latest(target, nowNs()) && target.result.seq == 5 && state->pixel.load() == 50, "pending buffer was not owned, or old frames were not replaced");
        check(target.result.has_range && std::abs(target.result.range_m - 3.0) < 1e-6, "queued depth was not owned or belonged to a different frame");
        check(runtime.stats().replaced >= 1 && runtime.stats().rate_limited >= 1, "drop counters");

        hold(state);
        pause();
        check(runtime.submit(makeFrame(7)), "epoch inference submission");
        waitFor([&] { return entered(state); }, "epoch backend did not enter");
        runtime.setEpoch(4, 9);
        check(!runtime.latest(target, nowNs()), "epoch did not invalidate old target");
        release(state);
        waitFor([&] { return runtime.stats().invalidated >= 1; }, "in-flight old epoch was published");
        pause();
        check(runtime.submit(makeFrame(1)), "new epoch reset sequence");
        waitFor([&] { return runtime.latest(target, nowNs()) && target.result.seq == 1; }, "new epoch result");
        check(target.session == 4 && target.generation == 9, "new epoch metadata");

        pause();
        state->fail.store(true);
        check(runtime.submit(makeFrame(2)), "failure test frame");
        waitFor([&] { return !runtime.running(); }, "inference failure was swallowed");
        check(!runtime.latest(target, nowNs()) && runtime.stats().last_error.find("injected") != std::string::npos, "failed backend left valid targets");
        runtime.stop();
        runtime.stop();
        check(state->closed.load() == 1, "backend not closed exactly once");
        state->fail.store(false);
        check(runtime.start(&error), "restart after failure");
        check(!runtime.latest(target, nowNs()), "restart reused stale detection");
        runtime.stop();
        // 每种缺失/无效深度都必须清掉上一帧距离，保留正常检测框。
        check(runtime.start(&error), "range failure worker start");
        for (int scenario = 0; scenario < 8; ++scenario) {
            pause();
            auto rgbd = makeFrame(static_cast<uint64_t>(scenario + 1));
            if (scenario == 0) rgbd.depth_mm.release();
            if (scenario == 1) rgbd.depth_device_us = 0;
            if (scenario == 2) rgbd.depth_device_us = rgbd.device_us + 60000;
            if (scenario == 3) rgbd.depth_device_us = -1;
            if (scenario == 4) rgbd.depth_mm = cv::Mat::zeros(16, 16, CV_16UC1);
            if (scenario == 5) rgbd.depth_mm = cv::Mat(16, 16, CV_16UC1, cv::Scalar(65535));
            if (scenario == 6) rgbd.depth_mm = cv::Mat(15, 16, CV_16UC1, cv::Scalar(2000));
            if (scenario == 7) rgbd.depth_mm = cv::Mat(16, 16, CV_32FC1, cv::Scalar(2));
            check(runtime.submit(rgbd), "invalid depth prevented color submission");
            waitFor([&] { return runtime.latest(target, nowNs()) && target.result.seq == rgbd.seq; }, "invalid depth result missing");
            check(target.result.present && !target.result.has_range && target.result.range_m == 0, "bad depth retained range or removed detection");
        }
        runtime.stop();
        std::printf("PASS: %d detection checks (synthetic backend, real worker/decoding; no NPU or camera)\n", checks);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
