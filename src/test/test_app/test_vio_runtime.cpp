#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "pure_uav/vio_runtime.hpp"

using namespace pure_uav;
namespace {
int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void expect(bool value, const char* reason) {
    if (!value) {
        throw std::runtime_error(reason);
    }
}
template <class Predicate>
void await(Predicate predicate, const char* reason) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    expect(predicate(), reason);
}
class Imus final : public VioImuSource {
  public:
    std::atomic<uint64_t> current_session{0}, drop_count{0};
    std::atomic<bool> is_armed{false}, fail{false};
    bool pop(TimedImu& out) override {
        if (fail.load()) {
            throw std::runtime_error("injected IMU source failure");
        }
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.empty()) {
            return false;
        }
        out = queue.front();
        queue.pop_front();
        return true;
    }
    void push(uint64_t timestamp, uint64_t s) {
        TimedImu item;
        item.imu.timestamp_us = timestamp;
        item.imu.az = -9.81;
        item.host_ns = nowNs();
        item.session = s;
        std::lock_guard<std::mutex> lock(mutex);
        queue.push_back(item);
    }
    uint64_t session() const override {
        return current_session.load();
    }
    bool armed() const override {
        return is_armed.load();
    }
    uint64_t dropped() const override {
        return drop_count.load();
    }

  private:
    std::mutex mutex;
    std::deque<TimedImu> queue;
};
class Frames final : public VioFrameSource {
  public:
    std::atomic<unsigned> discards{0};
    bool pop(VioFrame& out, int timeout_ms) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.empty()) {
            return false;
        }
        out = std::move(queue.front());
        queue.pop_front();
        return true;
    }
    void push(int64_t stamp) {
        VioFrame frame;
        frame.host_ns = stamp;
        frame.gray = cv::Mat(1080, 1920, CV_8UC1, cv::Scalar(0));
        std::lock_guard<std::mutex> lock(mutex);
        queue.push_back(std::move(frame));
    }
    void discardBefore(int64_t /*unused*/) override {
        ++discards;
    }

  private:
    std::mutex mutex;
    std::deque<VioFrame> queue;
};
}  // namespace
int main(int argc, char** argv) {
    try {
        expect(argc == 2, "need test configuration path");
        VioEstimator::Options options;
        options.config_dir = argv[1];
        options.verbose = false;
        options.quiet_runtime = true;
        Imus imu;
        Frames frames;
        {
            auto bad = options;
            bad.config_dir += "/missing";
            VioRuntime runtime(bad, imu, frames);
            std::string error;
            expect(!runtime.start(&error) && !error.empty(), "missing configuration must fail cleanly");
        }
        VioRuntime runtime(options, imu, frames);
        std::string error;
        expect(runtime.start(&error), error.c_str());
        expect(runtime.running() && !runtime.publishable() && !runtime.clockValid(), "startup gate open");
        uint64_t ts = 1000000;
        for (int i = 0; i < 180; ++i, ts += 5000) {
            imu.push(ts, 0);
        }
        await([&] { return runtime.stats().imu_popped == 180; }, "IMU batch not consumed");
        await([&] { return runtime.clockValid(); }, "clock did not recover after warmup");
        {
            auto guard = runtime.publication();
            uint64_t mapped_us = 0;
            expect(guard.hostToFcuUs(nowNs(), mapped_us) && mapped_us > 0, "mission cannot map a target with the active VIO clock");
        }
        auto st = runtime.stats();
        expect(st.imu_dropped >= 150 && st.imu_hz > 199 && st.imu_hz < 201, "warmup or measured timestamp-based rate is wrong");
        expect(!runtime.publishable(), "no images yet, publication must remain disabled");
        frames.push(nowNs());
        await([&] { return runtime.stats().frames_fed >= 1; }, "real estimator did not consume bracketed test image");
        expect(!runtime.publishable(), "blank test image fabricated a valid pose");
        const auto generation = runtime.timebaseGeneration();
        imu.is_armed.store(true);
        await([&] { return !runtime.stats().zupt_enabled && runtime.stats().zupt_changes >= 2; }, "armed ZUPT not disabled");
        imu.push(ts - 5000, 0);
        await([&] { return runtime.stats().imu_out_of_order > 0; }, "duplicate IMU was not rejected");
        expect(runtime.timebaseGeneration() == generation, "duplicate caused unnecessary reset");
        imu.push(ts + 200000, 0);
        await([&] { return runtime.timebaseGeneration() > generation; }, "IMU gap not reset");
        expect(!runtime.clockValid() && !runtime.publishable(), "gap left publication enabled");
        const auto after_gap = runtime.timebaseGeneration();
        {
            auto guard = runtime.publication();
            imu.current_session.store(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            expect(runtime.timebaseGeneration() == after_gap, "reset overtook a held publication guard");
        }
        await([&] { return runtime.timebaseGeneration() > after_gap; }, "session change without IMU not detected");
        const auto after_session = runtime.timebaseGeneration();
        imu.push(ts + 205000, 0);
        await([&] { return runtime.stats().imu_popped == 183; }, "old session not consumed");
        expect(!runtime.publishable(), "old session reopened publication");
        imu.drop_count.store(1);
        await([&] { return runtime.timebaseGeneration() > after_session; }, "source queue overflow not reset");
        {
            auto guard = runtime.publication();
            expect(!guard.snapshot().valid && !guard.publishable(), "guard exposed invalid pose");
            uint64_t mapped_us = 0;
            expect(!guard.hostToFcuUs(nowNs(), mapped_us), "target mapping retained an invalidated VIO clock");
        }
        imu.fail.store(true);
        await([&] { return !runtime.running(); }, "source exception did not stop worker");
        expect(runtime.stats().last_error == "injected IMU source failure", "source error was swallowed");
        runtime.stop();
        runtime.stop();
        expect(frames.discards.load() >= 4, "reset did not invalidate queued frames");
        imu.fail.store(false);
        expect(runtime.start(&error), "runtime could not restart after stop");
        expect(!runtime.publishable(), "restart exposed previous pose");
        const auto before = std::chrono::steady_clock::now();
        runtime.stop();
        expect(std::chrono::steady_clock::now() - before < std::chrono::milliseconds(500), "stop is not bounded while idle");
        std::puts("PASS: real runtime + fake sources: warmup, measured rate, ZUPT, duplicate, gap, session, overflow, failure, restart");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
