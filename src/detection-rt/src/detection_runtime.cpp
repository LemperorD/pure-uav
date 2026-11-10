#include "detection_runtime.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

namespace pure_uav::detection {
namespace {
int64_t steadyNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool fresh(int64_t stamp, int64_t now, int max_age_ms) {
    return stamp > 0 && now >= stamp && now - stamp <= static_cast<int64_t>(max_age_ms) * 1000000;
}
bool validResult(const FrameResult& result, const RuntimeConfig& config, int width, int height) {
    if (result.count < 0 || result.count > config.max_det || result.present != (result.count > 0)) {
        return false;
    }
    for (int index = 0; index < result.count; ++index) {
        const auto& box = result.boxes[index];
        if (!std::isfinite(box.x1) || !std::isfinite(box.y1) || !std::isfinite(box.x2) || !std::isfinite(box.y2) || !std::isfinite(box.conf) || box.x1 < 0 || box.y1 < 0 || box.x2 >= width || box.y2 >= height || box.x2 <= box.x1 || box.y2 <= box.y1 || box.conf < config.conf_thr || box.conf > 1 ||
            box.class_id < 0 || box.class_id >= config.num_classes || (config.target_class >= 0 && box.class_id != config.target_class)) {
            return false;
        }
    }
    return std::isfinite(result.infer_ms) && result.infer_ms >= 0;
}
void selectPrimary(FrameResult& result) {
    result.u_px = result.v_px = result.w_px = result.h_px = result.conf = 0;
    result.class_id = -1;
    if (!result.present) {
        return;
    }
    int best_index = 0;
    for (int index = 1; index < result.count; ++index) {
        if (result.boxes[index].conf > result.boxes[best_index].conf) {
            best_index = index;
        }
    }
    const auto& box = result.boxes[best_index];
    result.u_px = (static_cast<double>(box.x1) + box.x2) * 0.5;
    result.v_px = (static_cast<double>(box.y1) + box.y2) * 0.5;
    result.w_px = box.x2 - box.x1;
    result.h_px = box.y2 - box.y1;
    result.conf = box.conf;
    result.class_id = box.class_id;
}
}  // namespace

bool validateRuntimeConfig(const RuntimeConfig& config, std::string& error) {
    if (config.imgsz != 640 || config.num_classes < 1 || config.num_classes > 1000 || !std::isfinite(config.conf_thr) || config.conf_thr <= 0 || config.conf_thr > 1 || !std::isfinite(config.iou_thr) || config.iou_thr <= 0 || config.iou_thr > 1 || config.max_det < 1 ||
        config.max_det > kMaxDetections || config.max_hz < 1 || config.max_hz > 60 || config.max_age_ms < 1 || config.max_age_ms > 2000 || config.target_class < -1 || config.target_class >= config.num_classes || config.model_path.empty()) {
        error =
            "Invalid detection config: imgsz=640, classes=1..1000, thresholds=(0,1], "
            "max_det=1..300, max_hz=1..60, max_age_ms=1..2000, target_class=-1 or valid class ID";
        return false;
    }
    error.clear();
    return true;
}

cv::Mat decodeColorFrame(const EncodedColorFrame& frame) {
    if (frame.width < 1 || frame.height < 1 || frame.width > 4096 || frame.height > 2160 || frame.bytes.empty() || frame.bytes.type() != CV_8UC1 || !frame.bytes.isContinuous()) {
        throw std::runtime_error("Invalid encoded color frame");
    }
    cv::Mat bgr;
    if (frame.encoding == ColorEncoding::jpeg) {
        bgr = cv::imdecode(frame.bytes, cv::IMREAD_COLOR);
    } else {
        const int channels = frame.encoding == ColorEncoding::yuyv ? 2 : 3;
        const size_t required = static_cast<size_t>(frame.width) * frame.height * channels;
        if (frame.bytes.total() != required || (frame.encoding == ColorEncoding::yuyv && frame.width % 2 != 0)) {
            throw std::runtime_error("Encoded color buffer size does not match profile");
        }
        cv::Mat pixels(frame.height, frame.width, CV_MAKETYPE(CV_8U, channels), frame.bytes.data);
        if (frame.encoding == ColorEncoding::rgb) {
            cv::cvtColor(pixels, bgr, cv::COLOR_RGB2BGR);
        } else if (frame.encoding == ColorEncoding::yuyv) {
            cv::cvtColor(pixels, bgr, cv::COLOR_YUV2BGR_YUYV);
        } else if (frame.encoding == ColorEncoding::bgr) {
            bgr = pixels.clone();
        } else {
            throw std::runtime_error("Unsupported color encoding");
        }
    }
    if (bgr.type() != CV_8UC3 || bgr.cols != frame.width || bgr.rows != frame.height) {
        throw std::runtime_error("Decoded color frame does not match profile");
    }
    return bgr;
}

struct DetectionRuntime::Impl {
    RuntimeConfig config;
    std::unique_ptr<InferenceBackend> backend;
    mutable std::mutex mutex;
    std::condition_variable cv;
    std::thread worker;
    bool stopping = true, ready = false;
    std::atomic<bool> active{false};
    std::optional<EncodedColorFrame> pending;
    TargetSnapshot snapshot;
    Stats counters;
    uint64_t session = 0, generation = 0, epoch = 0, last_seq = 0;
    int64_t cutoff_ns = 0, last_capture_ns = 0, last_admitted_ns = 0;

    Impl(const RuntimeConfig& settings, std::unique_ptr<InferenceBackend> engine) : config(settings), backend(std::move(engine)) {
    }

    void run() noexcept {
        try {
            std::string error;
            if (!backend->load(config, error)) {
                throw std::runtime_error("Detection model load: " + error);
            }
            {
                std::scoped_lock lock(mutex);
                ready = true;
                active.store(true);
            }
            cv.notify_all();
            while (true) {
                EncodedColorFrame frame;
                uint64_t frame_epoch;
                uint64_t frame_session;
                uint64_t frame_generation;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    cv.wait(lock, [&] { return stopping || pending.has_value(); });
                    if (stopping) {
                        break;
                    }
                    frame = std::move(*pending);
                    pending.reset();
                    frame_epoch = epoch;
                    frame_session = session;
                    frame_generation = generation;
                }
                if (!fresh(frame.host_ns, steadyNs(), config.max_age_ms)) {
                    std::scoped_lock lock(mutex);
                    ++counters.stale;
                    continue;
                }
                const cv::Mat bgr = decodeColorFrame(frame);
                FrameResult result{};
                if (!backend->infer(bgr, result, error)) {
                    throw std::runtime_error("Detection inference: " + error);
                }
                if (!validResult(result, config, frame.width, frame.height)) {
                    throw std::runtime_error("Detection backend returned invalid boxes or model classes");
                }
                selectPrimary(result);
                result.host_us = 0;
                result.device_us = frame.device_us > 0 ? static_cast<uint64_t>(frame.device_us) : 0;
                result.seq = frame.seq;
                result.width = frame.width;
                result.height = frame.height;
                const int64_t produced_ns = steadyNs();
                std::scoped_lock lock(mutex);
                if (stopping) {
                    break;
                }
                if (frame_epoch != epoch) {
                    ++counters.invalidated;
                    continue;
                }
                if (!fresh(frame.host_ns, produced_ns, config.max_age_ms)) {
                    ++counters.stale;
                    continue;
                }
                snapshot = {true, result, frame.host_ns, produced_ns, frame_session, frame_generation};
                ++counters.processed;
                if (result.present) {
                    ++counters.detected;
                }
                counters.last_infer_ms = result.infer_ms;
            }
        } catch (const std::exception& error) {
            std::scoped_lock lock(mutex);
            counters.last_error = error.what();
        } catch (...) {
            std::scoped_lock lock(mutex);
            counters.last_error = "Unknown exception in detection worker";
        }
        {
            std::scoped_lock lock(mutex);
            snapshot = {};
            pending.reset();
            ready = true;
            stopping = true;
            active.store(false);
        }
        backend->close();
        cv.notify_all();
    }
};

DetectionRuntime::DetectionRuntime(const RuntimeConfig& config, std::unique_ptr<InferenceBackend> backend) : impl_(std::make_unique<Impl>(config, std::move(backend))) {
}
DetectionRuntime::~DetectionRuntime() {
    stop();
}
bool DetectionRuntime::start(std::string* error) {
    std::string reason;
    if (!validateRuntimeConfig(impl_->config, reason) || !impl_->backend) {
        if (error != nullptr) {
            *error = reason.empty() ? "NPU backend unavailable in this build" : reason;
        }
        return false;
    }
    if (impl_->worker.joinable()) {
        if (error != nullptr) {
            *error = "Detection worker already started";
        }
        return false;
    }
    {
        std::scoped_lock lock(impl_->mutex);
        impl_->stopping = false;
        impl_->ready = false;
        impl_->counters = {};
        impl_->snapshot = {};
        impl_->pending.reset();
        impl_->last_seq = 0;
        impl_->last_capture_ns = impl_->last_admitted_ns = 0;
        impl_->cutoff_ns = steadyNs();
        ++impl_->epoch;
    }
    try {
        impl_->worker = std::thread([this] { impl_->run(); });
    } catch (const std::exception& failure) {
        if (error != nullptr) {
            *error = failure.what();
        }
        return false;
    }
    std::unique_lock<std::mutex> lock(impl_->mutex);
    impl_->cv.wait(lock, [&] { return impl_->ready; });
    if (!impl_->active.load()) {
        if (error != nullptr) {
            *error = impl_->counters.last_error;
        }
        lock.unlock();
        stop();
        return false;
    }
    if (error != nullptr) {
        error->clear();
    }
    return true;
}
void DetectionRuntime::stop() {
    {
        std::scoped_lock lock(impl_->mutex);
        impl_->stopping = true;
        impl_->snapshot = {};
        impl_->pending.reset();
    }
    impl_->cv.notify_all();
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }
}
bool DetectionRuntime::submit(const EncodedColorFrame& frame) noexcept {
    try {
        std::unique_lock<std::mutex> lock(impl_->mutex);
        if (impl_->stopping || !impl_->active.load()) {
            return false;
        }
        if (frame.bytes.empty() || frame.bytes.type() != CV_8UC1 || frame.bytes.total() > 32U * 1024U * 1024U) {
            ++impl_->counters.stale;
            return false;
        }
        if (!fresh(frame.host_ns, steadyNs(), impl_->config.max_age_ms) || frame.host_ns < impl_->cutoff_ns || frame.host_ns <= impl_->last_capture_ns || frame.seq == 0 || frame.seq <= impl_->last_seq) {
            ++impl_->counters.stale;
            return false;
        }
        const int64_t period_ns = 1000000000LL / impl_->config.max_hz;
        if (impl_->last_admitted_ns != 0 && frame.host_ns - impl_->last_admitted_ns < period_ns) {
            ++impl_->counters.rate_limited;
            return false;
        }
        const auto frame_epoch = impl_->epoch;
        impl_->last_admitted_ns = impl_->last_capture_ns = frame.host_ns;
        impl_->last_seq = frame.seq;
        lock.unlock();
        EncodedColorFrame owned = frame;
        owned.bytes = frame.bytes.clone();
        lock.lock();
        if (impl_->stopping || frame_epoch != impl_->epoch || frame.seq != impl_->last_seq) {
            ++impl_->counters.invalidated;
            return false;
        }
        if (impl_->pending) {
            ++impl_->counters.replaced;
        }
        impl_->pending = std::move(owned);
        ++impl_->counters.submitted;
        impl_->cv.notify_one();
        return true;
    } catch (...) {
        std::scoped_lock lock(impl_->mutex);
        impl_->counters.last_error = "Cannot copy encoded detection frame";
        impl_->stopping = true;
        impl_->snapshot = {};
        impl_->pending.reset();
        impl_->cv.notify_all();
        return false;
    }
}
void DetectionRuntime::setEpoch(uint64_t session, uint64_t generation) {
    std::scoped_lock lock(impl_->mutex);
    if (session == impl_->session && generation == impl_->generation) {
        return;
    }
    impl_->session = session;
    impl_->generation = generation;
    ++impl_->epoch;
    impl_->cutoff_ns = steadyNs();
    impl_->last_capture_ns = impl_->last_admitted_ns = 0;
    impl_->pending.reset();
    impl_->snapshot = {};
}
bool DetectionRuntime::latest(TargetSnapshot& out, int64_t now_ns) const {
    std::scoped_lock lock(impl_->mutex);
    if (impl_->stopping || !impl_->active.load() || !impl_->snapshot.valid || !fresh(impl_->snapshot.captured_host_ns, now_ns, impl_->config.max_age_ms) || !fresh(impl_->snapshot.produced_host_ns, now_ns, impl_->config.max_age_ms)) {
        out = {};
        return false;
    }
    out = impl_->snapshot;
    return true;
}
bool DetectionRuntime::running() const {
    return impl_->active.load();
}
DetectionRuntime::Stats DetectionRuntime::stats() const {
    std::scoped_lock lock(impl_->mutex);
    return impl_->counters;
}

}  // namespace pure_uav::detection
