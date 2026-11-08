#include "pure_uav/detection/detector.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <opencv2/core.hpp>
#include <thread>
#include <utility>

#include "pure_uav/detection/yolo_npu.hpp"
#include "yolo_postprocess.hpp"

namespace pure_uav {
namespace detection {
namespace {

// awnn 是进程级全局状态，只允许一个 Detector 实例。
std::atomic<bool> g_instance_used{false};

double nowSeconds() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

struct Detector::Impl {
    DetectorConfig cfg;
    CameraSource camera;
    YoloNpu npu;
    std::thread infer_thread;

    std::atomic<bool> running{false};
    std::atomic<bool> alive{false};
    double start_sec = 0.0;

    // 采集线程 → 最新 BGR 帧槽
    std::mutex frame_mu;
    std::condition_variable frame_cv;
    cv::Mat latest_bgr;
    uint64_t latest_host_us = 0;
    uint64_t latest_device_us = 0;
    uint64_t latest_seq = 0;
    bool has_frame = false;
    uint64_t captured = 0;

    // 推理线程 → 最新结果槽
    std::mutex result_mu;
    FrameResult latest;
    cv::Mat latest_result_bgr;
    bool result_new = false;
    uint64_t processed = 0;
    double infer_sum_ms = 0.0;
    std::string last_infer_error;

    void onCameraFrame(const CameraFrame& frame) {
        {
            std::lock_guard<std::mutex> lk(frame_mu);
            latest_bgr = frame.bgr;  // cv::Mat 浅拷贝（引用计数），数据仍归本槽所有
            latest_host_us = frame.host_us;
            latest_device_us = frame.device_us;
            latest_seq = frame.seq;
            has_frame = true;
            captured++;
        }
        frame_cv.notify_one();
    }

    void inferLoop() {
        PostprocessParams pp;
        while (running.load(std::memory_order_relaxed)) {
            cv::Mat bgr;
            uint64_t host_us = 0, device_us = 0, seq = 0;
            {
                std::unique_lock<std::mutex> lk(frame_mu);
                frame_cv.wait(lk, [this] { return !running.load(std::memory_order_relaxed) || has_frame; });
                if (!running.load(std::memory_order_relaxed)) break;
                if (!has_frame) continue;
                bgr = latest_bgr;
                host_us = latest_host_us;
                device_us = latest_device_us;
                seq = latest_seq;
                has_frame = false;  // 消费掉，始终只处理最新一帧
            }
            if (bgr.empty()) continue;

            NpuOutputView view;
            std::string error;
            if (!npu.infer(bgr, view, error)) {
                std::lock_guard<std::mutex> lk(result_mu);
                last_infer_error = error;
                continue;
            }

            pp.num_classes = cfg.num_classes;
            pp.conf_thr = cfg.conf_thr;
            pp.iou_thr = cfg.iou_thr;
            pp.max_det = cfg.max_det;
            pp.img_w = bgr.cols;
            pp.img_h = bgr.rows;

            FrameResult r{};
            r.host_us = host_us;
            r.device_us = device_us;
            r.seq = seq;
            r.width = bgr.cols;
            r.height = bgr.rows;
            r.infer_ms = view.infer_ms;
            postprocess(view, pp, r);

            {
                std::lock_guard<std::mutex> lk(result_mu);
                latest = r;
                latest_result_bgr = bgr;
                result_new = true;
                processed++;
                infer_sum_ms += view.infer_ms;
                last_infer_error.clear();
            }
        }
    }
};

Detector::Detector() : impl_(std::make_unique<Impl>()) {
}

Detector::~Detector() {
    stop();
}

bool Detector::start(const DetectorConfig& cfg, std::string& error) {
    if (impl_->running.load()) return true;

    // 只允许一个 Detector 实例（awnn 全局状态）
    if (g_instance_used.exchange(true)) {
        error = "已存在一个 Detector 实例（awnn 为进程级全局状态）";
        return false;
    }

    impl_->cfg = cfg;
    if (impl_->cfg.max_det <= 0) impl_->cfg.max_det = 300;
    impl_->cfg.max_det = std::min(impl_->cfg.max_det, kMaxDetections);
    if (impl_->cfg.imgsz != 640) {
        error = "只支持 imgsz=640，收到 " + std::to_string(impl_->cfg.imgsz);
        g_instance_used.store(false);
        return false;
    }

    // 先加载 NPU，模型错误在打开相机之前暴露
    if (!impl_->npu.load(cfg.model_path, cfg.imgsz, error)) {
        g_instance_used.store(false);
        return false;
    }

    if (!impl_->camera.start(cfg.camera, [this](const CameraFrame& f) { impl_->onCameraFrame(f); }, &error)) {
        impl_->npu.release();
        g_instance_used.store(false);
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(impl_->frame_mu);
        impl_->has_frame = false;
        impl_->captured = 0;
        impl_->latest_bgr.release();
    }
    {
        std::lock_guard<std::mutex> lk(impl_->result_mu);
        impl_->latest = FrameResult{};
        impl_->latest_result_bgr.release();
        impl_->result_new = false;
        impl_->processed = 0;
        impl_->infer_sum_ms = 0.0;
        impl_->last_infer_error.clear();
    }
    impl_->start_sec = nowSeconds();

    impl_->running.store(true);
    impl_->alive.store(true);
    impl_->infer_thread = std::thread([this] { impl_->inferLoop(); });
    return true;
}

void Detector::stop() {
    if (!impl_) return;
    if (!impl_->running.load() && !impl_->infer_thread.joinable()) {
        if (impl_->alive.load()) impl_->alive.store(false);
        return;
    }

    impl_->running.store(false);
    impl_->frame_cv.notify_all();

    // 先停相机（join 采集线程），之后不再有回调写帧槽
    impl_->camera.stop();
    if (impl_->infer_thread.joinable()) impl_->infer_thread.join();

    impl_->npu.release();
    {
        std::lock_guard<std::mutex> lk(impl_->frame_mu);
        impl_->latest_bgr.release();
        impl_->has_frame = false;
    }

    {
        std::lock_guard<std::mutex> lk(impl_->result_mu);
        impl_->latest_result_bgr.release();
    }

    impl_->alive.store(false);
    g_instance_used.store(false);
}

bool Detector::running() const {
    return impl_ && impl_->running.load();
}

bool Detector::alive() const {
    return impl_ && impl_->alive.load() && impl_->running.load() && impl_->camera.running();
}

bool Detector::poll(FrameResult& out) {
    std::lock_guard<std::mutex> lk(impl_->result_mu);
    if (!impl_->result_new) return false;
    out = impl_->latest;
    impl_->result_new = false;
    return true;
}

bool Detector::latest_frame(cv::Mat& out) const {
    std::lock_guard<std::mutex> lk(impl_->result_mu);
    if (impl_->latest_result_bgr.empty()) return false;
    out = impl_->latest_result_bgr;
    return true;
}

DetectorStats Detector::stats() const {
    DetectorStats s;
    const double elapsed = std::max(1e-6, nowSeconds() - impl_->start_sec);

    {
        std::lock_guard<std::mutex> lk(impl_->frame_mu);
        s.capture_frames = impl_->captured;
        s.capture_fps = static_cast<double>(impl_->captured) / elapsed;
    }
    {
        std::lock_guard<std::mutex> lk(impl_->result_mu);
        s.process_frames = impl_->processed;
        s.process_fps = static_cast<double>(impl_->processed) / elapsed;
        s.avg_infer_ms = impl_->processed > 0 ? impl_->infer_sum_ms / static_cast<double>(impl_->processed) : 0.0;
    }
    return s;
}

std::string Detector::cameraProfile() const {
    return impl_ ? impl_->camera.activeProfile() : std::string();
}

std::string Detector::npuInfo() const {
    return impl_ ? impl_->npu.info() : std::string();
}

}  // namespace detection
}  // namespace pure_uav
