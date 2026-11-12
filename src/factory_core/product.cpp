#include "product.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include "yolo_backend.hpp"
#include "yolo_postprocess.hpp"

namespace {
// 复用原有 YOLO 推理与后处理；兼容板端 NPU 及开发机 CPU 构建。
class YoloRuntimeBackend final : public pure_uav::detection::InferenceBackend {
  public:
    bool load(const pure_uav::detection::RuntimeConfig& config, std::string& error) override {
        config_ = config;
        return backend_.load(config.model_path, config.imgsz, error);
    }
    bool infer(const cv::Mat& bgr, pure_uav::detection::FrameResult& result, std::string& error) override {
        pure_uav::detection::YoloOutputView output;
        if (!backend_.infer(bgr, output, error)) {
            return false;
        }
        pure_uav::detection::PostprocessParams params;
        params.num_classes = config_.num_classes;
        params.conf_thr = config_.conf_thr;
        params.iou_thr = config_.iou_thr;
        params.max_det = config_.max_det;
        params.img_w = bgr.cols;
        params.img_h = bgr.rows;
        result.infer_ms = output.infer_ms;
        if (!pure_uav::detection::postprocess(output, params, result)) {
            error = "Unsupported YOLO output layout";
            return false;
        }
        if (config_.target_class >= 0) {
            int kept = 0;
            for (int index = 0; index < result.count; ++index) {
                if (result.boxes[index].class_id == config_.target_class) {
                    result.boxes[kept++] = result.boxes[index];
                }
            }
            result.count = kept;
            result.present = kept > 0;
        }
        return true;
    }
    void close() noexcept override {
        backend_.release();
    }

  private:
    pure_uav::detection::RuntimeConfig config_;
    pure_uav::detection::YoloBackend backend_;
};

std::unique_ptr<pure_uav::detection::InferenceBackend> createBackend(const DetectionProductConfig& config) {
    auto backend = config.create_backend ? config.create_backend() : std::make_unique<YoloRuntimeBackend>();
    if (!backend) {
        throw std::invalid_argument("Detection backend creator returned null");
    }
    return backend;
}
}  // namespace

YoloA733Detector::YoloA733Detector(const DetectionProductConfig& config) : runtime_(config.runtime, createBackend(config)) {
}
YoloA733Detector::~YoloA733Detector() {
    stop();
}
void YoloA733Detector::init() {
    if (initialized_) {
        return;
    }
    std::string error;
    if (!runtime_.start(&error)) {
        throw std::runtime_error("Detector initialization failed: " + error);
    }
    initialized_ = true;
}
void YoloA733Detector::stop() {
    runtime_.stop();
    initialized_ = false;
}
bool YoloA733Detector::initialized() const {
    return initialized_;
}
bool YoloA733Detector::submit(const pure_uav::EncodedColorFrame& frame) {
    return initialized_ && runtime_.submit(frame);
}
void YoloA733Detector::setEpoch(uint64_t session, uint64_t generation) {
    runtime_.setEpoch(session, generation);
}
bool YoloA733Detector::detect(pure_uav::detection::TargetSnapshot& out, int64_t now_ns) {
    if (!initialized_) {
        out = {};
        return false;
    }
    return runtime_.latest(out, now_ns);
}

OpenVINS::OpenVINS(VioProductConfig config) : config_(std::move(config)) {
    if (!config_.imu || !config_.frames) {
        throw std::invalid_argument("OpenVINS requires IMU and frame sources");
    }
    runtime_ = std::make_unique<pure_uav::VioRuntime>(config_.options, *config_.imu, *config_.frames, config_.policy);
}
OpenVINS::~OpenVINS() {
    stop();
}
void OpenVINS::init() {
    if (initialized_) {
        return;
    }
    std::string error;
    if (!runtime_->start(&error)) {
        throw std::runtime_error("OpenVINS initialization failed: " + error);
    }
    initialized_ = true;
}
void OpenVINS::stop() {
    runtime_->stop();
    initialized_ = false;
}
bool OpenVINS::initialized() const {
    return initialized_;
}
bool OpenVINS::update(pure_uav::VioPoseSnapshot& out) {
    out = {};
    // 快照不是发送许可。实际发送须沿用 VioRuntime::publication() 保护与宿主门控。
    return initialized_ && runtime_->latestPose(out);
}
pure_uav::VioRuntime::PublicationGuard OpenVINS::publication() const {
    return runtime_->publication();
}

PNGuidance::PNGuidance(GuidanceProductConfig config) : config_(std::move(config)) {
    if (!config_.ins || !config_.target || !config_.sink) {
        throw std::invalid_argument("PN guidance requires INS, target and command ports");
    }
}
PNGuidance::~PNGuidance() {
    stop();
}
void PNGuidance::init() {
    if (loop_) {
        return;
    }
    // GuidanceLoop::init 不会清除 request_stop 标记，重启时创建新循环。
    auto loop = std::make_unique<pure_uav::guidance::flight::GuidanceLoop>(config_.params, config_.loop, *config_.ins, *config_.target, *config_.sink);
    if (!loop->init()) {
        throw std::runtime_error("PN guidance initialization failed: invalid parameters or camera calibration");
    }
    loop_ = std::move(loop);
}
void PNGuidance::stop() {
    if (loop_) {
        loop_->request_stop();
        loop_.reset();
    }
}
bool PNGuidance::initialized() const {
    return loop_ != nullptr;
}
pure_uav::guidance::flight::GuidanceLoopStatus PNGuidance::guide(pure_uav::guidance::flight::GuidanceOutput* out) {
    if (out != nullptr) {
        *out = {};
    }
    if (!loop_) {
        return pure_uav::guidance::flight::GuidanceLoopStatus::kNotInitialized;
    }
    return loop_->run_once(out);
}
