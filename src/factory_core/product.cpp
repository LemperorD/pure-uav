#include "product.hpp"

#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace pure_uav::app {

OpenVINS::OpenVINS(const VioProductConfig& config) : runtime_(config.options, config.imu, config.frames, config.publication) {
}
OpenVINS::~OpenVINS() {
    stop();
}
void OpenVINS::init() {
    if (initialized_) {
        return;
    }
    std::string error;
    if (!runtime_.start(&error)) {
        throw std::runtime_error("[VIO] 启动失败：" + error);
    }
    initialized_ = true;
    std::printf("[VIO] 已启动：IMU 来自通信模块逐帧队列，图像来自 camera_server 共享内存。\n");
}
void OpenVINS::stop() {
    runtime_.stop();
    initialized_ = false;
}
bool OpenVINS::initialized() const {
    return initialized_;
}
bool OpenVINS::update(VioPoseSnapshot& out) const {
    return runtime_.latestPose(out);
}
bool OpenVINS::running() const {
    return runtime_.running();
}
VioRuntime::Stats OpenVINS::stats() const {
    return runtime_.stats();
}
VioRuntime::PublicationGuard OpenVINS::publication() const {
    return runtime_.publication();
}

PNGuidance::PNGuidance(GuidanceProductConfig config) : config_(std::move(config)) {
}
void PNGuidance::setVIO(std::shared_ptr<IVIO> vio) {
    if (service_) {
        throw std::logic_error("cannot change VIO after guidance initialization");
    }
    if (!vio) {
        throw std::invalid_argument("guidance requires a VIO product");
    }
    vio_ = std::move(vio);
}
PNGuidance::~PNGuidance() {
    stop();
}
void PNGuidance::init() {
    if (initialized_) {
        return;
    }
    if (!vio_ || !vio_->initialized()) {
        throw std::logic_error("initialize VIO before guidance");
    }
    const auto& settings = config_.settings;
    service_ = std::make_unique<MissionService>(*vio_, config_.comm, settings.mission, settings.guidance, settings.camera, settings.preflight, settings.app.comm.heartbeat_timeout_ms, settings.app.max_age_ms, !config_.options.no_detection);
    initialized_ = true;
    std::printf("[MISSION] enabled send=%s guidance=%s; waiting for navigation\n", config_.options.no_send ? "false" : "true", service_->guidanceReady() ? "ready" : "unconfigured");
    const auto& log = settings.guidance.log;
    std::printf("[GUIDANCE] 中间量日志：%s，%.0f Hz，输出=%s\n", log.enabled ? "开" : "关", log.rate_hz, log.file.empty() ? "stdout" : log.file.c_str());
}
void PNGuidance::stop() {
    if (service_) {
        service_->stop();
    }
    // 检测监督仍可能持有目标 fd，通道随产品析构关闭。
    initialized_ = false;
}
bool PNGuidance::initialized() const {
    return service_ && initialized_;
}
const mission::RuntimeStatus& PNGuidance::guide(bool return_requested) {
    if (!initialized()) {
        throw std::logic_error("initialize guidance before guide");
    }
    return service_->tick(return_requested);
}
int PNGuidance::targetPublisherFd() const {
    if (!initialized()) {
        throw std::logic_error("initialize guidance before creating the detection channel");
    }
    return service_->targetPublisherFd();
}

YoloProcessDetector::YoloProcessDetector(DetectionProductConfig config) : options_(std::move(config.options)) {
}
void YoloProcessDetector::setGuidance(std::shared_ptr<IGuidance> guidance) {
    if (supervisor_) {
        throw std::logic_error("stop detection before changing guidance");
    }
    guidance_ = std::move(guidance);
}
YoloProcessDetector::~YoloProcessDetector() {
    stop();
}
void YoloProcessDetector::init() {
    if (supervisor_) {
        return;
    }
    if (options_.mission && !guidance_) {
        throw std::logic_error("connect guidance before mission detection");
    }
#ifdef PURE_APP_HAS_DETECTION
    DetectionProcessOptions process;
    if (options_.detect_bin.empty()) {
        process.executable = (std::filesystem::read_symlink("/proc/self/exe").parent_path() / "detect_rt").string();
    } else {
        process.executable = options_.detect_bin;
    }
    if (!options_.detection_config.empty()) {
        process.arguments.emplace_back("--config");
        process.arguments.push_back(options_.detection_config);
    }
    for (const auto& argument : options_.detect_args) {
        if (argument == "--mission-target-fd") {
            throw std::invalid_argument("--mission-target-fd is managed by main");
        }
        process.arguments.push_back(argument);
    }
    if (guidance_) {
        process.target_publish_fd = guidance_->targetPublisherFd();
        process.arguments.emplace_back("--mission-target-fd");
        process.arguments.push_back(std::to_string(process.target_publish_fd));
    }
    auto supervisor = std::make_unique<DetectionSupervisor>(std::move(process));
    std::string error;
    if (!supervisor->start(&error)) {
        throw std::runtime_error("[MAIN] 检测启动失败：" + error);
    }
    supervisor_ = std::move(supervisor);
#else
    throw std::runtime_error("[MAIN] 当前构建缺少 detect_rt；可用 --no-detection 只运行 VIO");
#endif
}
void YoloProcessDetector::stop() {
    if (supervisor_) {
        supervisor_->stop();
        supervisor_.reset();
    }
}
bool YoloProcessDetector::initialized() const {
    return supervisor_ != nullptr;
}
bool YoloProcessDetector::detect(SuperviseResult& result) const {
    return supervisor_ && supervisor_->finished(result);
}

}  // namespace pure_uav::app
