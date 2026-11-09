#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "color_frame.hpp"
#include "detection_types.hpp"

namespace pure_uav::detection {

struct RuntimeConfig {
    std::string model_path = "assets/models/yolo26n.nb";
    int imgsz = 640;
    int num_classes = 2;
    float conf_thr = 0.25f;
    float iou_thr = 0.45f;
    int max_det = 100;
    int target_class = -1;
    int max_hz = 10;
    int max_age_ms = 200;
};

class InferenceBackend {
  public:
    virtual ~InferenceBackend() = default;
    virtual bool load(const RuntimeConfig& config, std::string& error) = 0;
    virtual bool infer(const cv::Mat& bgr, FrameResult& result, std::string& error) = 0;
    virtual void close() noexcept = 0;
};

bool npuBackendAvailable();
std::unique_ptr<InferenceBackend> makeNpuBackend();
bool validateRuntimeConfig(const RuntimeConfig& config, std::string& error);
cv::Mat decodeColorFrame(const EncodedColorFrame& frame);

struct TargetSnapshot {
    bool valid = false;
    FrameResult result{};
    int64_t captured_host_ns = 0;
    int64_t produced_host_ns = 0;
    uint64_t session = 0;
    uint64_t generation = 0;
};

class DetectionRuntime {
  public:
    struct Stats {
        uint64_t submitted = 0;
        uint64_t replaced = 0;
        uint64_t rate_limited = 0;
        uint64_t stale = 0;
        uint64_t invalidated = 0;
        uint64_t processed = 0;
        uint64_t detected = 0;
        double last_infer_ms = 0;
        std::string last_error;
    };

    DetectionRuntime(const RuntimeConfig& config, std::unique_ptr<InferenceBackend> backend);
    ~DetectionRuntime();
    DetectionRuntime(const DetectionRuntime&) = delete;
    DetectionRuntime& operator=(const DetectionRuntime&) = delete;
    bool start(std::string* error);
    void stop();
    bool submit(const EncodedColorFrame& frame) noexcept;
    void setEpoch(uint64_t session, uint64_t generation);
    bool latest(TargetSnapshot& out, int64_t now_ns) const;
    bool running() const;
    Stats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace pure_uav::detection
