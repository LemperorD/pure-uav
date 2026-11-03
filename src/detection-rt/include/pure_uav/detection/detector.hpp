#pragma once
// =============================================================================
//  检测门面：采集线程 → 最新 BGR 帧槽 → 推理线程 → 最新 FrameResult 槽
//
//  · poll(FrameResult&) 非阻塞：有新结果返回 true 并覆盖 out；
//    没有新结果返回 false 且**不改 out**；永远只返回最新一帧（旧帧可丢）。
//  · 只允许同时存在一个 Detector 实例（awnn 全局状态）。
//  · stop() 正确 join 推理线程、停相机、awnn_destroy()/awnn_uninit()。
// =============================================================================

#include <cstdint>
#include <memory>
#include <string>

#include "pure_uav/detection/camera_source.hpp"
#include "pure_uav/detection/detection_types.hpp"

namespace pure_uav {
namespace detection {

struct DetectorConfig {
    CameraConfig camera;

    // NPU / 后处理
    std::string model_path;
    int imgsz = 640;
    int num_classes = 2;
    float conf_thr = 0.25f;
    float iou_thr = 0.45f;
    int max_det = 300;

    bool verbose = false;
};

struct DetectorStats {
    double capture_fps = 0.0;     // 采集帧率
    double process_fps = 0.0;     // 处理（发布结果）帧率
    double avg_infer_ms = 0.0;    // NPU 前向 + 融合平均耗时
    uint64_t capture_frames = 0;  // 采集帧数
    uint64_t process_frames = 0;  // 处理帧数
};

class Detector {
public:
    Detector();
    ~Detector();

    Detector(const Detector&) = delete;
    Detector& operator=(const Detector&) = delete;

    bool start(const DetectorConfig& cfg, std::string& error);
    void stop();

    bool running() const;
    bool alive() const;

    bool poll(FrameResult& out);
    DetectorStats stats() const;

    // 最近一次已发布结果对应的 BGR 帧（浅拷贝），仅供可视化 / --dump-dir 使用。
    bool latest_frame(cv::Mat& out) const;

    // 相机实际打开的 profile（"WxH@FPS fmtF"），供 CLI 打印。
    std::string cameraProfile() const;
    // NBG 输入/输出与量化信息。
    std::string npuInfo() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace detection
}  // namespace pure_uav
