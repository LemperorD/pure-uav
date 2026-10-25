// Orbbec Gemini 彩色相机（灰度输出）。
//
// 只用彩色流（VIO 不需要深度），MJPG 解码为 mono8；时间戳用帧到达主机的
// CLOCK_MONOTONIC 时间，再由 FcuClock 映射到飞控 IMU 时钟。

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace ob {
class Pipeline;
}

namespace pure_uav {

class OrbbecCamera {
public:
    struct Config {
        int width = 1920;
        int height = 1080;
        int fps = 30;

        // 相机控制。默认值与标定采集（capture_config.py）保持一致：
        // 自动曝光每帧改曝光时间 → 相机时间戳的"曝光中点"漂移几毫秒到几十毫秒，
        // 这正是 cam-IMU 时移估计要追的东西，必须锁死。
        bool auto_exposure = false;
        int exposure = 100;        // 单位 100us：100 = 10 ms（同 V4L2 的 EXPOSURE=100）
        int gain = 30;             // 同 capture_config.py 的 GAIN=30
        int white_balance = 4600;  // K

        // 【2026-10-20】影响亮度、但以前一个都没碰过的控件。
        // 支持的写死，不支持的跳过（只打印，不当作失败）——否则会把能用的相机挡在门外。
        int brightness = 0;
        int contrast = 32;
        int sharpness = 3;
        int backlight_compensation = 0;  // 逆光补偿会在 AE 漏网时二次改测光，必须关

        // 锁定控件后丢弃的帧数。改动生效前那几帧还是旧曝光，进了估计器会让首帧状态偏。
        int warmup_frames = 15;

        // 【2026-10-20】进场测光：曝光先顶到「模糊预算」和「帧周期」的较小值，
        // 还不够就爬增益，选定后整段飞行锁死。
        // 写死一组曝光/增益只在"标定那天的光"下成立 —— Python 采集端踩过这个坑
        // （同一间教室，晚上只有日间 1/8 的亮度，同一份代码直接发散）。
        // 可复现靠的是"飞行期间不动"，不是"永远用同一个数字"。
        bool auto_tune_exposure = true;
        int tune_target_mean = 100;        // 目标灰度均值
        int tune_min_mean = 45;            // 最好也只能到这以下 → 拒绝启动

        // 【2026-10-21】曝光上限不再是拍脑袋的 15 ms，而是从运动模糊预算反推：
        //     blur_px = f * ω * t_exp
        // 实测（session29，18 ms 曝光）：模糊 >12 px 时 KLT 存活率掉到 55%，
        // >20 px 掉到 38%，而且 FAST 检出数也从 400 掉到 25——角点直接被糊没了。
        // 所以「曝光时长」必须是「预期角速度」的函数，不能写死。
        double tune_blur_budget_px = 3.0;     // 允许的运动模糊上限（像素）
        double tune_expected_rate_dps = 100.0;  // 预期飞行最大角速度（度/秒）
        double tune_focal_px = 1367.5;        // 与 kalibr_imucam_chain.yaml 的 fx 一致
        double tune_max_exposure_ms = 15.0;   // 绝对上限（帧周期以外再兜一层）
        int tune_max_gain = 40;               // 增益上限：100 的噪声会把 KLT 拖死
        bool tune_allow_blur = false;         // 明知会糊也继续（只用于排查）
        bool tune_allow_dark = false;         // 测光不达标也硬飞（只用于排查）

        // 【2026-10-21】是否打开深度流（对齐到彩色）。深度给米制尺度，
        // 让单目 VIO 不再依赖平移视差 —— 直接补上和双目方案的架构差距。
        bool use_depth = true;
        // 【2026-10-23】帧集聚合模式。当年为拿到深度改成 FULL_FRAME_REQUIRE（0），
        // 代价是彩色被拽到深度的帧率（实测 640x480@60 + 深度只交付 31 fps）。
        //   0 = FULL_FRAME_REQUIRE   两路都齐才出帧（旧行为）
        //   1 = COLOR_FRAME_REQUIRE  彩色到就出帧，深度有就带上 ← 想要的
        //   2 = ANY_SITUATION        谁先到都单独出帧
        int aggregate_mode = 0;
    };

    // 回调在相机线程执行：gray 是单通道 8 位图，host_ns 是帧到达主机的时刻，
    // device_us 是**相机自己的帧时间戳**（SDK 的 getTimeStampUs）。
    // 两者之差 = 编码 + USB 传输 + 解码 + 排队 的整条链路延迟。
    // 它是否稳定，直接决定相机时间戳能不能当曝光时刻用（VIO 的时间基准）。
    /// depth_mm：对齐到彩色的深度图（CV_16UC1，单位毫米）。没开深度流时是空 Mat。
    using FrameCallback = std::function<void(const cv::Mat& gray, const cv::Mat& depth_mm,
                                             int64_t host_ns, int64_t device_us,
                                             int64_t depth_device_us)>;

    OrbbecCamera();
    ~OrbbecCamera();

    OrbbecCamera(const OrbbecCamera&) = delete;
    OrbbecCamera& operator=(const OrbbecCamera&) = delete;

    bool start(const Config& cfg, FrameCallback callback, std::string* error);
    void stop();

    /// 深度相机内参（fx,fy,cx,cy）与到彩色的平移（mm）。没开深度流时全 0。
    struct DepthInfo {
        double fx = 0, fy = 0, cx = 0, cy = 0;
        int width = 0, height = 0;
        double t_mm[3] = {0.0, 0.0, 0.0};   // 彩色→深度 的平移（mm），已含 −Rᵀt
        double R_c2d[9] = {1,0,0, 0,1,0, 0,0,1};  // 彩色→深度 旋转（行主序）
        bool valid() const { return fx > 0 && width > 0; }
    };
    DepthInfo depthInfo() const;

    bool running() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace pure_uav
