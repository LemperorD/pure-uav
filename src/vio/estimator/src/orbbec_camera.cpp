#include "orbbec_camera.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

namespace pure_uav {

namespace {
int64_t steadyNowNs() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

cv::Mat frameToGray(const std::shared_ptr<ob::ColorFrame>& frame) {
    const int w = frame->width();
    const int h = frame->height();
    const OBFormat format = frame->format();

    if (format == OB_FORMAT_MJPG) {
        cv::Mat raw(1, static_cast<int>(frame->dataSize()), CV_8UC1, frame->data());
        return cv::imdecode(raw, cv::IMREAD_GRAYSCALE);
    }
    if (format == OB_FORMAT_RGB) {
        cv::Mat rgb(h, w, CV_8UC3, frame->data());
        cv::Mat gray;
                
        cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
        return gray;
    }
    if (format == OB_FORMAT_YUYV) {
        cv::Mat yuyv(h, w, CV_8UC2, frame->data());
        cv::Mat gray;
        cv::cvtColor(yuyv, gray, cv::COLOR_YUV2GRAY_YUYV);
        return gray;
    }
    if (format == OB_FORMAT_Y16 || format == OB_FORMAT_Z16) {
        cv::Mat y16(h, w, CV_16UC1, frame->data());
        cv::Mat gray;
        y16.convertTo(gray, CV_8U, 1.0 / 256.0);
        return gray;
    }
    return cv::Mat();
}
}  // namespace

struct OrbbecCamera::Impl {
    Config cfg;
    FrameCallback callback;
    std::unique_ptr<ob::Pipeline> pipe;
    std::thread thread;
    std::atomic<bool> running{false};

    // start() 之前相机的真实控制值，退出时写回去。
    // 【2026-10-20】以前退出从不恢复 → 相机把"锁定 10ms"的状态留给下一个进程，
    // 下一次打开时读到的 original_controls 就成了上一轮的锁定值，
    // 于是"同一段代码第二次跑，亮度不一样"。
    struct Snapshot {
        bool valid = false;
        int auto_exposure = 0;
        int exposure = 0;
        int gain = 0;
        int auto_white_balance = 0;
        int white_balance = 0;
    } snapshot;

    // 【2026-10-21】深度：对齐到彩色的深度图（毫米）
    bool depth_enabled = false;
    std::unique_ptr<ob::Align> align;
    long long laser_tick = 0;   // 激光重开节流计数器
    // 深度相机内参 + 到彩色的外参（给"彩色像素 → 深度"投影用）
    double depth_fx = 0, depth_fy = 0, depth_cx = 0, depth_cy = 0;
    int depth_w = 0, depth_h = 0;
    double color_fx = 0, color_fy = 0, color_cx = 0, color_cy = 0;
    OBExtrinsic T_color_to_depth{};

    //=============================================================
    // 【2026-10-23】运行中自动重测光（重要）
    //
    // 实测 long1：一趟里画面均值从 236（过曝）掉到 17（几乎全黑），14 倍变化。
    // 曝光只在开跑时锁一次，走到亮处就打爆、走到暗处就没纹理 —— FAST 掉到 55，
    // 转弯时"没有参照"直接崩。这不是运动模糊，是曝光锁死。
    // 这里保存边界与上限，运行中按滞回+限速重新调整。
    //=============================================================
    int exp_min = 1, exp_max = 5000, exp_step = 1;
    int gain_min = 0, gain_max = 100;
    int blur_cap = 104;      // 模糊预算允许的最大曝光（0.1ms 单位）
    int light_cap = 150;     // 现场光允许的最大曝光
    int gain_cap = 40;
    bool auto_retune = true;
};

OrbbecCamera::OrbbecCamera() = default;
OrbbecCamera::~OrbbecCamera() { stop(); }

bool OrbbecCamera::start(const Config& cfg, FrameCallback callback, std::string* error) {
    if (impl_ && impl_->running.load()) return true;

    auto impl = std::make_unique<Impl>();
    impl->cfg = cfg;
    impl->callback = std::move(callback);

    try {
        impl->pipe = std::make_unique<ob::Pipeline>();

        auto profiles = impl->pipe->getStreamProfileList(OB_SENSOR_COLOR);
        std::shared_ptr<ob::VideoStreamProfile> profile;

        // 优先 MJPG（USB 带宽小），其次 RGB、YUYV
        for (const OBFormat fmt : {OB_FORMAT_MJPG, OB_FORMAT_RGB, OB_FORMAT_YUYV}) {
            try {
                profile = profiles->getVideoStreamProfile(cfg.width, cfg.height, fmt, cfg.fps);
                if (profile) break;
            } catch (const ob::Error&) {
                // 尝试下一种格式
            }
        }
        if (!profile) {
            // 最后回退：任意格式，但分辨率/帧率必须匹配
            try {
                profile = profiles->getVideoStreamProfile(cfg.width, cfg.height, OB_FORMAT_ANY,
                                                          cfg.fps);
            } catch (const ob::Error&) {
                profile = nullptr;
            }
        }

        if (!profile) {
            if (error) {
                *error = "找不到 " + std::to_string(cfg.width) + "x" + std::to_string(cfg.height) +
                         "@" + std::to_string(cfg.fps) + "fps 的彩色流";
                // 列出可用 profile 方便排查
                const uint32_t n = profiles->count();
                *error += "；可用彩色 profile:";
                for (uint32_t i = 0; i < n; ++i) {
                    auto p = profiles->getProfile(i);
                    auto vp = p->as<ob::VideoStreamProfile>();
                    if (!vp) continue;
                    *error += " " + std::to_string(vp->width()) + "x" +
                              std::to_string(vp->height()) + "@" +
                              std::to_string(vp->fps()) + "f" +
                              std::to_string(static_cast<int>(vp->format()));
                }
            }
            return false;
        }

        auto config = std::make_shared<ob::Config>();
        config->enableStream(profile);

        //=======================================================================
        // 【2026-10-21】打开深度流并对齐到彩色
        //
        // 为什么：整套系统做的是**单目** VIO，而单目的致命处是"没有平移就没有视差
        // → 没有三角化 → 没有测量"（实测原地转时三角化率 0.04、直接发散）。
        // 别人的"IMU + 双目"不存在这个问题，因为基線直接给米制深度。
        // Gemini Pro 内部就有双 IR + 深度引擎，之前一直没用 —— 这里把它接上。
        //
        // 约束：USB2.0（480 Mbps）装不下两路 IR 原图，但深度是**相机内部算好的**，
        // 只输出一帧，所以只要一路带宽。
        //=======================================================================
        bool depth_ok = false;
        if (cfg.use_depth) {
            try {
                auto dprofiles = impl->pipe->getStreamProfileList(OB_SENSOR_DEPTH);
                std::printf("[Orbbec] 可用深度档位:");
                for (uint32_t i = 0; i < dprofiles->count(); ++i) {
                    auto vp = dprofiles->getProfile(i)->as<ob::VideoStreamProfile>();
                    if (!vp) continue;
                    std::printf(" %ux%u@%u", vp->width(), vp->height(), vp->fps());
                }
                std::printf("\n");
                std::shared_ptr<ob::VideoStreamProfile> dprofile;
                // 【2026-10-21】选**最小够用**的深度档位，不是最大。
                // 1280x800 Y16 @30 = 61 MB/s，单独就把 USB2 吃满 —— 实测相机时间戳抖动
                // 从 2 ms 涨到 10~12 ms，VIO 的时间基准直接变差。
                // 640x400 @30 只要 15 MB/s，深度图是给稀疏特征取值的，分辨率够用。
                // 【2026-10-23】彩色和深度**不再要求同帧率**。
                // 原因：彩色想跑 60 fps 换更大的运动包络，但深度 640x400@60 的原始带宽
                // 是 30.7 MB/s，加上彩色 MJPG@60 已经贴着 USB2 实际上限 —— 实测两个流都
                // 报告 @60，实际只交付 31 fps（帧间隔 32.9 ms）。
                // 深度只是给稀疏特征取值用的，30 fps 完全够；彩色 60 fps 才是要的。
                // 选法：优先同帧率；没有就取"不超过 30 的最高帧率"。
                const int depth_fps_want = std::min(cfg.fps, 30);
                int best_w = 1 << 30;
                int best_fps = -1;
                for (uint32_t i = 0; i < dprofiles->count(); ++i) {
                    auto vp = dprofiles->getProfile(i)->as<ob::VideoStreamProfile>();
                    if (!vp) continue;
                    const int fps_i = static_cast<int>(vp->fps());
                    const int w = static_cast<int>(vp->width());
                    if (w < 640) continue;
                    if (fps_i > depth_fps_want) continue;
                    // 先比帧率（越接近 depth_fps_want 越好），再比宽度（越小越省带宽）
                    const bool better = (fps_i > best_fps) ||
                                        (fps_i == best_fps && w < best_w);
                    if (better) { best_fps = fps_i; best_w = w; dprofile = vp; }
                }
                if (!dprofile) {
                    dprofile = dprofiles->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY,
                                                                OB_FORMAT_ANY, cfg.fps);
                }
                if (dprofile) {
                    config->enableStream(dprofile);
                    // 结构光必须开红外激光，否则深度全是 0（实测默认状态下有效像素 0%）
                    try {
                        if (impl->pipe->getDevice()->isPropertySupported(OB_PROP_LASER_BOOL,
                                                                        OB_PERMISSION_WRITE)) {
                            impl->pipe->getDevice()->setBoolProperty(OB_PROP_LASER_BOOL, true);
                            std::printf("[Orbbec] 已打开红外激光（结构光）\n");
                        }
                    } catch (const ob::Error &e) {
                        std::printf("[Orbbec] ⚠️ 打开激光失败：%s\n", e.getMessage());
                    }
                    depth_ok = true;
                    // 取出深度内参和到彩色的外参 —— 估计器要用它把彩色像素投到深度图上。
                    try {
                        auto di = dprofile->getIntrinsic();
                        impl->depth_fx = di.fx;
                        impl->depth_fy = di.fy;
                        impl->depth_cx = di.cx;
                        impl->depth_cy = di.cy;
                        impl->depth_w = static_cast<int>(di.width);
                        impl->depth_h = static_cast<int>(di.height);
                        std::printf("[Orbbec] 深度内参: fx=%.3f fy=%.3f cx=%.3f cy=%.3f (%dx%d)\n",
                                    di.fx, di.fy, di.cx, di.cy, di.width, di.height);
                    } catch (const ob::Error &e) {
                        std::printf("[Orbbec] ⚠️ 取深度内参失败：%s\n", e.getMessage());
                    }
                    try {
                        auto ci = profile->getIntrinsic();
                        impl->color_fx = ci.fx;
                        impl->color_fy = ci.fy;
                        impl->color_cx = ci.cx;
                        impl->color_cy = ci.cy;
                        std::printf("[Orbbec] 彩色内参: fx=%.3f fy=%.3f cx=%.3f cy=%.3f\n",
                                    ci.fx, ci.fy, ci.cx, ci.cy);
                    } catch (const ob::Error &e) {
                        std::printf("[Orbbec] ⚠️ 取彩色内参失败：%s\n", e.getMessage());
                    }
                    // 深度→彩色的外参：SDK 在这台设备上可能查不到（实测报
                    // "Can not find matched camera param"），失败就用默认单位阵，
                    // 反正两相机实测只差 10 毫米，先把主项（内参投影）做对。
                    try {
                        impl->T_color_to_depth = dprofile->getExtrinsicTo(profile);
                        std::printf("[Orbbec] 彩色→深度 平移: [%.2f %.2f %.2f] mm\n",
                                    impl->T_color_to_depth.trans[0],
                                    impl->T_color_to_depth.trans[1],
                                    impl->T_color_to_depth.trans[2]);
                    } catch (const ob::Error &) {
                        impl->T_color_to_depth = OBExtrinsic{};
                        std::printf("[Orbbec] 深度外参查不到，按两相机重合处理（实测差 10 mm）\n");
                    }
                    std::printf("[Orbbec] 深度流已开启：%dx%d @ %d fps\n",
                                dprofile->width(), dprofile->height(), dprofile->fps());
                }
            } catch (const ob::Error &e) {
                std::printf("[Orbbec] ⚠️ 深度流开启失败：%s（继续只用彩色）\n", e.getMessage());
            }
        }
        impl->depth_enabled = depth_ok;
        if (depth_ok) {
            // 把深度对齐到彩色：这样 (u,v) 在两个图上指同一个物理点，
            // 特征可以"看彩色、取深度"，不需要再算外参插值。
            impl->align = std::make_unique<ob::Align>(OB_STREAM_COLOR);
        }

        // 让每个 FrameSet 同时包含彩色和深度：默认的 ANY_SITUATION 会让两路分开到达，
        // 结果大部分帧只有彩色、拿不到深度（实测"深度 0x0 有效 0%"）。
        try {
            switch (cfg.aggregate_mode) {
                case 1:
                    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_COLOR_FRAME_REQUIRE);
                    std::printf("[Orbbec] 帧集聚合：COLOR_FRAME_REQUIRE（彩色到就出帧，深度可选）\n");
                    break;
                case 2:
                    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
                    std::printf("[Orbbec] 帧集聚合：ANY_SITUATION（两路各自出帧）\n");
                    break;
                default:
                    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE);
                    std::printf("[Orbbec] 帧集聚合：FULL_FRAME_REQUIRE（两路都齐才出帧）\n");
                    break;
            }
        } catch (const ob::Error &) {
            std::printf("[Orbbec] ⚠️ 设置帧集聚合模式失败，沿用默认\n");
        }
        impl->pipe->start(config);

        std::printf("[Orbbec] 彩色流 %dx%d @ %d fps, format=%d\n", profile->width(),
                    profile->height(), profile->fps(), static_cast<int>(profile->format()));
        std::fflush(stdout);

        // 失败时先把流停掉再返回：不然相机被占着，下一次重试会直接打不开。
        const auto fail = [&](const std::string& msg) -> bool {
            if (error) *error = msg;
            try {
                impl->pipe->stop();
            } catch (const ob::Error&) {
            }
            return false;
        };

        // 锁定相机控制。必须在 start 之后设：UVC 控制项要等流起来才生效。
        // 【2026-10-20 改】原来失败只打一行"继续跑，但曝光不受控"就往下走，
        // 于是相机跑自动曝光、曝光时间逐帧变、cam-IMU 时移跟着漂 —— 这种数据
        // 对标定和 VIO 都是负资产。现在改成失败即拒绝启动（fail-fast）。
        auto dev = impl->pipe->getDevice();
        if (!dev) {
            return fail("拿不到 Orbbec device，无法锁定曝光/增益 —— 拒绝在曝光不受控的情况下采集");
        }

        auto& snap = impl->snapshot;
        try {
            snap.auto_exposure = dev->getBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL) ? 1 : 0;
            snap.exposure = dev->getIntProperty(OB_PROP_COLOR_EXPOSURE_INT);
            snap.gain = dev->getIntProperty(OB_PROP_COLOR_GAIN_INT);
            snap.auto_white_balance =
                dev->getBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL) ? 1 : 0;
            snap.white_balance = dev->getIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT);
            snap.valid = true;
        } catch (const ob::Error& e) {
            std::printf("[Orbbec] ⚠️ 读不到原始控制值（退出时将无法恢复）: %s\n", e.getMessage());
        }

        // 可选控件：支持就写死，不支持就跳过并说明 —— 不同固件支持的项不一样。
        const auto try_int = [&](OBPropertyID id, int value, const char* name) {
            try {
                dev->setIntProperty(id, value);
                const int actual = dev->getIntProperty(id);
                if (actual == value) {
                    std::printf("[Orbbec]   %s=%d\n", name, actual);
                } else {
                    std::printf("[Orbbec]   %s 请求 %d，实际 %d（固件夹了值，不致命）\n", name,
                                value, actual);
                }
            } catch (const ob::Error& e) {
                std::printf("[Orbbec]   %s 不支持，跳过: %s\n", name, e.getMessage());
            }
        };

        try {
            dev->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, cfg.auto_exposure);
            if (!cfg.auto_exposure) {
                dev->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, cfg.exposure);
            }
            dev->setIntProperty(OB_PROP_COLOR_GAIN_INT, cfg.gain);
            dev->setBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, false);
            dev->setIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, cfg.white_balance);

            try_int(OB_PROP_COLOR_BRIGHTNESS_INT, cfg.brightness, "亮度");
            try_int(OB_PROP_COLOR_CONTRAST_INT, cfg.contrast, "对比度");
            try_int(OB_PROP_COLOR_SHARPNESS_INT, cfg.sharpness, "锐化");
            try_int(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, cfg.backlight_compensation,
                    "逆光补偿");
            // gamma 不猜数值：读出来再显式写回同一个值，把它钉死，
            // 防止别的进程（或固件自己）在中途改掉却又看不出来。
            try {
                const int gamma_now = dev->getIntProperty(OB_PROP_COLOR_GAMMA_INT);
                try_int(OB_PROP_COLOR_GAMMA_INT, gamma_now, "gamma");
            } catch (const ob::Error& e) {
                std::printf("[Orbbec]   gamma 读不到，跳过: %s\n", e.getMessage());
            }

            // 读回确认（有些固件会静默夹到别的值，写进去不等于生效）
            const int auto_exp = dev->getBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL) ? 1 : 0;
            const int exp = dev->getIntProperty(OB_PROP_COLOR_EXPOSURE_INT);
            const int gain = dev->getIntProperty(OB_PROP_COLOR_GAIN_INT);
            const int wb = dev->getIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT);
            std::printf("[Orbbec] 相机控制: 自动曝光=%d 曝光=%d (%.1f ms) 增益=%d 白平衡=%d\n",
                        auto_exp, exp, exp * 0.1, gain, wb);

            if (cfg.auto_exposure) {
                if (auto_exp == 0) {
                    return fail("想要自动曝光，但相机读回是手动模式 —— 亮度不可复现，拒绝采集");
                }
            } else {
                if (auto_exp != 0) {
                    return fail("自动曝光没关掉（读回仍为开启）—— 曝光时间会逐帧变，拒绝采集");
                }
                if (exp != cfg.exposure) {
                    return fail("曝光没写进去（想要 " + std::to_string(cfg.exposure) +
                                "，实际 " + std::to_string(exp) +
                                "）—— cam-IMU 时移会跟着漂，拒绝采集");
                }
            }
        } catch (const ob::Error& e) {
            return fail(std::string("相机控制设置失败，拒绝在曝光不受控的情况下采集: ") +
                        e.getMessage());
        }

        // 【2026-10-20 加】进场测光 —— 这一段是"换场地不再失效"的关键。
        // 之前曝光/增益是写死的，只在标定那天成立；Python 采集端已经验证过：
        // 同一间教室，10ms/gain30 → 灰度 20（发散），18ms/gain100 → 灰度 50（可用）。
        if (cfg.auto_tune_exposure && !cfg.auto_exposure) {
            const auto grab_mean = [&](int settle, int samples) -> double {
                double sum = 0.0;
                int n = 0;
                for (int i = 0; i < settle + samples; ++i) {
                    std::shared_ptr<ob::FrameSet> set;
                    try {
                        set = impl->pipe->waitForFrames(200);
                    } catch (const ob::Error&) {
                        continue;
                    }
                    if (!set) continue;
                    auto color = set->colorFrame();
                    if (!color) continue;
                    cv::Mat gray = frameToGray(color);
                    if (gray.empty()) continue;
                    if (i < settle) continue;   // 丢掉改动生效前的过渡帧
                    sum += cv::mean(gray)[0];
                    ++n;
                }
                return n > 0 ? sum / n : -1.0;
            };

            OBIntPropertyRange er{};
            OBIntPropertyRange gr{};
            bool have_range = false;
            try {
                er = dev->getIntPropertyRange(OB_PROP_COLOR_EXPOSURE_INT);
                gr = dev->getIntPropertyRange(OB_PROP_COLOR_GAIN_INT);
                have_range = true;
            } catch (const ob::Error& e) {
                std::printf("[Orbbec] ⚠️ 问不到控件范围（%s），跳过自动测光\n", e.getMessage());
            }

            if (have_range) {
                const int frame_cap = static_cast<int>(0.9 * 1e6 / (cfg.fps * 100.0));
                // 模糊预算反推曝光上限：blur = f * ω * t_exp  ⇒  t_exp = blur / (f * ω)
                const double omega = cfg.tune_expected_rate_dps * M_PI / 180.0;
                const double blur_ms = (omega > 1e-6 && (impl->color_fx > 1.0 ? impl->color_fx : cfg.tune_focal_px) > 1.0)
                                           ? 1000.0 * cfg.tune_blur_budget_px /
                                                 ((impl->color_fx > 1.0 ? impl->color_fx : cfg.tune_focal_px) * omega)
                                           : cfg.tune_max_exposure_ms;
                const double eff_ms = std::min(blur_ms, cfg.tune_max_exposure_ms);
                const int blur_cap = static_cast<int>(eff_ms * 10.0);
                //===================================================================
                // 【2026-10-21】策略反转（重要）
                //
                // 原逻辑：曝光上限由"预期角速度"反推（blur_cap），光不够就**拒绝启动**。
                // 问题是：比赛现场可能就是暗的，拒绝启动等于直接不能用。
                //
                // 新逻辑：**先按现场亮度把曝光拉够**（上限只受帧周期和硬上限约束），
                //        再用实际的曝光反算"这曝光允许多快的转动"并打印出来。
                //        亮的时候自动退回短曝光+高转速；暗的时候自动变成长曝光+低转速，
                //        永远不会因为"暗"而拒绝启动。
                //===================================================================
                const int light_cap = static_cast<int>(
                    std::min(std::min(er.max, frame_cap), static_cast<int>(cfg.tune_max_exposure_ms * 10.0)));
                // 两级曝光：先尊重模糊预算（blur_cap），不够亮才放到现场光的上限（light_cap）。
                // 因为暗场地必须能跑，但亮场地也不该白牺牲转速。
                const int exp_cap = std::max(er.min + std::max(1, er.step), blur_cap);
                const int exp_cap_dark = std::max(er.min + std::max(1, er.step), light_cap);
                const int gain_cap = std::max(gr.min, std::min(gr.max, cfg.tune_max_gain));
                impl->exp_min = er.min; impl->exp_max = er.max; impl->exp_step = std::max(1, er.step);
                impl->gain_min = gr.min; impl->gain_max = gr.max;
                impl->blur_cap = blur_cap; impl->light_cap = light_cap; impl->gain_cap = gain_cap;
                std::printf("[Orbbec] 自动测光：目标灰度 %d｜先试 %.2f ms（模糊预算 %.1f px @ %.0f°/s）"
                            "，不够亮最多放到 %.2f ms｜增益上限 %d｜曝光范围 %d..%d\n",
                            cfg.tune_target_mean, exp_cap * 0.1, cfg.tune_blur_budget_px,
                            cfg.tune_expected_rate_dps, exp_cap_dark * 0.1, gain_cap,
                            er.min, er.max);

                // 给定曝光，跑一遍增益阶梯，返回 (最佳增益, 最佳灰度)
                auto try_exposure = [&](int exp_units, int &out_gain) -> double {
                    int bg = gr.min;
                    double bm = -1.0;
                    for (const double frac : {0.0, 0.03, 0.07, 0.12, 0.2, 0.3, 0.45, 0.65, 1.0}) {
                        const int want = gr.min + static_cast<int>(frac * (gain_cap - gr.min) + 0.5);
                        try {
                            dev->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, exp_units);
                            dev->setIntProperty(OB_PROP_COLOR_GAIN_INT, want);
                        } catch (const ob::Error &) {
                            break;
                        }
                        const double mean = grab_mean(4, 5);
                        if (mean < 0) continue;
                        std::printf("[Orbbec]   曝光 %.1f ms 增益 %d → 灰度 %.1f\n",
                                    exp_units * 0.1, want, mean);
                        if (mean > bm) { bm = mean; bg = want; }
                        if (mean >= cfg.tune_target_mean) break;
                    }
                    out_gain = bg;
                    return bm;
                };

                int best_gain = gr.min;
                int used_exp = exp_cap;
                double best_mean = try_exposure(exp_cap, best_gain);
                if (best_mean >= 0 && best_mean < cfg.tune_min_mean && exp_cap_dark > exp_cap) {
                    std::printf("[Orbbec] 灰度只有 %.1f（门槛 %d），把曝光放到 %.1f ms 再试"
                                "（会牺牲转速）\n", best_mean, cfg.tune_min_mean, exp_cap_dark * 0.1);
                    int g2 = gr.min;
                    const double m2 = try_exposure(exp_cap_dark, g2);
                    if (m2 > best_mean) { best_mean = m2; best_gain = g2; used_exp = exp_cap_dark; }
                }
                const int exp_used = used_exp;
                try {
                    dev->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, exp_used);
                    dev->setIntProperty(OB_PROP_COLOR_GAIN_INT, best_gain);
                } catch (const ob::Error &) {
                }
                double final_mean = grab_mean(4, 5);
                if (final_mean < 0) final_mean = best_mean;
                std::printf("[Orbbec]   锁定：曝光 %.1f ms 增益 %d → 灰度 %.1f\n",
                            exp_used * 0.1, best_gain, final_mean);
                // 存进 Impl 的副本（start() 的 cfg 参数是 const，不能直接写）
                impl->cfg.exposure = exp_used;
                impl->cfg.gain = best_gain;
                // 实际曝光反算运动上限：blur = f * ω * t_exp  ==>  ω_max = blur / (f * t_exp)
                const double t_exp_s = exp_used * 1e-4; // 曝光单位 0.1 ms
                const double omega_max_dps =
                    (t_exp_s > 1e-6 && (impl->color_fx > 1.0 ? impl->color_fx : cfg.tune_focal_px) > 1.0)
                        ? (cfg.tune_blur_budget_px / ((impl->color_fx > 1.0 ? impl->color_fx : cfg.tune_focal_px) * t_exp_s)) * 180.0 / M_PI
                        : 0.0;
                // 【2026-10-21】运动上限**无条件打印**：它现在是结果，不是输入。
                // 亮场地会自动退回短曝光（上限更高），暗场地自动变成长曝光（上限更低），
                // 但操作员任何时候都需要知道"这个曝光允许多快转"。
                std::printf("[Orbbec] ➜ 运动上限 %.1f°/s（曝光 %.1f ms × 模糊预算 %.1f px；"
                            "超过它角点会糊）\n",
                            omega_max_dps, exp_used * 0.1, cfg.tune_blur_budget_px);
                //=====================================================================
                // 【2026-10-23】平移也有同样的模糊上限，以前只打印了转动。
                // 模糊 = f·v·t_exp / Z  ==>  v_max = blur·Z / (f·t_exp)
                // 实测：这条线比转动更早被踩破——ver1 走 0.2-0.4 m/s 全程健康，
                // ver2 走 0.7-1.0 m/s 就在第 11 秒发散。必须让操作员看见。
                //=====================================================================
                if (t_exp_s > 1e-6 && (impl->color_fx > 1.0 ? impl->color_fx : cfg.tune_focal_px) > 1.0) {
                    const double k = cfg.tune_blur_budget_px / ((impl->color_fx > 1.0 ? impl->color_fx : cfg.tune_focal_px) * t_exp_s);
                    std::printf("[Orbbec] ➜ 平移上限 ≈ %.2f×Z m/s（Z=场景深度）："
                                "1m→%.2f  2m→%.2f  3m→%.2f m/s；超过它同样会糊\n",
                                k, k * 1.0, k * 2.0, k * 3.0);
                }
                if (exp_used > exp_cap) {
                    std::printf("[Orbbec]   （比模糊预算的 %.2f ms 长，因为现场光只够到这里）\n",
                                exp_cap * 0.1);
                }
                if (final_mean >= 0 && final_mean < cfg.tune_min_mean) {
                    std::printf("[Orbbec] ⚠️ 低光降级运行：灰度只有 %.1f（门槛 %d）。"
                                "不拒绝启动 —— 比赛现场可能就是暗的。\n",
                                final_mean, cfg.tune_min_mean);
                }
                std::fflush(stdout);
                // 曝光/增益都顶到上限才勉强够亮 = 场地光不够，飞起来一定糊。
                // session29 就是这样录的：18 ms + 增益 100，灰度只有 49.5，
                // 结果 KLT 存活率 72%、三角化失败 55%、更新率 26%、尺度偏 5 倍。
                // 曝光顶到了"起作用的那条限制"（相机上限/帧周期/模糊预算三者最小）就算顶死
                const int exp_limit = std::min(std::min(er.max, frame_cap), blur_cap);
                const bool exp_pinned =
                    exp_cap >= exp_limit - std::max(1, er.step) || blur_cap <= er.min + er.step;
                const bool gain_pinned = best_gain >= gain_cap;
                // "差得远"要有明确的界线：目标是 100，差 10% 以内算达标（否则灰度 99.3
                // 这种场景会被判死，而它和 100 没有任何使用上的区别）。
                // 真正的"太暗"由上面的 tune_min_mean 负责，这一条只拦"顶死了还差一截"。
                const int pin_min_mean =
                    static_cast<int>(cfg.tune_target_mean * 0.9);
                // 曝光/增益顶死了仍然偏暗：只警告，不拒绝（同上：暗场地也必须能跑）
                if (final_mean >= 0 && exp_pinned && gain_pinned &&
                    final_mean < pin_min_mean) {
                    std::printf("[Orbbec] ⚠️ 曝光和增益都到上限（%.1f ms / 增益 %d）仍只有灰度 %.1f，"
                                "目标 %d。继续运行，但特征会更少、更不稳定。\n",
                                exp_used * 0.1, best_gain, final_mean, cfg.tune_target_mean);
                }
            }
        }
        std::fflush(stdout);
    } catch (const ob::Error& e) {
        if (error) {
            *error = std::string("Orbbec 初始化失败: ") + e.getName() + " " + e.getMessage();
        }
        return false;
    } catch (const std::exception& e) {
        if (error) *error = std::string("Orbbec 初始化异常: ") + e.what();
        return false;
    }

    impl->running.store(true);
    impl_ = std::move(impl);
    impl_->thread = std::thread([this]() {
        Impl* impl = impl_.get();
        // 运行中重测光的统计量（见上面的说明）
        double retune_sum = 0.0; int retune_n = 0; int64_t last_retune_ns = 0;
        long long retune_over_blur = 0;

        // 【2026-10-20 加】锁定控件之前采集的帧还是旧曝光，不能进估计器。
        int warmup = impl->cfg.warmup_frames;
        try {
            while (impl->running.load()) {
                auto frameset = impl->pipe->waitForFrames(100);
                if (!frameset) continue;

                auto color = frameset->colorFrame();
                if (!color) continue;

                if (warmup > 0) {
                    --warmup;
                    continue;
                }

                const int64_t host_ns = steadyNowNs();



                //=======================================================================
                // 【2026-10-21】每帧重新打开红外激光（重要！）
                // 实测设备在启动约 1 帧后会把 LASER_BOOL 自动置 0，之后深度全零。
                // 只设一次没用，必须持续置位 —— 这是"深度一直读不到"的真凶。
                // 每 5 帧写一次，避免每帧都发 USB 控制传输。
                //=======================================================================
                if (impl->depth_enabled && (impl->laser_tick++ % 5 == 0)) {
                    try {
                        impl->pipe->getDevice()->setBoolProperty(OB_PROP_LASER_BOOL, true);
                    } catch (const ob::Error &) {
                    }
                }

                // 深度图（毫米，原始深度相机坐标系）。没有深度流时是空 Mat。
                // 【2026-10-21】没有用 SDK 的 Align 滤波器 —— 它在本机（USB2 + MJPG 1080p 彩色
                // + 1280x800 深度）会直接段错误（SIGSEGV）。这里只取原始深度，
                // 之后按内参/外参把特征投影到深度图上取值（每帧 300 个点，开销可忽略）。
                cv::Mat depth_mm;
                int64_t depth_dev_us = 0;   // 【2026-10-23】深度帧自己的设备时间戳
                if (impl->depth_enabled) {
                    try {
                        std::shared_ptr<ob::DepthFrame> df = frameset->depthFrame();
                        if (df && df->data() != nullptr && df->width() > 0 && df->height() > 0) {
                            depth_dev_us = static_cast<int64_t>(df->timeStampUs());
                            const int w = static_cast<int>(df->width());
                            const int h = static_cast<int>(df->height());
                            const uint32_t bytes = df->dataSize();
                            const float scale = df->getValueScale();
                            static bool dumped = false;
                            if (!dumped) {
                                dumped = true;
                                // 统计一下非零深度：全 0 说明激光没开、或者目标超出结构光量程
                                int nz = 0, mn = 1 << 30, mx = 0;
                                std::vector<int> vals;
                                const unsigned short *p16 =
                                    static_cast<const unsigned short *>(df->data());
                                for (int r = 0; r < h; r += 4) {
                                    for (int c = 0; c < w; c += 4) {
                                        const unsigned short v = p16[r * w + c];
                                        if (v > 0 && v != 0xFFFF) {
                                            ++nz;
                                            vals.push_back(v);
                                            mn = std::min(mn, (int)v);
                                            mx = std::max(mx, (int)v);
                                        }
                                    }
                                }
                                int med = 0;
                                if (!vals.empty()) {
                                    std::nth_element(vals.begin(), vals.begin() + vals.size() / 2,
                                                     vals.end());
                                    med = vals[vals.size() / 2];
                                }
                                std::printf("[Orbbec] 原始深度帧: %dx%d scale=%.4f fmt=%d "
                                            "非零样本=%d 范围=[%d,%d]mm 中位=%dmm\n",
                                            w, h, scale, static_cast<int>(df->format()), nz,
                                            nz ? mn : 0, mx, med);
                                std::fflush(stdout);
                            }
                            const size_t need =
                                static_cast<size_t>(w) * static_cast<size_t>(h) * 2u;
                            if (bytes >= need) {
                                cv::Mat raw(h, w, CV_16UC1, df->data());
                                if (scale > 0.0f && scale != 1.0f) {
                                    raw.convertTo(depth_mm, CV_16UC1, static_cast<double>(scale));
                                } else {
                                    depth_mm = raw.clone();
                                }
                            }
                        }
                    } catch (...) {
                        // 深度处理任何异常都不能拖死采集线程
                    }
                }

                // 相机自带的帧时间戳（SDK 设备时间，微秒）。
                // 【2026-10-21】记录两个相机侧时间戳，用来量链路：
                //   sys_us    = SDK 收到帧时的**系统时间**（主机时基，可与我们的 host 比）
                //   device_us = 设备自己的时间戳（时基不同，只能比帧间增量）
                // 实测 timeStampUs() 每帧增量 4.998e8，不是微秒 —— 只能当相对量用。
                int64_t device_us = -1, sys_us = -1;
                try {
                    device_us = static_cast<int64_t>(color->timeStampUs());
                    sys_us = static_cast<int64_t>(color->systemTimeStampUs());
                } catch (...) {
                }
                cv::Mat gray = frameToGray(color);
                if (gray.empty()) continue;
                //---------------------------------------------------------------
                // 运行中重测光：滞回 + 限速（最多每 2 s 调一次）
                //   过曝（mean>170）：先砍曝光
                //   过暗（mean<45）：先加增益，增益顶了再加曝光（到现场光上限）
                // 每帧只统计，决策每 15 帧（≈0.25 s）一次
                //---------------------------------------------------------------
                if (impl->auto_retune && impl->cfg.auto_tune_exposure) {
                    const double m_now = cv::mean(gray)[0];
                    retune_sum += m_now; ++retune_n;
                    if (retune_n >= 15) {
                        const double mean_now = retune_sum / retune_n;
                        retune_sum = 0.0; retune_n = 0;
                        const int64_t now_ns2 = host_ns;
                        const bool cooled = (now_ns2 - last_retune_ns) > 2000000000LL;
                        int cur_exp = impl->cfg.exposure, cur_gain = impl->cfg.gain;
                        int new_exp = cur_exp, new_gain = cur_gain;
                        // 阈值可用环境变量压窄，用来在亮/暗场景里验证这条路径真的会触发
                        static const double hi_thr = getenv("OV_RETUNE_HI") ? atof(getenv("OV_RETUNE_HI")) : 170.0;
                        static const double lo_thr = getenv("OV_RETUNE_LO") ? atof(getenv("OV_RETUNE_LO")) : 45.0;
                        if (mean_now > hi_thr) {
                            // 过曝：优先砍曝光（也顺带减小模糊）
                            new_exp = std::max(impl->exp_min, static_cast<int>(cur_exp * 2.0 / 3.0));
                            if (new_exp == cur_exp) new_gain = std::max(impl->gain_min, cur_gain - 8);
                        } else if (mean_now < lo_thr) {
                            // 过暗：先加增益，噪声换亮度；增益顶了再放曝光
                            if (cur_gain < impl->gain_cap) new_gain = std::min(impl->gain_cap, cur_gain + 8);
                            else new_exp = std::min(impl->light_cap, static_cast<int>(cur_exp * 4.0 / 3.0));
                        }
                        if (cooled && (new_exp != cur_exp || new_gain != cur_gain)) {
                            try {
                                impl->pipe->getDevice()->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, new_exp);
                                impl->pipe->getDevice()->setIntProperty(OB_PROP_COLOR_GAIN_INT, new_gain);
                                impl->cfg.exposure = new_exp;
                                impl->cfg.gain = new_gain;
                                last_retune_ns = now_ns2;
                                const double t_exp_s2 = new_exp * 1e-4;
                                const double om2 = (t_exp_s2 > 1e-6 && impl->color_fx > 1.0)
                                    ? (impl->cfg.tune_blur_budget_px / (impl->color_fx * t_exp_s2)) * 180.0 / M_PI
                                    : 0.0;
                                if (new_exp > impl->blur_cap) ++retune_over_blur;
                                std::printf("[Orbbec] 重测光：灰度 %.0f → 曝光 %.1f ms 增益 %d"
                                            "（运动上限 %.1f°/s%s）\n",
                                            mean_now, new_exp * 0.1, new_gain, om2,
                                            new_exp > impl->blur_cap ? "，已超模糊预算" : "");
                                std::fflush(stdout);
                            } catch (const ob::Error &) {}
                        }
                    }
                }

                if (impl->callback) impl->callback(gray, depth_mm, host_ns, sys_us, depth_dev_us);
            }
        } catch (const ob::Error& e) {
            std::fprintf(stderr, "[Orbbec] 采集异常: %s %s\n", e.getName(), e.getMessage());
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[Orbbec] 采集异常: %s\n", e.what());
        }
        impl->running.store(false);
    });

    return true;
}

void OrbbecCamera::stop() {
    if (!impl_) return;
    impl_->running.store(false);
    if (impl_->thread.joinable()) impl_->thread.join();

    // 退出恢复：把 start() 之前的值写回去，不给下一个进程留下"被锁定"的状态。
    auto& snap = impl_->snapshot;
    if (snap.valid && impl_->pipe) {
        try {
            auto dev = impl_->pipe->getDevice();
            if (dev) {
                // 先恢复手动量、再放开自动开关（和 vendor/capture_calibration.py 顺序一致）
                dev->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, snap.exposure);
                dev->setIntProperty(OB_PROP_COLOR_GAIN_INT, snap.gain);
                dev->setIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, snap.white_balance);
                dev->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, snap.auto_exposure != 0);
                dev->setBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL,
                                     snap.auto_white_balance != 0);
                std::printf("[Orbbec] 已恢复采集前的相机控制（自动曝光=%d 曝光=%d 增益=%d 白平衡=%d）\n",
                            snap.auto_exposure, snap.exposure, snap.gain, snap.white_balance);
                std::fflush(stdout);
            }
        } catch (const ob::Error& e) {
            std::fprintf(stderr, "[Orbbec] ⚠️ 退出时恢复相机控制失败（下一个进程可能读到残留状态）: %s\n",
                         e.getMessage());
        }
    }

    try {
        if (impl_->pipe) impl_->pipe->stop();
    } catch (const ob::Error&) {
    }
    impl_.reset();
}

OrbbecCamera::DepthInfo OrbbecCamera::depthInfo() const {
    DepthInfo d;
    if (!impl_) return d;
    d.fx = impl_->depth_fx;
    d.fy = impl_->depth_fy;
    d.cx = impl_->depth_cx;
    d.cy = impl_->depth_cy;
    d.width = impl_->depth_w;
    d.height = impl_->depth_h;
    //=======================================================================
    // 【2026-10-22 修正】外参方向与平移的对应关系
    //
    // SDK 的 getExtrinsicTo(target) 给的是 **深度 → 彩色**： p_c = R_d2c·p_d + t_d2c
    // 我们需要的是 **彩色 → 深度**：
    //     R_c2d = R_d2cᵀ
    //     t_c2d = −R_d2cᵀ · t_d2c        ← 之前这里漏了，直接用了 t_d2c，是错的
    // 平移量级 10 mm（1 米处约 14 像素），所以方向搞反不是小事。
    //=======================================================================
    const auto &R_d2c = impl_->T_color_to_depth.rot;
    const double td[3] = {impl_->T_color_to_depth.trans[0], impl_->T_color_to_depth.trans[1],
                          impl_->T_color_to_depth.trans[2]};
    double Rc[3][3];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) Rc[r][c] = R_d2c[c * 3 + r]; // 转置 = 逆
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) d.R_c2d[r * 3 + c] = Rc[r][c];
    // t_c2d = −R_c2d · t_d2c
    for (int r = 0; r < 3; ++r)
        d.t_mm[r] = -(Rc[r][0] * td[0] + Rc[r][1] * td[1] + Rc[r][2] * td[2]);
    return d;
}

bool OrbbecCamera::running() const { return impl_ && impl_->running.load(); }

}  // namespace pure_uav
