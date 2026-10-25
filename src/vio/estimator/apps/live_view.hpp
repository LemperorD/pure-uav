// 实时查看器：把「相机图像 + 跟踪点 + 状态面板 + 轨迹」画到一个窗口里。
//
// 设计取舍：
//   - 只用 OpenCV highgui，不引入 Qt / rviz。整机没有 ROS，这个程序要能单文件跑起来。
//   - 只能在 VIO 线程调用（highgui 的 imshow/waitKey 必须同一线程），每帧一次。
//   - 显示的东西一律只读：不参与估计，也不改任何滤波状态。
//   - 窗口里的文字全部用 ASCII：Hershey 字体没有中文字形，写中文会变成一串问号。
//
// 窗口布局（1400 x 710）：
//   +--------------------------+--------------+
//   |  相机图像 960x540         |  状态面板     |
//   |  （跟踪点叠加）           |  440x540     |
//   +--------------------------+--------------+
//   |  轨迹俯视图 1400x170                     |
//   +------------------------------------------+

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "pure_uav/vio_estimator.hpp"
#include "pure_uav/vio_health.hpp"
#include "pure_uav/vision_pose.hpp"

namespace pure_uav {

class LiveView {
public:
    static constexpr int kImgW = 960;
    static constexpr int kImgH = 540;
    static constexpr int kPanelW = 440;
    static constexpr int kTrajH = 170;
    static constexpr int kW = kImgW + kPanelW;  // 1400
    static constexpr int kH = kImgH + kTrajH;   // 710

    struct Status {
        bool initialized = false;
        VioHealthState health = VioHealthState::UNKNOWN;
        bool publishable = false;
        int features = 0;
        double since_update_s = 0.0;
        int recoveries = 0;
        double update_rate = 0.0;  // MSCKF 更新率（近 2 s 滑窗）
        long long images = 0, poses = 0, sent = 0, blocked = 0;
        double fps = 0.0;
        double imu_hz = 0.0;
        // 【2026-10-21】运动有效性：||a||-g 的窗口中位，和 MSCKF 三角化成功率。
        // 单目 VIO 靠平移拿视差；原地转时 lin 只有 0.015~0.07、tri 恒为 0。
        float lin_acc = -1.f;
        float tri_rate = -1.f;

        bool has_pose = false;
        float x = 0.f, y = 0.f, z = 0.f;
        float vx = 0.f, vy = 0.f, vz = 0.f;
        float qw = 1.f, qx = 0.f, qy = 0.f, qz = 0.f;

        bool has_raw = false;
        Eigen::Vector3d bias_a = Eigen::Vector3d::Zero();
        Eigen::Vector3d bias_g = Eigen::Vector3d::Zero();

        // 倾角自检：滤波器认为的重力方向 vs 加速度计实测的重力方向。
        // 只用准静止时刻算；valid=false 表示这一刻在动，数值不可信。
        bool tilt_valid = false;
        double tilt_deg = 0.0;

        std::string note;  // 附加信息（相机亮度等），ASCII
        bool recording = false;
    };

    LiveView() = default;
    ~LiveView() { close(); }

    LiveView(const LiveView&) = delete;
    LiveView& operator=(const LiveView&) = delete;

    bool open(const std::string& title = "VIO real-time") {
        title_ = title;
        try {
            cv::namedWindow(title_, cv::WINDOW_AUTOSIZE);
        } catch (const cv::Exception& e) {
            std::fprintf(stderr, "[view] 打不开窗口（%s）—— 检查 DISPLAY\n", e.what());
            return false;
        }
        opened_ = true;
        return true;
    }

    void close() {
        if (opened_) {
            try {
                cv::destroyWindow(title_);
            } catch (const cv::Exception&) {
            }
            opened_ = false;
        }
    }

    // 处理窗口事件。返回 true 表示操作者要求退出（按了 q / Esc，或关掉了窗口）。
    bool poll() {
        if (!opened_) return false;
        int key = -1;
        try {
            key = cv::waitKey(1) & 0xFF;
        } catch (const cv::Exception&) {
            return false;
        }
        if (key == 'q' || key == 27) return true;
        // 不查 WND_PROP_VISIBLE：实测这个 OpenCV 构建里窗口已显示时它也返回 <0，
        // 会把"窗口正常"误判成"被关闭"。退出统一用 q / Esc（或 Ctrl-C）。
        return false;
    }

    void setShotPrefix(const std::string& prefix) { shot_prefix_ = prefix; }

    // 每帧画一次。gray 可以是 1920x1080，也可以任意尺寸（会自动缩放到 960x540）。
    void render(const cv::Mat& gray, const std::vector<VioEstimator::TrackedPoint>& tracks,
                const Status& st) {
        if (!opened_) return;
        canvas_.create(kH, kW, CV_8UC3);
        canvas_.setTo(cv::Scalar(28, 28, 30));

        drawImage(gray, tracks, st);
        drawPanel(st);
        drawTrajectory(st);

        const bool want_shot =
            !shot_prefix_.empty() &&
            (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_shot_).count() >
             2.0);
        if (want_shot) {
            last_shot_ = std::chrono::steady_clock::now();
            char path[512];
            std::snprintf(path, sizeof(path), "%s_%03d.png", shot_prefix_.c_str(), shot_idx_++);
            try {
                cv::imwrite(path, canvas_);
            } catch (const cv::Exception&) {
            }
        }

        try {
            cv::imshow(title_, canvas_);
            if (!shown_) {
                shown_ = true;
                first_show_ = std::chrono::steady_clock::now();
            }
        } catch (const cv::Exception&) {
        }
    }

    size_t trajectorySize() const { return traj_.size(); }

private:
    static cv::Scalar healthColor(VioHealthState s) {
        switch (s) {
            case VioHealthState::OK: return cv::Scalar(90, 220, 90);
            case VioHealthState::DEGRADED: return cv::Scalar(60, 200, 240);
            case VioHealthState::LOST: return cv::Scalar(70, 70, 255);
            default: return cv::Scalar(170, 170, 170);
        }
    }

    void drawImage(const cv::Mat& gray, const std::vector<VioEstimator::TrackedPoint>& tracks,
                   const Status& st) {
        cv::Mat roi = canvas_(cv::Rect(0, 0, kImgW, kImgH));
        if (!gray.empty()) {
            cv::Mat small;
            const double sx = static_cast<double>(kImgW) / gray.cols;
            const double sy = static_cast<double>(kImgH) / gray.rows;
            cv::resize(gray, small, cv::Size(kImgW, kImgH), 0, 0, cv::INTER_AREA);
            if (small.channels() == 1) {
                cv::cvtColor(small, small, cv::COLOR_GRAY2BGR);
            }
            small.copyTo(roi);

            // 跟踪点：颜色 = 这个点已经连续跟了多少帧（越老越"红"）
            for (const auto& t : tracks) {
                const int u = static_cast<int>(t.u * sx);
                const int v = static_cast<int>(t.v * sy);
                if (u < 0 || v < 0 || u >= kImgW || v >= kImgH) continue;
                cv::Scalar c(90, 235, 90);  // 新点：绿
                if (t.length >= 10) c = cv::Scalar(60, 150, 255);       // 老点：橙红
                else if (t.length >= 3) c = cv::Scalar(60, 230, 245);   // 中点：黄
                cv::circle(roi, cv::Point(u, v), 2, c, -1, cv::LINE_AA);
            }
        } else {
            roi.setTo(cv::Scalar(10, 10, 10));
            putText(roi, "no image", cv::Point(20, kImgH / 2), 0.9, cv::Scalar(200, 200, 200));
        }

        // 左上角：状态大字 + 未初始化提示
        const cv::Scalar hc = healthColor(st.health);
        cv::rectangle(roi, cv::Point(0, 0), cv::Point(kImgW, 34), cv::Scalar(0, 0, 0), -1);
        char line[256];
        std::snprintf(line, sizeof(line), "%s   feat=%d   vel=%.2f m/s",
                      st.initialized ? toString(st.health) : "NOT INITIALIZED", st.features,
                      std::sqrt(st.vx * st.vx + st.vy * st.vy + st.vz * st.vz));
        putText(roi, line, cv::Point(10, 24), 0.62, hc);
        if (!st.publishable) {
            putText(roi, "NOT PUBLISHING", cv::Point(10, 70), 0.75, cv::Scalar(70, 70, 255));
        } else {
            putText(roi, "publishing -> FCU", cv::Point(10, 70), 0.6, cv::Scalar(90, 220, 90));
        }
        if (st.recording) {
            cv::circle(roi, cv::Point(kImgW - 24, 24), 8, cv::Scalar(40, 40, 230), -1);
        }
    }

    void drawPanel(const Status& st) {
        const int x0 = kImgW;
        cv::Mat roi = canvas_(cv::Rect(x0, 0, kPanelW, kImgH));
        roi.setTo(cv::Scalar(40, 40, 44));
        cv::line(canvas_, cv::Point(x0, 0), cv::Point(x0, kImgH), cv::Scalar(90, 90, 90), 1);

        int yy = 26;
        const int dx = x0 + 14;
        char buf[256];
        auto line = [&](const cv::Scalar& c, const char* fmt, auto... args) {
            std::snprintf(buf, sizeof(buf), fmt, args...);
            putText(canvas_, buf, cv::Point(dx, yy), 0.48, c);
            yy += 22;
        };
        auto section = [&](const char* t) {
            yy += 6;
            cv::line(canvas_, cv::Point(dx, yy - 14), cv::Point(kW - 14, yy - 14),
                     cv::Scalar(80, 80, 84), 1);
            putText(canvas_, t, cv::Point(dx, yy), 0.5, cv::Scalar(150, 150, 155));
            yy += 22;
        };

        const cv::Scalar white(225, 225, 225);
        const cv::Scalar dim(170, 170, 175);
        const cv::Scalar hc = healthColor(st.health);

        putText(canvas_, "VIO REAL-TIME", cv::Point(dx, yy), 0.62, cv::Scalar(120, 200, 255));
        yy += 26;
        line(hc, "health   %s", toString(st.health));
        line(st.publishable ? cv::Scalar(90, 220, 90) : cv::Scalar(70, 70, 255), "publish  %s",
             st.publishable ? "YES" : "NO");
        line(white, "feat     %d", st.features);
        line(white, "msckf upd %.1f /s", st.update_rate);
        line(dim, "since upd %.2f s", st.since_update_s);
        line(dim, "reinit   %d", st.recoveries);

        section("camera / imu");
        line(dim, "cam %.1f fps", st.fps);
        line(dim, "imu %.0f Hz", st.imu_hz);
        if (st.lin_acc >= 0.f) {
            // 判据（实测）：lin<0.10 基本原地转 -> tri 必为 0；lin>0.12 且 tri>0.03 才算有效测试
            cv::Scalar mc(70, 70, 255);
            if (st.lin_acc > 0.12f && st.tri_rate > 0.03f) mc = cv::Scalar(90, 220, 90);
            else if (st.lin_acc > 0.12f) mc = cv::Scalar(60, 200, 240);
            line(mc, "motion %.2f  tri %.3f", st.lin_acc, st.tri_rate);
            if (st.tri_rate <= 0.0005f)
                line(cv::Scalar(70, 70, 255), "  (no depth: pure rotation)");
        }
        if (!st.note.empty()) line(dim, "%s", st.note.c_str());

        section("state (LOCAL_FRD)");
        if (st.has_pose) {
            line(white, "pos  %7.2f %7.2f %7.2f", st.x, st.y, st.z);
            line(white, "vel  %7.2f %7.2f %7.2f", st.vx, st.vy, st.vz);
            const double yaw = std::atan2(2.0 * (st.qw * st.qz + st.qx * st.qy),
                                          1.0 - 2.0 * (st.qy * st.qy + st.qz * st.qz));
            line(white, "yaw  %7.1f deg   |p|=%.2f m", yaw * 180.0 / M_PI,
                 std::sqrt(static_cast<double>(st.x) * st.x + static_cast<double>(st.y) * st.y +
                           static_cast<double>(st.z) * st.z));
        } else {
            line(dim, "(not initialized)");
        }

        section("tilt check (gravity)");
        if (st.tilt_valid) {
            // 1° 已经是危险信号：g·sin(1°)=0.17 m/s²，和走路时的平均加速度同量级
            cv::Scalar tc(90, 220, 90);
            if (st.tilt_deg > 3.0) tc = cv::Scalar(70, 70, 255);
            else if (st.tilt_deg > 1.0) tc = cv::Scalar(60, 200, 240);
            line(tc, "err  %5.2f deg", st.tilt_deg);
            char leak[96];
            std::snprintf(leak, sizeof(leak), "leak %.2f m/s2 (g*sin)", 9.81 * std::sin(st.tilt_deg * M_PI / 180.0));
            line(dim, "%s", leak);
        } else {
            line(dim, "err    --  (moving)");
        }

        section("imu bias");
        if (st.has_raw) {
            line(white, "acc  %6.3f %6.3f %6.3f", st.bias_a.x(), st.bias_a.y(), st.bias_a.z());
            line(dim, "     |acc| = %.3f m/s2", st.bias_a.norm());
            line(white, "gyr  %6.2f %6.2f %6.2f", st.bias_g.x() * 180 / M_PI,
                 st.bias_g.y() * 180 / M_PI, st.bias_g.z() * 180 / M_PI);
        } else {
            line(dim, "(waiting)");
        }

        section("counters");
        line(dim, "img  %lld", st.images);
        line(dim, "pose %lld", st.poses);
        line(dim, "sent %lld   blocked %lld", st.sent, st.blocked);
        line(cv::Scalar(130, 130, 140), "q/Esc = quit");
    }

    void drawTrajectory(const Status& st) {
        const cv::Rect box(0, kImgH, kW, kTrajH);
        cv::Mat roi = canvas_(box);
        roi.setTo(cv::Scalar(34, 34, 38));
        cv::line(canvas_, cv::Point(0, kImgH), cv::Point(kW, kImgH), cv::Scalar(90, 90, 90), 1);

        if (st.has_pose) {
            const cv::Point2f p(st.x, st.y);
            if (traj_.empty() ||
                std::hypot(traj_.back().x - p.x, traj_.back().y - p.y) > 0.005f) {
                traj_.push_back(p);
                if (traj_.size() > 20000) traj_.pop_front();
            }
        }

        float mx = 1.5f, my = 1.5f;
        for (const auto& p : traj_) {
            mx = std::max(mx, std::abs(p.x));
            my = std::max(my, std::abs(p.y));
        }
        const float span_m = std::max(mx, my) * 1.15f;
        // 【2026-10-23 修 BUG】以前这里用的是**画布绝对坐标**（y 中心 = kImgH + kTrajH/2），
        // 但所有绘制都发生在这个 ROI（局部原点在 (0, kImgH)），于是网格和轨迹全被画到
        // ROI 之外 → 底部轨迹带永远是空的（只显示标题文字）。改成 ROI 局部坐标。
        const cv::Point2f c(kW * 0.5f, kTrajH * 0.5f);
        const float scale = std::min((kW * 0.5f - 30.f) / span_m, (kTrajH * 0.5f - 14.f) / span_m);
        auto toPx = [&](float x, float y) {
            return cv::Point(static_cast<int>(c.x + x * scale), static_cast<int>(c.y - y * scale));
        };

        // 1 m 网格
        const cv::Scalar grid(52, 52, 56);
        for (int g = 1; g * 1.0f <= span_m; ++g) {
            const float gm = static_cast<float>(g);
            cv::line(roi, toPx(-span_m, gm), toPx(span_m, gm), grid, 1);
            cv::line(roi, toPx(-span_m, -gm), toPx(span_m, -gm), grid, 1);
            cv::line(roi, toPx(gm, -span_m), toPx(gm, span_m), grid, 1);
            cv::line(roi, toPx(-gm, -span_m), toPx(-gm, span_m), grid, 1);
        }
        // 坐标轴：+x 向前（右），+y 向右（上）
        cv::line(roi, toPx(0, -span_m), toPx(0, span_m), cv::Scalar(70, 70, 78), 1);
        cv::line(roi, toPx(-span_m, 0), toPx(span_m, 0), cv::Scalar(70, 70, 78), 1);

        for (size_t i = 1; i < traj_.size(); ++i) {
            cv::line(roi, toPx(traj_[i - 1].x, traj_[i - 1].y), toPx(traj_[i].x, traj_[i].y),
                     cv::Scalar(90, 200, 255), 1, cv::LINE_AA);
        }
        if (!traj_.empty()) {
            const auto& last = traj_.back();
            // 朝向箭头
            const double yaw = std::atan2(2.0 * (st.qw * st.qz + st.qx * st.qy),
                                          1.0 - 2.0 * (st.qy * st.qy + st.qz * st.qz));
            const cv::Point tip = toPx(last.x + static_cast<float>(0.35 * std::cos(yaw)),
                                       last.y + static_cast<float>(0.35 * std::sin(yaw)));
            cv::arrowedLine(roi, toPx(last.x, last.y), tip, cv::Scalar(60, 80, 255), 2, cv::LINE_AA,
                            0, 0.35);
            cv::circle(roi, toPx(last.x, last.y), 3, cv::Scalar(60, 80, 255), -1, cv::LINE_AA);
        }

        char buf[160];
        std::snprintf(buf, sizeof(buf), "trajectory (top view)  x forward / y right  grid=1 m  span=%.1f m  pts=%zu",
                      static_cast<double>(span_m) * 2.0, traj_.size());
        putText(roi, buf, cv::Point(12, 20), 0.48, cv::Scalar(160, 160, 165));
    }

    void putText(const cv::Mat& img, const std::string& text, const cv::Point& org, double scale,
                 const cv::Scalar& color) {
        cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
    }

    std::string title_ = "VIO real-time";
    bool opened_ = false;
    bool shown_ = false;
    std::chrono::steady_clock::time_point first_show_{};
    cv::Mat canvas_;
    std::deque<cv::Point2f> traj_;
    std::string shot_prefix_;
    int shot_idx_ = 0;
    std::chrono::steady_clock::time_point last_shot_{};
};

}  // namespace pure_uav
