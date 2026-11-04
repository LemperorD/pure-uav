// Orbbec 彩色采集实现；回调在采集线程执行，不得阻塞。
#include "pure_uav/detection/camera_source.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <thread>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

namespace pure_uav {
namespace detection {
namespace {

uint64_t unixNowUs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(system_clock::now().time_since_epoch()).count());
}

int formatCode(OBFormat f) { return static_cast<int>(f); }

std::string profileListString(const std::shared_ptr<ob::StreamProfileList>& list) {
    std::ostringstream os;
    const uint32_t n = list ? list->count() : 0;
    for (uint32_t i = 0; i < n; ++i) {
        std::shared_ptr<ob::VideoStreamProfile> vp;
        try {
            vp = list->getProfile(i)->as<ob::VideoStreamProfile>();
        } catch (const std::exception&) {
            continue;
        }
        if (!vp) continue;
        os << " " << vp->width() << "x" << vp->height() << "@" << vp->fps() << "f"
           << formatCode(vp->format());
    }
    return os.str();
}

// 统一把彩色帧转成 BGR CV_8UC3（MJPG → imdecode；RGB/YUYV → cvtColor）。
cv::Mat colorFrameToBgr(const std::shared_ptr<ob::ColorFrame>& frame) {
    const int w = static_cast<int>(frame->width());
    const int h = static_cast<int>(frame->height());
    const OBFormat format = frame->format();

    if (format == OB_FORMAT_MJPG) {
        cv::Mat raw(1, static_cast<int>(frame->dataSize()), CV_8UC1,
                    static_cast<uint8_t*>(frame->data()));
        return cv::imdecode(raw, cv::IMREAD_COLOR);
    }
    if (format == OB_FORMAT_RGB) {
        cv::Mat rgb(h, w, CV_8UC3, static_cast<uint8_t*>(frame->data()));
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
    if (format == OB_FORMAT_YUYV) {
        cv::Mat yuyv(h, w, CV_8UC2, static_cast<uint8_t*>(frame->data()));
        cv::Mat bgr;
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
        return bgr;
    }
    return cv::Mat();
}

}  // namespace

struct CameraSource::Impl {
    CameraConfig cfg;
    FrameCallback callback;
    std::unique_ptr<ob::Pipeline> pipe;
    std::thread thread;
    std::atomic<bool> running{false};
    std::atomic<uint64_t> seq{0};
    uint64_t last_host_us = 0;
    std::string profile;
    OBFormat format = OB_FORMAT_ANY;

    void captureLoop() {
        while (running.load(std::memory_order_relaxed)) {
            std::shared_ptr<ob::FrameSet> frameset;
            try {
                frameset = pipe->waitForFrames(100);
            } catch (const ob::Error&) {
                break;
            } catch (const std::exception&) {
                break;
            }
            if (!frameset) continue;

            auto color = frameset->colorFrame();
            if (!color) continue;

            // host_us 为到达主机的 Unix 微秒；强制单调递增，避免系统时钟回调造成倒退。
            uint64_t host_us = unixNowUs();
            if (host_us <= last_host_us) host_us = last_host_us + 1;
            last_host_us = host_us;
            const uint64_t device_us = color->timeStampUs();

            cv::Mat bgr = colorFrameToBgr(color);
            if (bgr.empty() || bgr.type() != CV_8UC3) continue;

            CameraFrame frame;
            frame.bgr = std::move(bgr);
            frame.host_us = host_us;
            frame.device_us = device_us;
            frame.seq = seq.fetch_add(1, std::memory_order_relaxed) + 1;

            if (callback) callback(frame);
        }
    }
};

CameraSource::CameraSource() : impl_(std::make_unique<Impl>()) {}

CameraSource::~CameraSource() { stop(); }

bool CameraSource::start(const CameraConfig& cfg, FrameCallback cb, std::string* error) {
    if (impl_->running.load()) return true;

    impl_->cfg = cfg;
    impl_->callback = std::move(cb);
    impl_->seq.store(0);
    impl_->last_host_us = 0;

    // SDK 日志默认压到 ERROR，避免污染 CLI 的 JSON 输出。
    ob::Context::setLoggerSeverity(cfg.verbose ? OB_LOG_SEVERITY_INFO : OB_LOG_SEVERITY_ERROR);

    try {
        impl_->pipe = std::make_unique<ob::Pipeline>();

        auto profiles = impl_->pipe->getStreamProfileList(OB_SENSOR_COLOR);
        std::shared_ptr<ob::VideoStreamProfile> profile;
        OBFormat chosen = OB_FORMAT_ANY;

        // 编码优先级：MJPG → RGB → YUYV
        for (const OBFormat fmt : {OB_FORMAT_MJPG, OB_FORMAT_RGB, OB_FORMAT_YUYV}) {
            try {
                profile = profiles->getVideoStreamProfile(cfg.width, cfg.height, fmt, cfg.fps);
                if (profile) { chosen = fmt; break; }
            } catch (const ob::Error&) {
                profile = nullptr;
            }
        }

        if (!profile) {
            if (error) {
                *error = "找不到 " + std::to_string(cfg.width) + "x" + std::to_string(cfg.height) +
                         "@" + std::to_string(cfg.fps) + "fps 的彩色 profile（MJPG/RGB/YUYV 均不可用）";
                *error += "；可用彩色 profile:" + profileListString(profiles);
            }
            impl_->pipe.reset();
            impl_->callback = nullptr;
            return false;
        }

        auto config = std::make_shared<ob::Config>();
        config->enableStream(profile);
        impl_->pipe->start(config);

        impl_->format = chosen;
        impl_->profile = std::to_string(profile->width()) + "x" +
                         std::to_string(profile->height()) + "@" +
                         std::to_string(profile->fps()) + "f" + std::to_string(formatCode(chosen));

        impl_->running.store(true);
        impl_->thread = std::thread([this] { impl_->captureLoop(); });
        return true;
    } catch (const ob::Error& e) {
        if (error) *error = std::string("Orbbec: ") + e.getMessage();
    } catch (const std::exception& e) {
        if (error) *error = std::string("Orbbec: ") + e.what();
    }

    // 失败时把流停掉，避免相机被占着导致下一次重试打不开。
    if (impl_->pipe) {
        try {
            impl_->pipe->stop();
        } catch (const ob::Error&) {
        } catch (const std::exception&) {
        }
        impl_->pipe.reset();
    }
    impl_->callback = nullptr;
    impl_->running.store(false);
    return false;
}

void CameraSource::stop() {
    if (!impl_) return;
    impl_->running.store(false);
    if (impl_->thread.joinable()) impl_->thread.join();
    if (impl_->pipe) {
        try {
            impl_->pipe->stop();
        } catch (const ob::Error&) {
        } catch (const std::exception&) {
        }
        impl_->pipe.reset();
    }
    impl_->callback = nullptr;
    impl_->profile.clear();
    impl_->format = OB_FORMAT_ANY;
}

bool CameraSource::running() const {
    return impl_ && impl_->running.load();
}

std::string CameraSource::activeProfile() const {
    return impl_ ? impl_->profile : std::string();
}

}  // namespace detection
}  // namespace pure_uav
