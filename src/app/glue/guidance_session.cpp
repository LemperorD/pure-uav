#include "glue/guidance_session.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <initializer_list>
#include <memory>
#include <string>

#include "communicator.hpp"
#include "guidance_loop.hpp"
#include "runtime/guidance_ports.hpp"
#include "startup/guidance_application.hpp"

namespace pure_uav::app {
namespace {

namespace flight = guidance::flight;

class CommInsSource final : public flight::GuidanceInsSource {
  public:
    CommInsSource(Communicator& comm, double max_skew_s) : comm_(comm), max_skew_s_(max_skew_s) {
    }

    bool poll(flight::GuidanceInsSample& out) override {
        if (!comm_.isConnected()) {
            return false;
        }
        Attitude attitude;
        LocalPosition position;
        if (!comm_.getAttitude(attitude) || !comm_.getLocalPosition(position)) {
            return false;
        }
        flight::GuidanceInsSample sample;
        if (!makeGuidanceIns(attitude, position, max_skew_s_, sample) || sample.boot_us <= last_) {
            return false;
        }
        last_ = sample.boot_us;
        out = sample;
        return true;
    }

    bool alive() const override {
        return comm_.isConnected();
    }

  private:
    Communicator& comm_;
    double max_skew_s_ = 0.2;
    std::uint64_t last_ = 0;
};

// 本阶段没有检测目标；占位源只保证 GuidanceLoop 能构造和 init。
class NullTargetSource final : public flight::GuidanceTargetSource {
  public:
    bool poll(flight::GuidanceTargetSample& /*out*/) override {
        return false;
    }
    bool alive() const override {
        return true;
    }
};

struct CameraCalibration {
    int width = 0;
    int height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double distortion[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double R_body_cam[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::string source;
};

bool checkKeys(const YAML::Node& node, std::initializer_list<const char*> allowed, const char* section, std::string& error) {
    if (!node) {
        return true;
    }
    if (!node.IsMap()) {
        error = std::string(section) + " 必须是映射";
        return false;
    }
    for (const auto& item : node) {
        if (!item.first.IsScalar()) {
            error = std::string(section) + " 存在非字符串键";
            return false;
        }
        const std::string key = item.first.as<std::string>();
        bool known = false;
        for (const char* name : allowed) {
            if (key == name) {
                known = true;
                break;
            }
        }
        if (!known) {
            error = std::string(section) + " 存在未知键: " + key;
            return false;
        }
    }
    return true;
}

template <typename T>
bool readValue(const YAML::Node& node, const char* key, T& out, std::string& error, const char* type) {
    if (!node || !node[key]) {
        return true;
    }
    try {
        out = node[key].as<T>();
    } catch (const YAML::Exception&) {
        error = std::string("键 ") + key + " 不是" + type;
        return false;
    }
    return true;
}

bool readString(const YAML::Node& node, const char* key, std::string& out, std::string& error) {
    return readValue<std::string>(node, key, out, error, "字符串");
}

bool readInt(const YAML::Node& node, const char* key, int& out, std::string& error) {
    return readValue<int>(node, key, out, error, "整数");
}

bool readDouble(const YAML::Node& node, const char* key, double& out, std::string& error) {
    return readValue<double>(node, key, out, error, "浮点数");
}

bool readBool(const YAML::Node& node, const char* key, bool& out, std::string& error) {
    return readValue<bool>(node, key, out, error, "布尔值");
}

bool rotationValid(const double R[9], std::string& error) {
    for (int i = 0; i < 9; ++i) {
        if (!std::isfinite(R[i])) {
            error = "R_body_cam 含非有限数";
            return false;
        }
    }
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            double dot = 0.0;
            for (int k = 0; k < 3; ++k) {
                dot += R[row * 3 + k] * R[col * 3 + k];
            }
            const double expected = (row == col) ? 1.0 : 0.0;
            if (std::fabs(dot - expected) > 1e-3) {
                error = "R_body_cam 不是正交矩阵";
                return false;
            }
        }
    }
    const double det = R[0] * (R[4] * R[8] - R[5] * R[7]) - R[1] * (R[3] * R[8] - R[5] * R[6]) + R[2] * (R[3] * R[7] - R[4] * R[6]);
    if (std::fabs(det - 1.0) > 1e-3) {
        error = "R_body_cam 行列式不是 +1";
        return false;
    }
    return true;
}

}  // namespace

struct GuidanceSession::Impl {
    GuidanceAppParams params;
    std::unique_ptr<CommInsSource> ins;
    std::unique_ptr<NullTargetSource> target;
    std::unique_ptr<flight::GuidanceNullSink> sink;
    std::unique_ptr<flight::GuidanceLoop> loop;

    bool params_ok = false;
    bool camera_ok = false;
    bool camera_fatal = false;
    bool ready = false;
    std::string camera_reason;
    CameraCalibration camera;

    bool initParams(const std::string& path, std::string& error);
    CameraLoadResult initCamera(const std::string& path, std::string& error);
    bool attach(Communicator& comm, std::string& error);
};

bool GuidanceSession::Impl::initParams(const std::string& path, std::string& error) {
    loop.reset();
    ins.reset();
    target.reset();
    sink.reset();
    ready = false;
    params_ok = false;

    params = guidance_default_params();
    const GuidanceParamResult loaded = guidance_load_yaml_file(path, &params);
    if (!loaded.ok) {
        error = loaded.error;
        return false;
    }
    const GuidanceParamResult valid = guidance_validate_params(params);
    if (!valid.ok) {
        error = valid.error;
        return false;
    }
    if (params.guidance.law != GuidanceLaw::PPN || params.guidance.los_rate_source != LosRateSource::FROM_LOS_DIFFERENCE || params.guidance.t_go_freeze != 0.0) {
        error = "当前无测距机载路径只支持 PPN + FROM_LOS_DIFFERENCE + t_go_freeze=0";
        return false;
    }
    params_ok = true;
    return true;
}

CameraLoadResult GuidanceSession::Impl::initCamera(const std::string& path, std::string& error) {
    camera_ok = false;
    camera_fatal = false;
    camera_reason.clear();
    camera = CameraCalibration{};
    params.loop.cam = flight::GuidanceCameraModel{};

    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
        camera_reason = "找不到相机标定文件: " + path;
        camera_fatal = true;
        error = camera_reason;
        return CameraLoadResult::kError;
    }

    try {
        const YAML::Node root = YAML::LoadFile(path);
        if (!root || !root.IsMap()) {
            camera_reason = "相机标定文件不是映射: " + path;
            camera_fatal = true;
            error = camera_reason;
            return CameraLoadResult::kError;
        }
        if (!checkKeys(root, {"calibrated", "source", "width", "height", "camera_model", "distortion_model", "fx", "fy", "cx", "cy", "distortion", "R_body_cam"}, "camera", error)) {
            camera_reason = error;
            camera_fatal = true;
            return CameraLoadResult::kError;
        }

        bool calibrated = false;
        if (!root["calibrated"]) {
            camera_reason = "相机标定缺少 calibrated";
            camera_fatal = true;
            error = camera_reason;
            return CameraLoadResult::kError;
        }
        if (!readBool(root, "calibrated", calibrated, error)) {
            camera_reason = error;
            camera_fatal = true;
            return CameraLoadResult::kError;
        }
        if (!readString(root, "source", camera.source, error)) {
            camera_reason = error;
            camera_fatal = true;
            return CameraLoadResult::kError;
        }

        if (!calibrated) {
            camera_reason = "calibrated=false";
            if (!camera.source.empty()) {
                camera_reason += " (" + camera.source + ")";
            }
            return CameraLoadResult::kNotCalibrated;
        }

        if (!readInt(root, "width", camera.width, error) || !readInt(root, "height", camera.height, error) || !readDouble(root, "fx", camera.fx, error) || !readDouble(root, "fy", camera.fy, error) || !readDouble(root, "cx", camera.cx, error) || !readDouble(root, "cy", camera.cy, error)) {
            camera_reason = error;
            camera_fatal = true;
            return CameraLoadResult::kError;
        }

        std::string camera_model;
        std::string distortion_model;
        if (!readString(root, "camera_model", camera_model, error) || !readString(root, "distortion_model", distortion_model, error)) {
            camera_reason = error;
            camera_fatal = true;
            return CameraLoadResult::kError;
        }
        if (camera_model != "pinhole" || distortion_model != "radtan") {
            camera_reason = "当前只支持 pinhole + radtan 相机模型";
            camera_fatal = true;
            error = camera_reason;
            return CameraLoadResult::kError;
        }

        const YAML::Node distortion = root["distortion"];
        if (!distortion || !distortion.IsSequence() || (distortion.size() != 4 && distortion.size() != 5)) {
            camera_reason = "distortion 必须是 4 或 5 个系数";
            camera_fatal = true;
            error = camera_reason;
            return CameraLoadResult::kError;
        }
        for (std::size_t i = 0; i < distortion.size(); ++i) {
            try {
                camera.distortion[i] = distortion[i].as<double>();
            } catch (const YAML::Exception&) {
                camera_reason = "distortion 含非数字";
                camera_fatal = true;
                error = camera_reason;
                return CameraLoadResult::kError;
            }
            if (!std::isfinite(camera.distortion[i])) {
                camera_reason = "distortion 含非有限数";
                camera_fatal = true;
                error = camera_reason;
                return CameraLoadResult::kError;
            }
        }

        const YAML::Node rotation = root["R_body_cam"];
        if (!rotation || !rotation.IsSequence() || rotation.size() != 9) {
            camera_reason = "R_body_cam 必须是 9 个数";
            camera_fatal = true;
            error = camera_reason;
            return CameraLoadResult::kError;
        }
        for (std::size_t i = 0; i < rotation.size(); ++i) {
            try {
                camera.R_body_cam[i] = rotation[i].as<double>();
            } catch (const YAML::Exception&) {
                camera_reason = "R_body_cam 含非数字";
                camera_fatal = true;
                error = camera_reason;
                return CameraLoadResult::kError;
            }
        }

        if (camera.width <= 0 || camera.height <= 0 || !(camera.fx > 0.0) || !(camera.fy > 0.0) || !std::isfinite(camera.cx) || !std::isfinite(camera.cy)) {
            camera_reason = "相机内参/分辨率非法";
            camera_fatal = true;
            error = camera_reason;
            return CameraLoadResult::kError;
        }
        if (!rotationValid(camera.R_body_cam, error)) {
            camera_reason = error;
            camera_fatal = true;
            return CameraLoadResult::kError;
        }

        params.loop.cam.fx = camera.fx;
        params.loop.cam.fy = camera.fy;
        params.loop.cam.cx = camera.cx;
        params.loop.cam.cy = camera.cy;
        for (int i = 0; i < 9; ++i) {
            params.loop.cam.R_body_cam[i] = camera.R_body_cam[i];
        }
        camera_ok = true;
        camera_reason = camera.source.empty() ? "calibrated=true" : ("calibrated=true (" + camera.source + ")");
        return CameraLoadResult::kOk;
    } catch (const YAML::Exception& e) {
        camera_reason = std::string("相机 YAML 解析失败: ") + e.what();
        camera_fatal = true;
        error = camera_reason;
        return CameraLoadResult::kError;
    } catch (const std::exception& e) {
        camera_reason = std::string("相机标定加载失败: ") + e.what();
        camera_fatal = true;
        error = camera_reason;
        return CameraLoadResult::kError;
    }
}

bool GuidanceSession::Impl::attach(Communicator& comm, std::string& error) {
    loop.reset();
    sink.reset();
    target.reset();
    ins.reset();
    ready = false;
    if (!params_ok) {
        error = "guidance 参数尚未初始化";
        return false;
    }
    if (!camera_ok) {
        error = camera_reason.empty() ? "相机标定未就绪" : camera_reason;
        return false;
    }

    ins = std::make_unique<CommInsSource>(comm, params.loop.max_sample_age_s);
    target = std::make_unique<NullTargetSource>();
    sink = std::make_unique<flight::GuidanceNullSink>();
    loop = std::make_unique<flight::GuidanceLoop>(params.guidance, params.loop, *ins, *target, *sink);
    if (!loop->init()) {
        error = "GuidanceLoop::init 失败（参数或相机模型无效）";
        camera_fatal = true;
        loop.reset();
        return false;
    }
    ready = true;
    return true;
}

GuidanceSession::GuidanceSession() : impl_(std::make_unique<Impl>()) {
}

GuidanceSession::~GuidanceSession() = default;

bool GuidanceSession::initParams(const std::string& path, std::string& error) {
    return impl_->initParams(path, error);
}

CameraLoadResult GuidanceSession::initCamera(const std::string& path, std::string& error) {
    return impl_->initCamera(path, error);
}

bool GuidanceSession::attach(Communicator& comm, std::string& error) {
    return impl_->attach(comm, error);
}

bool GuidanceSession::paramsOk() const {
    return impl_->params_ok;
}

bool GuidanceSession::cameraOk() const {
    return impl_->camera_ok;
}

bool GuidanceSession::cameraFatal() const {
    return impl_->camera_fatal;
}

bool GuidanceSession::readyForGuidance() const {
    return impl_->ready;
}

const std::string& GuidanceSession::cameraReason() const {
    return impl_->camera_reason;
}

const GuidanceAppParams& GuidanceSession::params() const {
    return impl_->params;
}

}  // namespace pure_uav::app
