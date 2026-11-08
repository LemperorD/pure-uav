#include "app_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <initializer_list>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pure_uav::app {
namespace {
namespace fs = std::filesystem;
void require(bool condition, const std::string& reason) {
    if (!condition) throw std::runtime_error(reason);
}
fs::path findRoot(fs::path start) {
    if (!fs::is_directory(start)) start = start.parent_path();
    while (!start.empty()) {
        if (fs::exists(start / "src/vio/estimator/CMakeLists.txt") && fs::exists(start / "config/driver.yaml")) return fs::canonical(start);
        auto next = start.parent_path();
        if (next == start) break;
        start = next;
    }
    return {};
}
void keys(const YAML::Node& node, std::initializer_list<const char*> allowed, const std::string& name) {
    if (!node) return;
    require(node.IsMap(), name + " must be a mapping");
    for (const auto& entry : node) {
        const auto key = entry.first.as<std::string>();
        bool known = false;
        for (const char* candidate : allowed)
            if (key == candidate) known = true;
        require(known, "Unknown setting " + name + "." + key);
    }
}
template <class T>
void read(const YAML::Node& node, const char* key, T& value) {
    if (node && node[key]) value = node[key].as<T>();
}
void range(double value, double lo, double hi, const char* name) {
    require(std::isfinite(value) && value >= lo && value <= hi, std::string(name) + " must be in [" + std::to_string(lo) + ", " + std::to_string(hi) + "]");
}
bool cvBool(const cv::FileNode& node) {
    if (node.empty()) return false;
    if (node.isString()) {
        const auto text = static_cast<std::string>(node);
        if (text == "false" || text == "0") return false;
        if (text == "true" || text == "1") return true;
        throw std::runtime_error("Invalid boolean in VIO configuration: " + node.name());
    }
    require(node.isInt(), "Invalid boolean in VIO configuration: " + node.name());
    const int value = static_cast<int>(node);
    require(value == 0 || value == 1, "Invalid boolean in VIO configuration: " + node.name());
    return value == 1;
}
cv::FileStorage openCv(const fs::path& path) {
    require(fs::is_regular_file(path), "VIO configuration file missing: " + path.string());
    cv::FileStorage storage(path.string(), cv::FileStorage::READ);
    require(storage.isOpened(), "Cannot parse VIO configuration: " + path.string());
    return storage;
}
}  // namespace

bool loadAppConfig(const std::string& app_name, const std::string& driver_name, AppConfig& out, std::string& error) {
    try {
        AppConfig c;
        fs::path root = findRoot(fs::read_symlink("/proc/self/exe"));
        if (root.empty()) root = findRoot(fs::absolute(app_name));
        if (root.empty()) root = findRoot(fs::current_path());
        require(!root.empty(), "Cannot locate repository root from executable or config path");
        const auto resolve = [&](const fs::path& p) { return fs::weakly_canonical(p.is_absolute() ? p : root / p); };
        c.repository_root = root.string();
        c.app_path = resolve(app_name).string();
        c.driver_path = resolve(driver_name).string();
        const auto top = YAML::LoadFile(c.app_path);
        keys(top, {"app"}, "root");
        const auto app = top["app"];
        require(app && app.IsMap(), "Missing app mapping in " + c.app_path);
        keys(app, {"main_loop_hz", "heartbeat_wait_s", "status_period_s", "pose", "vio", "camera", "guidance_enabled", "detection_enabled", "detection"}, "app");
        read(app, "main_loop_hz", c.main_loop_hz);
        read(app, "heartbeat_wait_s", c.heartbeat_wait_s);
        read(app, "status_period_s", c.status_period_s);
        read(app, "guidance_enabled", c.guidance_enabled);
        read(app, "detection_enabled", c.detection_enabled);
        const auto detection = app["detection"];
        keys(detection, {"model_path", "imgsz", "num_classes", "conf_thr", "iou_thr", "max_det", "target_class", "max_hz", "max_age_ms"}, "app.detection");
        read(detection, "model_path", c.detection.model_path);
        read(detection, "imgsz", c.detection.imgsz);
        read(detection, "num_classes", c.detection.num_classes);
        read(detection, "conf_thr", c.detection.conf_thr);
        read(detection, "iou_thr", c.detection.iou_thr);
        read(detection, "max_det", c.detection.max_det);
        read(detection, "target_class", c.detection.target_class);
        read(detection, "max_hz", c.detection.max_hz);
        read(detection, "max_age_ms", c.detection.max_age_ms);
        c.detection.model_path = resolve(c.detection.model_path).string();
        std::string detection_error;
        require(detection::validateRuntimeConfig(c.detection, detection_error), detection_error);
        if (c.detection_enabled) {
            require(fs::is_regular_file(c.detection.model_path), "Detection model file missing: " + c.detection.model_path);
            require(detection::npuBackendAvailable(),
                    "NPU detection unavailable in this build: need vip_lite.h, libNBGlinker.so and libVIPhal.so; "
                    "build on the NPU board with -DPURE_DETECTION_BUILD=ON");
        }
        const auto pose = app["pose"];
        keys(pose, {"max_age_ms", "quality_ok", "quality_degraded", "pos_sigma_ok_m", "vel_sigma_ok_mps", "pos_sigma_degraded_m", "vel_sigma_degraded_mps"}, "app.pose");
        read(pose, "max_age_ms", c.max_age_ms);
        read(pose, "quality_ok", c.publication.quality_ok);
        read(pose, "quality_degraded", c.publication.quality_degraded);
        read(pose, "pos_sigma_ok_m", c.publication.pos_sigma_ok_m);
        read(pose, "vel_sigma_ok_mps", c.publication.vel_sigma_ok_mps);
        read(pose, "pos_sigma_degraded_m", c.publication.pos_sigma_degraded_m);
        read(pose, "vel_sigma_degraded_mps", c.publication.vel_sigma_degraded_mps);
        const auto vio = app["vio"];
        keys(vio, {"config_dir", "width", "height", "fps", "imu_hz", "use_depth", "use_depth_init", "init_vel_zero", "static_init", "online_timeoffset"}, "app.vio");
        c.vio.config_dir = "src/vio/estimator/config/gemini_1920x1080";
        c.vio.verbose = false;
        c.vio.quiet_runtime = true;
        c.vio.use_depth_init = true;
        c.vio.init_vel_zero = true;
        read(vio, "config_dir", c.vio.config_dir);
        read(vio, "width", c.camera.width);
        read(vio, "height", c.camera.height);
        read(vio, "fps", c.camera.fps);
        read(vio, "imu_hz", c.imu_hz);
        read(vio, "use_depth", c.camera.use_depth);
        read(vio, "use_depth_init", c.vio.use_depth_init);
        read(vio, "init_vel_zero", c.vio.init_vel_zero);
        read(vio, "static_init", c.vio.static_init);
        read(vio, "online_timeoffset", c.vio.estimate_camera_timeoffset);
        c.vio.config_dir = resolve(c.vio.config_dir).string();
        const auto camera = app["camera"];
        keys(camera, {"aggregate_mode", "auto_exposure", "exposure", "gain", "white_balance", "auto_tune_exposure", "tune_expected_rate_dps", "tune_blur_budget_px", "tune_max_gain"}, "app.camera");
        read(camera, "aggregate_mode", c.camera.aggregate_mode);
        read(camera, "auto_exposure", c.camera.auto_exposure);
        read(camera, "exposure", c.camera.exposure);
        read(camera, "gain", c.camera.gain);
        read(camera, "white_balance", c.camera.white_balance);
        read(camera, "auto_tune_exposure", c.camera.auto_tune_exposure);
        read(camera, "tune_expected_rate_dps", c.camera.tune_expected_rate_dps);
        read(camera, "tune_blur_budget_px", c.camera.tune_blur_budget_px);
        read(camera, "tune_max_gain", c.camera.tune_max_gain);
        range(c.main_loop_hz, 1, 200, "main_loop_hz");
        range(c.heartbeat_wait_s, 0.1, 120, "heartbeat_wait_s");
        range(c.status_period_s, 0.1, 60, "status_period_s");
        range(c.max_age_ms, 1, 2000, "max_age_ms");
        range(c.camera.width, 160, 4096, "width");
        range(c.camera.height, 120, 2160, "height");
        range(c.camera.fps, 1, 120, "fps");
        range(c.imu_hz, 21, 1000, "imu_hz");
        range(c.publication.quality_ok, 1, 100, "quality_ok");
        range(c.publication.quality_degraded, 1, c.publication.quality_ok, "quality_degraded");
        range(c.publication.pos_sigma_ok_m, 0.001, 100, "pos_sigma_ok_m");
        range(c.publication.vel_sigma_ok_mps, 0.001, 100, "vel_sigma_ok_mps");
        range(c.publication.pos_sigma_degraded_m, c.publication.pos_sigma_ok_m, 100, "pos_sigma_degraded_m");
        range(c.publication.vel_sigma_degraded_mps, c.publication.vel_sigma_ok_mps, 100, "vel_sigma_degraded_mps");
        range(c.camera.aggregate_mode, 0, 2, "aggregate_mode");
        range(c.camera.exposure, 1, 10000, "exposure");
        range(c.camera.gain, 0, 255, "gain");
        range(c.camera.white_balance, 1000, 12000, "white_balance");
        range(c.camera.tune_expected_rate_dps, 1, 1000, "tune_expected_rate_dps");
        range(c.camera.tune_blur_budget_px, 0.1, 20, "tune_blur_budget_px");
        range(c.camera.tune_max_gain, 0, 255, "tune_max_gain");
        require(!c.guidance_enabled, "guidance_enabled must remain false; control is not implemented");
        require(!c.vio.use_depth_init || c.camera.use_depth, "use_depth_init requires use_depth=true");
        require(fs::is_directory(c.vio.config_dir), "config_dir does not exist: " + c.vio.config_dir);
        const auto estimator = openCv(fs::path(c.vio.config_dir) / c.vio.estimator_config);
        require(!cvBool(estimator["record_timing_information"]), "record_timing_information must be false in the production runtime");
        for (const char* name : {"OV_DIAG_CSV", "OV_DIAG_UPD", "OV_DIAG_DEPTHERR", "OV_DIAG_FEATS", "OV_DIAG_FEATDEC", "OV_DEPTH_NO_EXTR", "OV_GYRO_PREDICT", "OV_MIN_ACCEPT", "OV_EKF_STRICT", "OV_RETUNE_HI", "OV_RETUNE_LO"}) {
            require(std::getenv(name) == nullptr, std::string("Unset diagnostic override ") + name + " before using the production runtime");
        }
        const std::string imu_chain = static_cast<std::string>(estimator["relative_config_imu"]);
        const std::string cam_chain = static_cast<std::string>(estimator["relative_config_imucam"]);
        require(!imu_chain.empty() && !cam_chain.empty(), "Missing relative_config_imu/imucam");
        const auto imu_config = openCv(fs::path(c.vio.config_dir) / imu_chain);
        require(!imu_config["imu0"].empty(), "IMU chain must contain imu0");
        const auto cam_config = openCv(fs::path(c.vio.config_dir) / cam_chain);
        const auto cam0 = cam_config["cam0"];
        std::vector<int> resolution;
        cam0["resolution"] >> resolution;
        require(resolution.size() == 2 && resolution[0] == c.camera.width && resolution[1] == c.camera.height, "Camera width/height do not match calibrated cam0.resolution");
        std::vector<double> intrinsics;
        cam0["intrinsics"] >> intrinsics;
        require(intrinsics.size() == 4 && std::isfinite(intrinsics[0]) && intrinsics[0] > 0, "Invalid calibrated cam0.intrinsics");
        c.camera.tune_focal_px = intrinsics[0];
        require(loadCommConfig(c.driver_path, c.comm), "Failed to load driver configuration: " + c.driver_path);
        const auto driver = YAML::LoadFile(c.driver_path)["communicate"];
        if (driver["fcu_sysid"]) {
            const int id = driver["fcu_sysid"].as<int>();
            range(id, 0, 255, "fcu_sysid");
            c.comm.fcu_sysid = static_cast<uint8_t>(id);
        }
        if (driver["fcu_compid"]) {
            const int id = driver["fcu_compid"].as<int>();
            range(id, 0, 255, "fcu_compid");
            c.comm.fcu_compid = static_cast<uint8_t>(id);
        }
        require(!c.comm.device.empty(), "communicate.file must not be empty");
        require(c.comm.baudrate == 57600 || c.comm.baudrate == 115200 || c.comm.baudrate == 230400 || c.comm.baudrate == 460800 || c.comm.baudrate == 921600, "Unsupported communicate.baudrate");
        range(c.comm.heartbeat_timeout_ms, 100, 30000, "heartbeat_timeout_ms");
        const int expected_us = static_cast<int>(std::lround(1e6 / c.imu_hz));
        require(c.comm.highres_imu_interval_us == expected_us, "driver highres_imu_us must match app.vio.imu_hz (expected " + std::to_string(expected_us) + ")");
        out = std::move(c);
        error.clear();
        return true;
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }
}
std::string describeConfig(const AppConfig& c) {
    std::ostringstream s;
    s << std::boolalpha << "repository_root: " << c.repository_root << "\napp_config: " << c.app_path << "\ndriver_config: " << c.driver_path << "\nmain_loop_hz: " << c.main_loop_hz << "\nheartbeat_wait_s: " << c.heartbeat_wait_s << "\nstatus_period_s: " << c.status_period_s
      << "\npose.max_age_ms: " << c.max_age_ms << " (capture and production age, inclusive)"
      << "\npose.quality_ok/degraded: " << c.publication.quality_ok << '/' << c.publication.quality_degraded << "\npose.pos_sigma_ok/degraded_m: " << c.publication.pos_sigma_ok_m << '/' << c.publication.pos_sigma_degraded_m << "\npose.vel_sigma_ok/degraded_mps: " << c.publication.vel_sigma_ok_mps
      << '/' << c.publication.vel_sigma_degraded_mps << "\nvio.config_dir: " << c.vio.config_dir << "\ncamera: " << c.camera.width << 'x' << c.camera.height << '@' << c.camera.fps << "\nvio.imu_hz/request_interval_us: " << c.imu_hz << '/' << c.comm.highres_imu_interval_us
      << "\nvio.use_depth/use_depth_init: " << c.camera.use_depth << '/' << c.vio.use_depth_init << "\nvio.init_vel_zero/static_init/online_timeoffset: " << c.vio.init_vel_zero << '/' << c.vio.static_init << '/' << c.vio.estimate_camera_timeoffset
      << "\ncamera.aggregate_mode/auto_exposure: " << c.camera.aggregate_mode << '/' << c.camera.auto_exposure << "\ncamera.exposure/gain/white_balance: " << c.camera.exposure << '/' << c.camera.gain << '/' << c.camera.white_balance
      << "\ncamera.auto_tune_exposure/expected_rate_dps/blur_budget_px/max_gain: " << c.camera.auto_tune_exposure << '/' << c.camera.tune_expected_rate_dps << '/' << c.camera.tune_blur_budget_px << '/' << c.camera.tune_max_gain << "\nserial: " << c.comm.device << " @ " << c.comm.baudrate
      << "\nheartbeat_timeout_ms: " << c.comm.heartbeat_timeout_ms << "\ntelemetry intervals attitude/local_position/battery/system_time_us: " << c.comm.attitude_interval_us << '/' << c.comm.local_position_interval_us << '/' << c.comm.battery_interval_us << '/' << c.comm.system_time_interval_us
      << "\nfcu_sysid/compid (0=auto): " << unsigned(c.comm.fcu_sysid) << '/' << unsigned(c.comm.fcu_compid) << "\nguidance_enabled: " << c.guidance_enabled << "\ndetection_enabled: " << c.detection_enabled << "\ndetection.npu_available: " << detection::npuBackendAvailable()
      << "\ndetection.model_path: " << c.detection.model_path << "\ndetection.imgsz/classes/target_class: " << c.detection.imgsz << '/' << c.detection.num_classes << '/' << c.detection.target_class << "\ndetection.conf/iou/max_det: " << c.detection.conf_thr << '/' << c.detection.iou_thr << '/'
      << c.detection.max_det << "\ndetection.max_hz/max_age_ms: " << c.detection.max_hz << '/' << c.detection.max_age_ms << '\n';
    return s.str();
}
}  // namespace pure_uav::app
