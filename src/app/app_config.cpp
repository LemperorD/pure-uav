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

bool loadAppConfig(const std::string& app_name, const ConfigOverrides& overrides, AppConfig& out, std::string& error) {
    try {
        AppConfig c;
        fs::path root = findRoot(fs::read_symlink("/proc/self/exe"));
        if (root.empty()) root = findRoot(fs::absolute(app_name));
        if (root.empty()) root = findRoot(fs::current_path());
        require(!root.empty(), "Cannot locate repository root from executable or config path");
        const auto resolve = [&](const fs::path& p) { return fs::weakly_canonical(p.is_absolute() ? p : root / p); };
        c.repository_root = root.string();
        c.app_path = resolve(app_name).string();
        const auto top = YAML::LoadFile(c.app_path);
        keys(top, {"app"}, "root");
        const auto app = top["app"];
        require(app && app.IsMap(), "Missing app mapping in " + c.app_path);
        keys(app, {"main_loop_hz", "heartbeat_wait_s", "status_period_s", "guidance_enabled", "driver_config", "vio_config"}, "app");
        read(app, "main_loop_hz", c.main_loop_hz);
        read(app, "heartbeat_wait_s", c.heartbeat_wait_s);
        read(app, "status_period_s", c.status_period_s);
        read(app, "guidance_enabled", c.guidance_enabled);
        require(!c.guidance_enabled, "guidance_enabled must remain false; control is not implemented");
        c.driver_path = "config/driver.yaml";
        c.vio_path = "config/vio.yaml";
        read(app, "driver_config", c.driver_path);
        read(app, "vio_config", c.vio_path);
        if (!overrides.driver_path.empty()) c.driver_path = overrides.driver_path;
        if (!overrides.vio_path.empty()) c.vio_path = overrides.vio_path;
        require(!c.driver_path.empty() && !c.vio_path.empty(), "driver_config and vio_config must not be empty");
        c.driver_path = resolve(c.driver_path).string();
        c.vio_path = resolve(c.vio_path).string();
        const auto vio_top = YAML::LoadFile(c.vio_path);
        keys(vio_top, {"vio"}, "root");
        const auto vio = vio_top["vio"];
        require(vio && vio.IsMap(), "Missing vio mapping in " + c.vio_path);
        keys(vio, {"config_dir", "width", "height", "fps", "imu_hz", "use_depth", "use_depth_init", "init_vel_zero", "static_init", "online_timeoffset", "input", "pose"}, "vio");
        c.vio.config_dir = "src/vio/estimator/config/gemini_1920x1080";
        c.vio.verbose = false;
        c.vio.quiet_runtime = true;
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
        if (!overrides.config_dir.empty()) c.vio.config_dir = overrides.config_dir;
        require(!c.vio.config_dir.empty(), "vio.config_dir must not be empty");
        c.vio.config_dir = resolve(c.vio.config_dir).string();
        const auto input = vio["input"];
        keys(input, {"shm_vio", "timeout_ms"}, "vio.input");
        read(input, "shm_vio", c.camera.channel);
        read(input, "timeout_ms", c.camera.timeout_ms);
        if (!overrides.shm_vio.empty()) c.camera.channel = overrides.shm_vio;
        if (overrides.shm_timeout_ms >= 0) c.camera.timeout_ms = overrides.shm_timeout_ms;
        require(c.camera.channel.size() > 1 && c.camera.channel.front() == '/' && c.camera.channel.find('/', 1) == std::string::npos, "vio.input.shm_vio must be a POSIX shared memory name such as /pure_uav_camera_vio");
        range(c.camera.timeout_ms, 0, 60000, "vio.input.timeout_ms");
        const auto pose = vio["pose"];
        keys(pose, {"max_age_ms", "quality_ok", "quality_degraded", "pos_sigma_ok_m", "vel_sigma_ok_mps", "pos_sigma_degraded_m", "vel_sigma_degraded_mps"}, "vio.pose");
        read(pose, "max_age_ms", c.max_age_ms);
        read(pose, "quality_ok", c.publication.quality_ok);
        read(pose, "quality_degraded", c.publication.quality_degraded);
        read(pose, "pos_sigma_ok_m", c.publication.pos_sigma_ok_m);
        read(pose, "vel_sigma_ok_mps", c.publication.vel_sigma_ok_mps);
        read(pose, "pos_sigma_degraded_m", c.publication.pos_sigma_degraded_m);
        read(pose, "vel_sigma_degraded_mps", c.publication.vel_sigma_degraded_mps);
        range(c.main_loop_hz, 1, 200, "main_loop_hz");
        range(c.heartbeat_wait_s, 0.1, 120, "heartbeat_wait_s");
        range(c.status_period_s, 0.1, 60, "status_period_s");
        range(c.max_age_ms, 1, 2000, "vio.pose.max_age_ms");
        range(c.camera.fps, 1, 120, "vio.fps");
        range(c.imu_hz, 21, 1000, "vio.imu_hz");
        range(c.publication.quality_ok, 1, 100, "quality_ok");
        range(c.publication.quality_degraded, 1, c.publication.quality_ok, "quality_degraded");
        range(c.publication.pos_sigma_ok_m, 0.001, 100, "pos_sigma_ok_m");
        range(c.publication.vel_sigma_ok_mps, 0.001, 100, "vel_sigma_ok_mps");
        range(c.publication.pos_sigma_degraded_m, c.publication.pos_sigma_ok_m, 100, "pos_sigma_degraded_m");
        range(c.publication.vel_sigma_degraded_mps, c.publication.vel_sigma_ok_mps, 100, "vel_sigma_degraded_mps");
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
        if (!overrides.config_dir.empty() && resolution.size() == 2) {
            c.camera.width = resolution[0];
            c.camera.height = resolution[1];
        }
        range(c.camera.width, 160, 4096, "vio.width");
        range(c.camera.height, 120, 2160, "vio.height");
        require(resolution.size() == 2 && resolution[0] == c.camera.width && resolution[1] == c.camera.height, "VIO width/height do not match calibrated cam0.resolution");
        std::vector<double> intrinsics;
        cam0["intrinsics"] >> intrinsics;
        require(intrinsics.size() == 4 && std::isfinite(intrinsics[0]) && intrinsics[0] > 0, "Invalid calibrated cam0.intrinsics");
        c.camera.config_dir = c.vio.config_dir;
        c.camera.estimator_config = c.vio.estimator_config;
        require(loadCommConfig(c.driver_path, c.comm), "Failed to load driver configuration: " + c.driver_path);
        require(!c.comm.device.empty(), "communicate.file must not be empty");
        require(c.comm.baudrate == 57600 || c.comm.baudrate == 115200 || c.comm.baudrate == 230400 || c.comm.baudrate == 460800 || c.comm.baudrate == 921600, "Unsupported communicate.baudrate");
        range(c.comm.heartbeat_timeout_ms, 100, 30000, "heartbeat_timeout_ms");
        const int expected_us = static_cast<int>(std::lround(1e6 / c.imu_hz));
        require(c.comm.highres_imu_interval_us == expected_us, "driver highres_imu_us must match vio.imu_hz (expected " + std::to_string(expected_us) + ")");
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
    s << std::boolalpha << "repository_root: " << c.repository_root << "\napp_config: " << c.app_path << "\ndriver_config: " << c.driver_path << "\nvio_config: " << c.vio_path << "\nmain_loop_hz: " << c.main_loop_hz << "\nheartbeat_wait_s: " << c.heartbeat_wait_s
      << "\nstatus_period_s: " << c.status_period_s << "\nvio.config_dir: " << c.vio.config_dir << "\nvio.resolution/fps: " << c.camera.width << 'x' << c.camera.height << '@' << c.camera.fps << "\nvio.imu_hz/request_interval_us: " << c.imu_hz << '/' << c.comm.highres_imu_interval_us
      << "\nvio.use_depth/use_depth_init: " << c.camera.use_depth << '/' << c.vio.use_depth_init << "\nvio.init_vel_zero/static_init/online_timeoffset: " << c.vio.init_vel_zero << '/' << c.vio.static_init << '/' << c.vio.estimate_camera_timeoffset << "\nvio.input.shm_vio: " << c.camera.channel
      << "\nvio.input.timeout_ms: " << c.camera.timeout_ms << "\nvio.pose.max_age_ms: " << c.max_age_ms << "\nvio.pose.quality_ok/degraded: " << c.publication.quality_ok << '/' << c.publication.quality_degraded << "\nvio.pose.pos_sigma_ok/degraded_m: " << c.publication.pos_sigma_ok_m << '/'
      << c.publication.pos_sigma_degraded_m << "\nvio.pose.vel_sigma_ok/degraded_mps: " << c.publication.vel_sigma_ok_mps << '/' << c.publication.vel_sigma_degraded_mps << "\nserial: " << c.comm.device << " @ " << c.comm.baudrate << '\n';
    return s.str();
}
}  // namespace pure_uav::app
