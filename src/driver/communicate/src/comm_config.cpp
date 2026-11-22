#include "comm_config.hpp"

#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <climits>
#include <cstdio>
#include <cstring>

namespace pure_uav {

bool loadCommConfig(const std::string& yaml_path, CommConfig& cfg) {
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root.IsMap() || !root["communicate"]) {
            return false;  // 没有 communicate 段
        }
        YAML::Node comm = root["communicate"];
        for (const char* name : {"fcu_sysid", "fcu_compid"}) {
            if (!comm[name]) continue;
            const int id = comm[name].as<int>();
            if (id < 0 || id > 255) return false;
            if (std::strcmp(name, "fcu_sysid") == 0)
                cfg.fcu_sysid = static_cast<uint8_t>(id);
            else
                cfg.fcu_compid = static_cast<uint8_t>(id);
        }
        if (comm["file"]) cfg.device = comm["file"].as<std::string>();
        if (comm["device"]) cfg.device = comm["device"].as<std::string>();
        if (comm["port"]) cfg.device = comm["port"].as<std::string>();
        if (comm["baudrate"]) cfg.baudrate = comm["baudrate"].as<int>();
        if (comm["baud"]) cfg.baudrate = comm["baud"].as<int>();
        if (comm["heartbeat_timeout_ms"]) cfg.heartbeat_timeout_ms = comm["heartbeat_timeout_ms"].as<int>();
        if (comm["stream"]) {
            YAML::Node s = comm["stream"];
            if (s["attitude_us"]) cfg.attitude_interval_us = s["attitude_us"].as<int>();
            if (s["local_position_us"]) cfg.local_position_interval_us = s["local_position_us"].as<int>();
            if (s["highres_imu_us"]) cfg.highres_imu_interval_us = s["highres_imu_us"].as<int>();
            if (s["battery_us"]) cfg.battery_interval_us = s["battery_us"].as<int>();
            if (s["system_time_us"]) cfg.system_time_interval_us = s["system_time_us"].as<int>();
        }
        return true;
    } catch (const YAML::Exception& e) {
        fprintf(stderr, "加载 yaml 配置失败: %s\n", e.what());
        return false;
    }
}

// 自动查找配置文件
std::string findConfigFile(const std::string& rel_path) {
    // 1. 当前工作目录直接找
    if (access(rel_path.c_str(), F_OK) == 0) return rel_path;
    // 2. 从可执行文件位置逐层往上找
    char buf[PATH_MAX];
    ssize_t n = readlink("/proc/self/exe", buf, PATH_MAX);
    if (n > 0 && n < (ssize_t)PATH_MAX) {
        buf[n] = '\0';
        std::string dir(buf);
        dir = dir.substr(0, dir.find_last_of('/'));
        for (int i = 0; i < 6; i++) {
            std::string cand = dir + "/" + rel_path;
            if (access(cand.c_str(), F_OK) == 0) return cand;
            auto pos = dir.find_last_of('/');
            if (pos == std::string::npos) break;
            dir = dir.substr(0, pos);
        }
    }

    return rel_path;
}

}  // namespace pure_uav
