#pragma once

#include <cstdint>
#include <string>

namespace pure_uav {

// 通信模块配置
struct CommConfig {
    std::string device = "/dev/ttyACM0";
    int baudrate = 921600;
    int heartbeat_timeout_ms = 2000;  // 心跳超时判定掉线（毫秒）
    // 消息流下发间隔（微秒）
    int attitude_interval_us = 20000;        // 姿态 50Hz
    int local_position_interval_us = 20000;  // 位置+速度 50Hz
    int highres_imu_interval_us = 10000;     // IMU 100Hz
    int battery_interval_us = 1000000;       // 电量 1Hz
    int system_time_interval_us = 1000000;   // 时间同步 1Hz
    // 【2026-10-29】目标飞控身份。0 = 自动锁定（首个"像飞控"的心跳）。
    // 锁定后只接受该 sysid/compid 的心跳与数据；地面站/机载计算机/其他飞行器一律忽略。
    uint8_t fcu_sysid = 0;
    uint8_t fcu_compid = 0;  // 0 = 不限定组件号
};

// 从 YAML 加载通信配置
bool loadCommConfig(const std::string& yaml_path, CommConfig& cfg);

// 自动查找配置文件：先试相对路径（当前目录），再从可执行文件位置逐层往上找
std::string findConfigFile(const std::string& rel_path = "config/driver.yaml");

inline bool validHighresImuInterval(int interval_us) {
    return interval_us >= 1000 && interval_us <= 47619;
}

}  // namespace pure_uav
