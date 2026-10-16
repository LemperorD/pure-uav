#include "communicator.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

// mavlink c_library_v2 纯头文件：include 路径指向 third_party/mavlink
#include "common/mavlink.h"
#include "mavlink_helpers.h"

// yaml-cpp：配置解析
#include <yaml-cpp/yaml.h>

namespace pure_uav {

namespace {
constexpr uint8_t kFcuCompId = 1;

// 单调时钟毫秒
uint32_t timeBootMs() {
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

//欧拉角 → 四元数 wxyz
void eulerToQuat(float roll, float pitch, float yaw, float q[4]) {
    const float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
    const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    const float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
    q[0] = cr * cp * cy + sr * sp * sy;  // w
    q[1] = sr * cp * cy - cr * sp * sy;  // x
    q[2] = cr * sp * cy + sr * cp * sy;  // y
    q[3] = cr * cp * sy - sr * sp * cy;  // z
}
}  // namespace

Communicator::~Communicator() {
    stop();
}

bool Communicator::start(const std::string& device, int baud) {   //打开串口
    if (running_.load()) {
        return true;
    }
    if (!port_.open(device, baud)) {
        printf("打开串口失败: %s\n", device.c_str());
        return false;
    }
    port_.flush();   //清空缓冲
    running_.store(true);
    recv_thread_ = std::thread(&Communicator::recvLoop, this);    //开启接收线程
    return true;
}

void Communicator::stop() {    //关闭接收线程
    running_.store(false);
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
    port_.close();
    connected_.store(false);
}


// 接收线程
void Communicator::recvLoop() {
    using namespace std::chrono;
    const auto heartbeat_timeout = milliseconds(cfg_.heartbeat_timeout_ms);

    mavlink_message_t msg{};
    mavlink_status_t status{}; 
    uint8_t buf[1024];
    auto last_heartbeat = steady_clock::now();

    while (running_.load()) {
        ssize_t n = port_.read(buf, sizeof(buf));

        if (n > 0) {
            for (ssize_t i = 0; i < n; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_HEARTBEAT: {     //心跳和飞行模式
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
                            fcu_sysid_.store(msg.sysid);   // 学习飞控 sysid
                            custom_mode_.store(hb.custom_mode);
                            connected_.store(true);
                            last_heartbeat = steady_clock::now();
                            // 首次连上飞控，请求需要的消息流
                            if (!streams_requested_.exchange(true)) {
                                requestDefaultStreams();
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_ATTITUDE: {      //姿态
                            mavlink_attitude_t att;
                            mavlink_msg_attitude_decode(&msg, &att);
                            std::lock_guard<std::mutex> lk(data_mutex_);
                            attitude_.roll = att.roll;
                            attitude_.pitch = att.pitch;
                            attitude_.yaw = att.yaw;
                            eulerToQuat(att.roll, att.pitch, att.yaw, attitude_.q);
                            break;
                        }
                        case MAVLINK_MSG_ID_BATTERY_STATUS: {      //电量
                            mavlink_battery_status_t bat;
                            mavlink_msg_battery_status_decode(&msg, &bat);
                            std::lock_guard<std::mutex> lk(data_mutex_);
                            // 累加所有有效 cell 电压得到总电压（UINT16_MAX = 该 cell 无效）
                            float total_mv = 0.0f;
                            for (int i = 0; i < 10; ++i) {
                                if (bat.voltages[i] != UINT16_MAX) {
                                    total_mv += bat.voltages[i];
                                }
                            }
                            battery_.voltage = total_mv / 1000.0f;  // mV → V
                            battery_.remaining = bat.battery_remaining;    // %
                            break;
                        }
                        case MAVLINK_MSG_ID_COMMAND_ACK: {      //ACK
                            mavlink_command_ack_t ack;
                            mavlink_msg_command_ack_decode(&msg, &ack);
                            last_ack_command_.store(ack.command);
                            last_ack_result_.store(ack.result);
                            break;
                        }
                        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {      //位置和速度
                            mavlink_local_position_ned_t lp;
                            mavlink_msg_local_position_ned_decode(&msg, &lp);
                            std::lock_guard<std::mutex> lk(data_mutex_);
                            local_pos_.x = lp.x;
                            local_pos_.y = lp.y;
                            local_pos_.z = lp.z;
                            local_pos_.vx = lp.vx;
                            local_pos_.vy = lp.vy;
                            local_pos_.vz = lp.vz;
                            break;
                        }
                        case MAVLINK_MSG_ID_SYSTEM_TIME: {      //时间戳
                            mavlink_system_time_t st;
                            mavlink_msg_system_time_decode(&msg, &st);
                            std::lock_guard<std::mutex> lk(data_mutex_);
                            system_time_.time_unix_usec = st.time_unix_usec;
                            system_time_.time_boot_ms = st.time_boot_ms;
                            break;
                        }
                        case MAVLINK_MSG_ID_HIGHRES_IMU: {    //IMU原数数据
                            mavlink_highres_imu_t imu;
                            mavlink_msg_highres_imu_decode(&msg, &imu);
                            std::lock_guard<std::mutex> lk(data_mutex_);
                            imu_.timestamp_us = imu.time_usec; 
                            imu_.gx = imu.xgyro;
                            imu_.gy = imu.ygyro;
                            imu_.gz = imu.zgyro;
                            imu_.ax = imu.xacc;
                            imu_.ay = imu.yacc;
                            imu_.az = imu.zacc;
                            break;
                        }
                        default:
                            break;
                    }
                }
            }
        }

        // 心跳超时 → 判定断开
        if (connected_.load() &&
            steady_clock::now() - last_heartbeat > heartbeat_timeout) {
            connected_.store(false);
            streams_requested_.store(false);  // 掉线重置，飞控恢复后重新请求消息流
        }

        if (n <= 0) {
            std::this_thread::sleep_for(milliseconds(1));
        }
    }
}


// 发送底层
bool Communicator::sendMessage(const uint8_t* buf, uint16_t len) {
    std::lock_guard<std::mutex> lk(send_mutex_);
    size_t off = 0;
    while (off < len) {
        ssize_t n = port_.write(buf + off, len - off);
        if (n > 0) {
            off += static_cast<size_t>(n);
        } else {
            return false;  
        }
    }
    return true;
}

// 飞行模式 / ACK 结果 中文名映射
const char* Communicator::px4ModeNameCn(uint32_t custom_mode) const {
    const uint8_t main_mode = (custom_mode >> 16) & 0xFF;
    const uint8_t sub_mode  = (custom_mode >> 24) & 0xFF;
    switch (main_mode) {
        case 1: return "手动";
        case 2: return "定高";
        case 3:
            if (sub_mode == 1) return "环绕";
            if (sub_mode == 2) return "慢速定点";
            return "定点";
        case 4:
            switch (sub_mode) {
                case 1: return "自动·就绪";
                case 2: return "自动·起飞";
                case 3: return "自动·悬停";
                case 4: return "自动·任务";
                case 5: return "自动·返航";
                case 6: return "自动·降落";
                case 8: return "自动·跟随";
                case 9: return "自动·精准降落";
                default: return "自动";
            }
        case 5: return "特技";
        case 6: return "外部控制";
        case 7: return "自稳";
        case 10: return "故障终止";
        case 11: return "高度巡航";
        default: return "未知";
    }
}

const char* Communicator::ackResultNameCn(uint8_t result) const {
    switch (result) {
        case MAV_RESULT_ACCEPTED:             return "已接受";
        case MAV_RESULT_TEMPORARILY_REJECTED: return "临时拒绝";
        case MAV_RESULT_DENIED:               return "拒绝";
        case MAV_RESULT_UNSUPPORTED:          return "不支持";
        case MAV_RESULT_FAILED:               return "失败";
        case MAV_RESULT_IN_PROGRESS:          return "进行中";
        default:                              return "未知";
    }
}


// 各模块向飞控通讯
bool Communicator::pushVisionPose(const VisionPose& p) {   //vio数据
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    float pose_cov[21] = {0.0f};
    pose_cov[0] = std::nanf(""); 
    float vel_cov[21] = {0.0f};
    vel_cov[0] = std::nanf("");  
    mavlink_msg_odometry_pack(sysid_, compid_, &msg,
                              p.timestamp_us,
                              static_cast<uint8_t>(p.frame_id),       
                              static_cast<uint8_t>(p.child_frame_id),  
                              p.x, p.y, p.z, p.q,
                              p.vx, p.vy, p.vz,
                              0.0f, 0.0f, 0.0f,        
                              pose_cov, vel_cov,
                              0,                      
                              MAV_ESTIMATOR_TYPE_VIO,  
                              0);                     
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return sendMessage(buf, len);
}

bool Communicator::pushGuidanceCmd(const GuidanceCmd& c) {   //发送制导律
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    //   POSITION：忽略速度 + 加速度 + yaw_rate，只发位置 + yaw
    //   VELOCITY：忽略位置 + 加速度 + yaw_rate，只发速度 + yaw
    //   POS_WITH_VEL_FEEDFORWARD：忽略加速度 + yaw_rate，位置 + 速度 + yaw
    //   ACCELERATION：忽略位置 + 速度 + yaw + yaw_rate，只发加速度
    uint16_t type_mask = POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    if (c.mode == GuidanceMode::POSITION) {
        // 纯位置闭环：忽略速度 + 加速度
        type_mask |= POSITION_TARGET_TYPEMASK_VX_IGNORE |
                     POSITION_TARGET_TYPEMASK_VY_IGNORE |
                     POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                     POSITION_TARGET_TYPEMASK_AX_IGNORE |
                     POSITION_TARGET_TYPEMASK_AY_IGNORE |
                     POSITION_TARGET_TYPEMASK_AZ_IGNORE;
    } else if (c.mode == GuidanceMode::VELOCITY) {
        // 纯速度闭环：忽略位置 + 加速度
        type_mask |= POSITION_TARGET_TYPEMASK_X_IGNORE |
                     POSITION_TARGET_TYPEMASK_Y_IGNORE |
                     POSITION_TARGET_TYPEMASK_Z_IGNORE |
                     POSITION_TARGET_TYPEMASK_AX_IGNORE |
                     POSITION_TARGET_TYPEMASK_AY_IGNORE |
                     POSITION_TARGET_TYPEMASK_AZ_IGNORE;
    } else if (c.mode == GuidanceMode::POS_WITH_VEL_FEEDFORWARD) {
        // 位置目标 + 速度前馈：忽略加速度
        type_mask |= POSITION_TARGET_TYPEMASK_AX_IGNORE |
                     POSITION_TARGET_TYPEMASK_AY_IGNORE |
                     POSITION_TARGET_TYPEMASK_AZ_IGNORE;
    } else if (c.mode == GuidanceMode::ACCELERATION) {
        // 加速度 setpoint：忽略位置 + 速度 + yaw，只保留加速度
        type_mask |= POSITION_TARGET_TYPEMASK_X_IGNORE |
                     POSITION_TARGET_TYPEMASK_Y_IGNORE |
                     POSITION_TARGET_TYPEMASK_Z_IGNORE |
                     POSITION_TARGET_TYPEMASK_VX_IGNORE |
                     POSITION_TARGET_TYPEMASK_VY_IGNORE |
                     POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                     POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    } else {
        return false;  // 未知 mode，拒绝发送
    }
    mavlink_msg_set_position_target_local_ned_pack(
        sysid_, compid_, &msg, timeBootMs(), fcu_sysid_.load(), kFcuCompId, MAV_FRAME_LOCAL_NED,
        type_mask, c.x, c.y, c.z, c.vx, c.vy, c.vz, c.afx, c.afy, c.afz, c.yaw, 0.0f);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return sendMessage(buf, len);
}

bool Communicator::sendTakeoff(float altitude_m, int timeout_ms, uint8_t* result) {    //命令PX4起飞到目标高度
    last_ack_command_.store(0);
    last_ack_result_.store(UINT8_MAX);
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(sysid_, compid_, &msg, fcu_sysid_.load(), kFcuCompId,
                                  MAV_CMD_NAV_TAKEOFF, 0,
                                  0, 0, 0, 0, 0, 0, altitude_m);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (!sendMessage(buf, len)) return false;
    return waitForAck(MAV_CMD_NAV_TAKEOFF, timeout_ms, result);
}

bool Communicator::sendLand(int timeout_ms, uint8_t* result) {   //命令PX4自动降落
    last_ack_command_.store(0);
    last_ack_result_.store(UINT8_MAX);
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(sysid_, compid_, &msg, fcu_sysid_.load(), kFcuCompId,
                                  MAV_CMD_NAV_LAND, 0,
                                  0, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (!sendMessage(buf, len)) return false;
    return waitForAck(MAV_CMD_NAV_LAND, timeout_ms, result);
}

bool Communicator::sendSetMode(uint32_t custom_mode, int timeout_ms, uint8_t* result) {   //设置飞行模式（完整 px4 custom_mode）
    // PX4 的 DO_SET_MODE：param2 = main_mode（bit23-16），param3 = sub_mode（bit31-24）
    uint8_t main_mode = (custom_mode >> 16) & 0xFF;
    uint8_t sub_mode  = (custom_mode >> 24) & 0xFF;
    last_ack_command_.store(0);
    last_ack_result_.store(UINT8_MAX);
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(sysid_, compid_, &msg, fcu_sysid_.load(), kFcuCompId,
                                  MAV_CMD_DO_SET_MODE, 0,
                                  MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main_mode, sub_mode,
                                  0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (!sendMessage(buf, len)) return false;
    return waitForAck(MAV_CMD_DO_SET_MODE, timeout_ms, result);
}

bool Communicator::sendArm(bool arm, int timeout_ms, uint8_t* result) {   //解锁/加锁
    last_ack_command_.store(0);
    last_ack_result_.store(UINT8_MAX);
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // MAV_CMD_COMPONENT_ARM_DISARM：param1 = 1 解锁，0 加锁
    mavlink_msg_command_long_pack(sysid_, compid_, &msg, fcu_sysid_.load(), kFcuCompId,
                                  MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                  arm ? 1.0f : 0.0f, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (!sendMessage(buf, len)) return false;
    return waitForAck(MAV_CMD_COMPONENT_ARM_DISARM, timeout_ms, result);
}

bool Communicator::requestMessageInterval(uint32_t msgid, int32_t interval_us) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // MAV_CMD_SET_MESSAGE_INTERVAL(511)：param1=消息ID，param2=间隔(微秒，-1禁用，0默认)
    mavlink_msg_command_long_pack(sysid_, compid_, &msg, fcu_sysid_.load(), kFcuCompId,
                                  MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                  static_cast<float>(msgid),
                                  static_cast<float>(interval_us),
                                  0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return sendMessage(buf, len);
}

// 等某条命令的 COMMAND_ACK（带超时）。必须在 sendMessage 之后调用。
bool Communicator::waitForAck(uint16_t command, int timeout_ms, uint8_t* result) {
    using namespace std::chrono;
    const auto deadline = steady_clock::now() + milliseconds(timeout_ms);
    while (steady_clock::now() < deadline) {
        if (last_ack_command_.load() == command && last_ack_result_.load() != UINT8_MAX) {
            if (result) *result = last_ack_result_.load();
            return true;
        }
        std::this_thread::sleep_for(milliseconds(5));
    }
    return false;  // 超时
}

void Communicator::requestDefaultStreams() {
    // 向PX4请求以下数据（间隔来自 config/driver.yaml 的 stream 段）
    requestMessageInterval(MAVLINK_MSG_ID_ATTITUDE, cfg_.attitude_interval_us);
    requestMessageInterval(MAVLINK_MSG_ID_LOCAL_POSITION_NED, cfg_.local_position_interval_us);
    requestMessageInterval(MAVLINK_MSG_ID_HIGHRES_IMU, cfg_.highres_imu_interval_us);
    requestMessageInterval(MAVLINK_MSG_ID_BATTERY_STATUS, cfg_.battery_interval_us);
    requestMessageInterval(MAVLINK_MSG_ID_SYSTEM_TIME, cfg_.system_time_interval_us);
}




//各模块拿数据
bool Communicator::getAttitude(Attitude& out) const {
    std::lock_guard<std::mutex> lk(data_mutex_);
    out = attitude_;
    return true;
}

bool Communicator::getBattery(Battery& out) const {
    std::lock_guard<std::mutex> lk(data_mutex_);
    out = battery_;
    return true;
}

bool Communicator::getLocalPosition(LocalPosition& out) const {
    std::lock_guard<std::mutex> lk(data_mutex_);
    out = local_pos_;
    return true;
}

bool Communicator::getImu(ImuData& out) const {
    std::lock_guard<std::mutex> lk(data_mutex_);
    out = imu_;
    return true;
}

bool Communicator::getSystemTime(SystemTime& out) const {
    std::lock_guard<std::mutex> lk(data_mutex_);
    out = system_time_;
    return true;
}

const char* Communicator::getFlightModeNameCn() const {
    return px4ModeNameCn(custom_mode_.load());
}

const char* Communicator::getLastAck() const {
    const uint8_t result = last_ack_result_.load();
    if (result == UINT8_MAX) return "未收到";  // 尚未收到 ACK
    return ackResultNameCn(result);
}

// 从 YAML 加载通信配置（config/driver.yaml）
bool loadCommConfig(const std::string& yaml_path, CommConfig& cfg) {
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root.IsMap() || !root["communicate"]) {
            return false;  // 没有 communicate 段
        }
        YAML::Node comm = root["communicate"];
        if (comm["file"])     cfg.device   = comm["file"].as<std::string>();
        if (comm["device"])   cfg.device   = comm["device"].as<std::string>();
        if (comm["port"])     cfg.device   = comm["port"].as<std::string>();
        if (comm["baudrate"]) cfg.baudrate = comm["baudrate"].as<int>();
        if (comm["baud"])     cfg.baudrate = comm["baud"].as<int>();
        if (comm["heartbeat_timeout_ms"]) cfg.heartbeat_timeout_ms = comm["heartbeat_timeout_ms"].as<int>();
        if (comm["stream"]) {
            YAML::Node s = comm["stream"];
            if (s["attitude_us"])        cfg.attitude_interval_us        = s["attitude_us"].as<int>();
            if (s["local_position_us"])  cfg.local_position_interval_us  = s["local_position_us"].as<int>();
            if (s["highres_imu_us"])     cfg.highres_imu_interval_us     = s["highres_imu_us"].as<int>();
            if (s["battery_us"])         cfg.battery_interval_us         = s["battery_us"].as<int>();
            if (s["system_time_us"])     cfg.system_time_interval_us     = s["system_time_us"].as<int>();
        }
        return true;
    } catch (const YAML::Exception& e) {
        fprintf(stderr, "加载 yaml 配置失败: %s\n", e.what());
        return false;
    }
}

}  // namespace pure_uav
