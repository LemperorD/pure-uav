#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "serial_port.hpp"

namespace pure_uav {
//飞控数据结构体
// 机体姿态（rad）
struct Attitude {
    float roll = 0.0f;  //机体FRD相对惯性NED的姿态
    float pitch = 0.0f;
    float yaw = 0.0f;
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w x y z
};

// 电池电量
struct Battery {
    float voltage = 0.0f;    // 电压（V）
    int8_t remaining = -1;   // 剩余百分比（-1 表示未知）
};



// 飞控局部位置 + 速度（惯性NED）
struct LocalPosition {
    float x = 0.0f, y = 0.0f, z = 0.0f;       // 位置（m）
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;    // 速度（m/s）
};

// 飞控回传 IMU（角速度 + 加速度，机体FRD）
struct ImuData {
    uint64_t timestamp_us = 0;                    // 该帧 IMU 时间戳（微秒，飞控 boot 时间）
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;        // 角速度（rad/s）
    float ax = 0.0f, ay = 0.0f, az = 0.0f;        // 加速度（m/s²）
};

// 飞控时间戳（用于 VIO 时间同步）
struct SystemTime {
    uint64_t time_unix_usec = 0;   // Unix 时间戳（微秒）
    uint32_t time_boot_ms = 0;     // 上电时间（毫秒）
};




//上位机数据结构体
// 位姿/速度的参考坐标系
enum class VisionFrame : uint8_t {
    LOCAL_NED = 1,         // 世界系 NED（X北 Y东 Z下）
    BODY_FRD = 12,         // 机体 FRD（X前 Y右 Z下）
    LOCAL_FRD = 20,        // 世界系 FRD
};
// VIO 位姿
struct VisionPose {
    uint64_t timestamp_us = 0;  //us
    VisionFrame frame_id = VisionFrame::LOCAL_FRD;        // 位置的坐标系（惯性系）
    VisionFrame child_frame_id = VisionFrame::BODY_FRD;   // 速度的坐标系（机体系）
    float x = 0.0f;     //m
    float y = 0.0f;
    float z = 0.0f;
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w x y z      机体系相对惯性系，由上面选的参数指定
    float vx = 0.0f;  //  m/s
    float vy = 0.0f;
    float vz = 0.0f;
};

// 制导控制模式
enum class GuidanceMode : uint8_t {
    POSITION = 0,  // 位置控制（发 x/y/z）
    VELOCITY = 1,  // 速度控制（发 vx/vy/vz）
    POS_WITH_VEL_FEEDFORWARD = 2,  // 位置目标 + 速度前馈
    ACCELERATION = 3,  // 加速度 （发 afx/afy/afz）
};

// 制导指令：期望状态（NED）
struct GuidanceCmd {
    GuidanceMode mode = GuidanceMode::VELOCITY;  // 控制模式
    float x = 0.0f;    // 期望位置（mode=POSITION 时有效）
    float y = 0.0f;
    float z = 0.0f;
    float vx = 0.0f;   // 期望速度（mode=VELOCITY 时有效）
    float vy = 0.0f;
    float vz = 0.0f;
    float afx = 0.0f;  // 期望加速度（mode=ACCELERATION 时有效）
    float afy = 0.0f;
    float afz = 0.0f;
    float yaw = 0.0f;  // 期望航向（rad）
};


// custom_mode变量值  作为 bool sendSetMode(uint32_t custom_mode)的输入
namespace px4_mode {
constexpr uint32_t MANUAL       = (0u << 24) | (1u << 16);  //手动
constexpr uint32_t ALTCTL       = (0u << 24) | (2u << 16);  //定高
constexpr uint32_t POSCTL       = (0u << 24) | (3u << 16);  //定点
constexpr uint32_t ACRO         = (0u << 24) | (5u << 16);  //特技
constexpr uint32_t OFFBOARD     = (0u << 24) | (6u << 16);  //外部控制
constexpr uint32_t STABILIZED   = (0u << 24) | (7u << 16);  //自稳
constexpr uint32_t ALT_CRUISE   = (0u << 24) | (11u << 16); //高度巡航
constexpr uint32_t AUTO_READY   = (1u << 24) | (4u << 16);  //自动-就绪
constexpr uint32_t AUTO_TAKEOFF = (2u << 24) | (4u << 16);  //自动-起飞
constexpr uint32_t AUTO_LOITER  = (3u << 24) | (4u << 16);  //自动-悬停
constexpr uint32_t AUTO_MISSION = (4u << 24) | (4u << 16);  //自动-任务
constexpr uint32_t AUTO_RTL     = (5u << 24) | (4u << 16);  //自动-返航
constexpr uint32_t AUTO_LAND    = (6u << 24) | (4u << 16);  //自动-降落
}  // namespace px4_mode

// ArduPilot ArduCopter 飞行模式  作为 bool sendSetMode(uint32_t custom_mode)的输入
namespace arducopter_mode {
constexpr uint8_t STABILIZE = 0;   // 自稳
constexpr uint8_t ACRO      = 1;   // 特技
constexpr uint8_t ALT_HOLD  = 2;   // 定高
constexpr uint8_t AUTO      = 3;   // 自动任务
constexpr uint8_t GUIDED    = 4;   // 引导（离板控制，对应 PX4 Offboard）
constexpr uint8_t LOITER    = 5;   // 悬停
constexpr uint8_t RTL       = 6;   // 返航
constexpr uint8_t CIRCLE    = 7;   // 环绕
constexpr uint8_t LAND      = 9;   // 降落
constexpr uint8_t POSHOLD   = 16;  // 定点
constexpr uint8_t SMARTRTL  = 21;  // 智能返航
}  // namespace arducopter_mode

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
};

// 从 YAML 加载通信配置
bool loadCommConfig(const std::string& yaml_path, CommConfig& cfg);

// 自动查找配置文件：先试相对路径（当前目录），再从可执行文件位置逐层往上找
std::string findConfigFile(const std::string& rel_path = "config/driver.yaml");


// Communicator：飞控 PX4 的 MAVLink 通信模块
class Communicator {
public:
    Communicator() = default;
    ~Communicator();

    Communicator(const Communicator&) = delete;
    Communicator& operator=(const Communicator&) = delete;

    // 打开串口并启动接收线程
    bool start(const std::string& device, int baud);
    bool start(const CommConfig& cfg) { cfg_ = cfg; return start(cfg.device, cfg.baudrate); }  // 用配置启动

    // 停止接收线程并关闭串口
    void stop();

    // MAVLink 身份
    void setSystemId(uint8_t id) { sysid_ = id; }
    void setComponentId(uint8_t id) { compid_ = id; }

    
    
    //上位机向飞控通讯
    bool pushVisionPose(const VisionPose& p);   // 发视觉位姿
    bool pushGuidanceCmd(const GuidanceCmd& c); // 发制导期望状态（Offboard）
    bool sendTakeoff(float altitude_m, int timeout_ms = 1000, uint8_t* result = nullptr);  // 起飞（NAV_TAKEOFF）  高度m
    bool sendLand(int timeout_ms = 1000, uint8_t* result = nullptr);                       // 降落（NAV_LAND）
    bool sendSetMode(uint32_t custom_mode, int timeout_ms = 1000, uint8_t* result = nullptr);  // 切换飞行模式（DO_SET_MODE），传完整 px4_mode 值
    bool sendArm(bool arm, int timeout_ms = 1000, uint8_t* result = nullptr);              // 解锁/加锁（COMPONENT_ARM_DISARM）
    bool requestMessageInterval(uint32_t msgid, int32_t interval_us);  // 请求某消息频率（非阻塞，recvLoop 内调用）
    void requestDefaultStreams();               // 连接后请求默认消息流



    // 各模块拿数据
    bool getAttitude(Attitude& out) const;   //姿态
    bool getBattery(Battery& out) const;     //电量
    bool getLocalPosition(LocalPosition& out) const;   //位置和速度
    bool getImu(ImuData& out) const;                 //IMU数据
    bool getSystemTime(SystemTime& out) const;      //时间戳
    bool isConnected() const { return connected_.load(); }  //连接是否成功
    uint32_t getCustomMode() const { return custom_mode_.load(); } //飞行模式（完整 custom_mode 值）
    const char* getFlightModeNameCn() const;    // 当前飞行模式中文名
    uint8_t getAutopilot() const { return autopilot_.load(); }    // 飞控固件类型（3=ArduPilot, 12=PX4）
    const char* getAutopilotNameCn() const;     // 飞控固件中文名
    uint8_t lastAckResult() const { return last_ack_result_.load(); } //最近一次 ACK 结果
    const char* getLastAck() const;  // 最近一次 ACK 结果中文名（未收到返回"未收到"）

private:
    void recvLoop();  // 接收线程
    bool sendMessage(const uint8_t* buf, uint16_t len);  // 串口写
    bool waitForAck(uint16_t command, int timeout_ms, uint8_t* result);  // 等某条命令的 ACK（超时返回 false）
    const char* px4ModeNameCn(uint32_t custom_mode) const;  // PX4 飞行模式 → 中文名
    const char* arducopterModeNameCn(uint8_t custom_mode) const;  // ArduCopter 飞行模式 → 中文名
    const char* ackResultNameCn(uint8_t result) const;      // ACK result → 中文名

    CommConfig cfg_;  // 运行时配置（start 时保存）
    SerialPort port_;
    std::thread recv_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> streams_requested_{false};  // 是否已请求过消息流

    uint8_t sysid_ = 2;   // 上位机 system id
    uint8_t compid_ = 1;    // 上位机 component id
    std::atomic<uint8_t> fcu_sysid_{1};  // 飞控 system id（从心跳学习）
    std::atomic<uint8_t> autopilot_{0};  // 飞控固件类型（3=ArduPilot, 12=PX4, 0=未知）

    mutable std::mutex data_mutex_;
    Attitude attitude_;  //roll  pitch  yaw  rad   四元数
    Battery battery_;    //V  百分比剩余电量
    LocalPosition local_pos_;   //速度与位置 m/s   m 
    ImuData imu_;    //角速度 rad/s  加速度m/s*s  时间戳us 由数量级判断是什么时间戳
    SystemTime system_time_;    //时间戳  UNIX时间us   飞控开机时间ms
    std::atomic<uint32_t> custom_mode_{0};  //完整 custom_mode 整数值
    std::atomic<uint16_t> last_ack_command_{0};  //最近一次 ACK 对应的命令号
    std::atomic<uint8_t> last_ack_result_{UINT8_MAX};   //最近一次 ACK 结果（UINT8_MAX=未收到）

    std::mutex send_mutex_;  // 串口写锁
};

}  // namespace pure_uav
