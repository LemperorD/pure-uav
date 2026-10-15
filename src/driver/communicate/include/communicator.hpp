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



// Communicator：飞控 PX4 的 MAVLink 通信模块
class Communicator {
public:
    Communicator() = default;
    ~Communicator();

    Communicator(const Communicator&) = delete;
    Communicator& operator=(const Communicator&) = delete;

    // 打开串口并启动接收线程
    bool start(const std::string& device, int baud);

    // 停止接收线程并关闭串口
    void stop();

    // MAVLink 身份
    void setSystemId(uint8_t id) { sysid_ = id; }
    void setComponentId(uint8_t id) { compid_ = id; }

    
    
    //上位机向飞控通讯
    bool pushVisionPose(const VisionPose& p);   // 发视觉位姿
    bool pushGuidanceCmd(const GuidanceCmd& c); // 发制导期望状态（Offboard）
    bool sendTakeoff(float altitude_m);         // 起飞（NAV_TAKEOFF）   高度m
    bool sendLand();                            // 降落（NAV_LAND）
    bool sendSetMode(uint32_t custom_mode);     // 切换飞行模式（DO_SET_MODE）  custom_mode自定义
    bool sendArm(bool arm);                     // 解锁/加锁（COMPONENT_ARM_DISARM）   true解锁  false加锁
    bool requestMessageInterval(uint32_t msgid, int32_t interval_us);  // 请求某消息的发送频率
    void requestDefaultStreams();               // 连接后请求默认消息流



    // 各模块拿数据
    bool getAttitude(Attitude& out) const;   //姿态
    bool getBattery(Battery& out) const;     //电量
    bool getLocalPosition(LocalPosition& out) const;   //位置和速度
    bool getImu(ImuData& out) const;                 //IMU数据
    bool getSystemTime(SystemTime& out) const;      //时间戳
    bool isConnected() const { return connected_.load(); }  //连接是否成功
    uint32_t getCustomMode() const { return custom_mode_.load(); } //飞行模式
    uint8_t lastAckResult() const { return last_ack_result_.load(); } //ACK

private:
    void recvLoop();  // 接收线程
    bool sendMessage(const uint8_t* buf, uint16_t len);  // 串口写

    SerialPort port_;
    std::thread recv_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> streams_requested_{false};  // 是否已请求过消息流

    uint8_t sysid_ = 2;   // 上位机 system id
    uint8_t compid_ = 1;    // 上位机 component id
    std::atomic<uint8_t> fcu_sysid_{1};  // 飞控 system id（从心跳学习）

    mutable std::mutex data_mutex_;
    Attitude attitude_;  //roll  pitch  yaw  rad   四元数
    Battery battery_;    //V  百分比剩余电量
    LocalPosition local_pos_;   //速度与位置 m/s   m 
    ImuData imu_;    //角速度 rad/s  加速度m/s*s  时间戳us 由数量级判断是什么时间戳
    SystemTime system_time_;    //时间戳  UNIX时间us   飞控开机时间ms
    std::atomic<uint32_t> custom_mode_{0};  //整数值 映射自定义
    std::atomic<uint8_t> last_ack_result_{UINT8_MAX};   

    std::mutex send_mutex_;  // 串口写锁
};

}  // namespace pure_uav
