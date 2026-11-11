#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "msg.hpp"
#include "serial_port.hpp"

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

// Communicator：飞控 PX4 的 MAVLink 通信模块
class Communicator {
  public:
    Communicator() = default;
    ~Communicator();

    Communicator(const Communicator&) = delete;
    Communicator& operator=(const Communicator&) = delete;

    // 打开串口并启动接收线程
    bool start(const std::string& device, int baud);
    bool start(const CommConfig& cfg) {
        cfg_ = cfg;
        return start(cfg.device, cfg.baudrate);
    }  // 用配置启动

    // 停止接收线程并关闭串口
    void stop();

    // MAVLink 身份
    void setSystemId(uint8_t id) {
        sysid_ = id;
    }
    void setComponentId(uint8_t id) {
        compid_ = id;
    }

    // 上位机向飞控通讯
    bool pushVisionPose(const VisionPose& p);  // 发视觉位姿（质量/重置/协方差取 p 里的字段）
    // Session-aware variant for asynchronous producers; the original API is preserved.
    bool pushVisionPoseForSession(const VisionPose& p, uint64_t expected_session);
    bool pushGuidanceCmdForSession(const GuidanceCmd& c, uint64_t expected_session);
    // 任务使用的非阻塞请求：只发送，不等 ACK；动作完成由飞控反馈确认。
    bool requestOffboardForSession(uint64_t expected_session);
    bool requestArmForSession(uint64_t expected_session);
    bool requestLandForSession(uint64_t expected_session);
    bool getLandedState(LandedState& out, int max_age_ms = 2000) const;
    bool pushGuidanceCmd(const GuidanceCmd& c);                                                // 发制导期望状态（Offboard）
    bool sendTakeoff(float altitude_m, int timeout_ms = 1000, uint8_t* result = nullptr);      // 起飞（NAV_TAKEOFF）  高度m
    bool sendLand(int timeout_ms = 1000, uint8_t* result = nullptr);                           // 降落（NAV_LAND）
    bool sendSetMode(uint32_t custom_mode, int timeout_ms = 1000, uint8_t* result = nullptr);  // 切换飞行模式（DO_SET_MODE），传完整 px4_mode 值
    bool sendArm(bool arm, int timeout_ms = 1000, uint8_t* result = nullptr);                  // 解锁/加锁（COMPONENT_ARM_DISARM）
    bool requestMessageInterval(uint32_t msgid, int32_t interval_us);                          // 请求某消息频率（非阻塞，recvLoop 内调用）
    void requestDefaultStreams();                                                              // 连接后请求默认消息流

    //=======================================================================
    // 【2026-10-28 联调新增】逐帧 IMU 交付
    //
    // 原来只有 getImu()，它返回"最近一帧"的覆盖式缓存。VIO 必须按时间顺序
    // 一帧不落地吃 IMU（漏帧就等于积分少了一块），所以需要队列：
    //   · 接收线程只做"解码 + 入队"，耗时工作留在 VIO 自己的线程；
    //   · 每条带飞控 boot 时间戳 + 到达主机时的单调时钟（给 FcuClock 做映射）；
    //   · 溢出丢最旧的帧并单独计数，绝不静默丢；
    //   · IMU 不变，getImu() 保留，老模块不用改。
    //=======================================================================
    struct TimedImu {
        ImuData imu;           // timestamp_us 是飞控 boot 时间
        int64_t host_ns = 0;   // 收到该帧时的主机单调时钟（steady_clock）
        uint64_t session = 0;  // 接收时的会话代数，reconnectCount() 包含心跳超时
    };
    bool popImu(TimedImu& out);  // 取队列里最早的一帧；空队列返回 false
    size_t imuQueueSize() const;
    uint64_t droppedImu() const {
        return dropped_imu_.load();
    }  // 因队列满被丢的帧数
    uint64_t receivedImu() const {
        return count_highres_imu_.load();
    }  // 累计收到的帧数
    void setImuQueueCapacity(size_t cap);  // 0 表示不排队（只保留最新帧语义）
    size_t imuQueueCapacity() const {
        return imu_queue_capacity_.load();
    }

    // 各模块拿数据
    bool getAttitude(Attitude& out) const;  // 姿态
    bool getAttitude(Attitude& out, int max_age_ms) const;
    bool getAttitude(Attitude& out, int max_age_ms, int64_t* received_ns) const;
    bool getBattery(Battery& out) const;              // 电量
    bool getLocalPosition(LocalPosition& out) const;  // 位置和速度
    bool getLocalPosition(LocalPosition& out, int max_age_ms) const;
    bool getLocalPosition(LocalPosition& out, int max_age_ms, int64_t* received_ns) const;
    bool getImu(ImuData& out) const;            // IMU数据
    bool getSystemTime(SystemTime& out) const;  // 时间戳
    bool getSystemTime(SystemTime& out, int max_age_ms) const;
    int64_t attitudeAgeMs() const;
    int64_t localPositionAgeMs() const;
    int64_t systemTimeAgeMs() const;
    bool isConnected() const {
        return connected_.load();
    }  // 连接是否成功
    // 【2026-10-28】飞控是否已解锁（HEARTBEAT.base_mode 的 SAFETY_ARMED 位）。
    // VIO 用它决定地面零速更新是否开启：地面开、空中必须关。
    bool isArmed() const {
        return armed_.load();
    }
    // 是否已锁定目标飞控身份（收到第一个"像飞控"的心跳后为 true）
    bool targetLocked() const {
        return locked_sysid_.load() != 0;
    }
    // 【2026-10-29】会话代数。设备重开成功或心跳超时一次 +1。
    uint64_t reconnectCount() const {
        return reconnects_.load();
    }
    // 距离最近一次心跳的毫秒数；从未收到返回 -1。判断"连接状态还新不新鲜"用。
    int64_t lastHeartbeatAgeMs() const;
    uint32_t getCustomMode() const {
        return custom_mode_.load();
    }  // 飞行模式（完整 custom_mode 值）
    const char* getFlightModeNameCn() const;  // 当前飞行模式中文名
    uint8_t getAutopilot() const {
        return autopilot_.load();
    }  // 飞控固件类型（3=ArduPilot, 12=PX4）
    const char* getAutopilotNameCn() const;  // 飞控固件中文名
    uint8_t lastAckResult() const {
        return last_ack_result_.load();
    }  // 最近一次 ACK 结果
    const char* getLastAck() const;  // 最近一次 ACK 结果中文名（未收到返回"未收到"）
    // 各消息累计接收帧数（用于统计实际频率）
    uint32_t getCountAttitude() const {
        return count_attitude_.load();
    }
    uint32_t getCountLocalPosition() const {
        return count_local_position_.load();
    }
    uint32_t getCountImu() const {
        return count_highres_imu_.load();
    }
    uint32_t getCountBattery() const {
        return count_battery_.load();
    }
    uint32_t getCountSystemTime() const {
        return count_system_time_.load();
    }

  private:
    bool sendVisionPose(const VisionPose& p, std::optional<uint64_t> expected_session);
    bool sendGuidanceCmd(const GuidanceCmd& c, std::optional<uint64_t> expected_session);
    bool requestFlightCommand(uint16_t command, float p1, float p2, uint64_t expected_session);
    void recvLoop();  // 接收线程
    // 心跳是否来自目标飞控；首次匹配时锁定 sysid/compid
    bool acceptHeartbeat(uint8_t sysid, uint8_t compid, uint8_t autopilot, uint8_t type);
    // 每次连接会话开始/结束时的状态复位：清空 IMU 队列、缓存数据、解锁状态与身份锁定
    void resetSessionState();
    void invalidateTelemetry();
    // 只清 IMU 队列（串口掉线重连时用：保留解锁状态/计数，但旧时间基的帧不能留）
    void clearImuQueue();
    void sendGcsHeartbeat();  // 发一条 GCS 心跳（让飞控知道上位机在线，主动推流）
    bool sendMessage(const uint8_t* buf, uint16_t len, bool require_connected = false,
                     std::optional<uint64_t> expected_session = std::nullopt);  // 串口写
    bool waitForAck(uint16_t command, int timeout_ms, uint8_t* result);         // 等某条命令的 ACK（超时返回 false）
    const char* px4ModeNameCn(uint32_t custom_mode) const;                      // PX4 飞行模式 → 中文名
    const char* arducopterModeNameCn(uint8_t custom_mode) const;                // ArduCopter 飞行模式 → 中文名
    const char* ackResultNameCn(uint8_t result) const;                          // ACK result → 中文名

    CommConfig cfg_;  // 运行时配置（start 时保存）
    SerialPort port_;
    std::thread recv_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> streams_requested_{false};  // 是否已请求过消息流

    uint8_t sysid_ = 2;                     // 上位机 system id
    uint8_t compid_ = 1;                    // 上位机 component id
    std::atomic<uint8_t> fcu_sysid_{1};     // 飞控 system id（从心跳学习）
    std::atomic<uint8_t> fcu_compid_{1};    // 飞控 component id（从心跳学习）
    std::atomic<uint8_t> locked_sysid_{0};  // 0 = 还没锁定
    std::atomic<uint8_t> locked_compid_{0};
    std::atomic<uint64_t> reconnects_{0};  // 串口重开成功或心跳超时次数
    std::atomic<uint8_t> autopilot_{0};    // 飞控固件类型（3=ArduPilot, 12=PX4, 0=未知）
    // 各消息累计接收帧数
    std::atomic<uint32_t> count_attitude_{0};
    std::atomic<uint32_t> count_local_position_{0};
    std::atomic<uint32_t> count_highres_imu_{0};
    std::atomic<uint32_t> count_battery_{0};
    std::atomic<uint32_t> count_system_time_{0};

    // 每种数据一把锁
    mutable std::mutex landed_mutex_;
    LandedState landed_;
    mutable std::mutex attitude_mutex_;
    mutable std::mutex battery_mutex_;
    mutable std::mutex local_position_mutex_;
    mutable std::mutex imu_mutex_;
    mutable std::mutex system_time_mutex_;
    Attitude attitude_;        // roll  pitch  yaw  rad   四元数
    Battery battery_;          // V  百分比剩余电量
    LocalPosition local_pos_;  // 速度与位置 m/s   m
    ImuData imu_;              // 角速度 rad/s  加速度m/s*s  时间戳us 由数量级判断是什么时间戳
    SystemTime system_time_;   // 时间戳  UNIX时间us   飞控开机时间ms
    std::atomic<int64_t> last_attitude_ns_{0};
    std::atomic<int64_t> last_local_position_ns_{0};
    std::atomic<int64_t> last_system_time_ns_{0};
    std::atomic<uint32_t> custom_mode_{0};             // 完整 custom_mode 整数值
    std::atomic<uint16_t> last_ack_command_{0};        // 最近一次 ACK 对应的命令号
    std::atomic<uint8_t> last_ack_result_{UINT8_MAX};  // 最近一次 ACK 结果（UINT8_MAX=未收到）

    // 【2026-10-28】解锁状态 + 心跳新鲜度
    std::atomic<bool> armed_{false};
    std::atomic<int64_t> last_heartbeat_ns_{0};  // steady_clock 纳秒，0 = 从未收到

    // 【2026-10-28】逐帧 IMU 队列（接收线程写，VIO 线程 pop）
    mutable std::mutex imu_queue_mutex_;
    std::deque<TimedImu> imu_queue_;
    std::atomic<size_t> imu_queue_capacity_{2000};  // 200 Hz 下约 10 秒
    std::atomic<uint64_t> dropped_imu_{0};

    std::mutex send_mutex_;  // 串口写锁
};

}  // namespace pure_uav
