// MAVLink 串口链路：给 VIO 提供 IMU（HIGHRES_IMU），并发送 VisionPose（ODOMETRY）。
//
// 说明：这是 ~/calib 独立工程的“试验链路”，用于不依赖 pure-uav communicate
// 模块就能在真机上跑通 VIO。它的协议行为与吴钦沅的 Communicator 保持一致：
//   - sysid=2, compid=1（和 Communicator 相同）
//   - ODOMETRY 使用 frame_id=LOCAL_FRD、child_frame_id=BODY_FRD、VIO estimator type
//   - 协方差首元素填 NaN（语义：未知）
//
// 正式并入仓库时，建议改用 Communicator + 给它加一个 IMU 回调（见
// patches/communicator_imu_callback.patch），而不是同时开两个串口。

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <thread>

#include "pure_uav/vio_types.hpp"
#include "pure_uav/vision_pose.hpp"

namespace pure_uav {

class MavlinkLink {
public:
    struct Config {
        std::string device = "/dev/ttyACM0";
        int baudrate = 921600;
        int highres_imu_interval_us = 5000;   // 200 Hz（5000us；改这个数时随手改注释，别留错）
        int system_time_interval_us = 1000000; // 1 Hz
        int heartbeat_interval_ms = 1000;      // 本机心跳
        bool send_heartbeat = true;
        size_t max_imu_queue = 2000;           // 200Hz 下约 10 秒
        // 【2026-10-29】目标飞控身份。0 = 自动锁定（首个"像飞控"的心跳）。
        // 锁定后只接受该 sysid/compid 的心跳；地面站/机载计算机/其他飞行器的心跳一律忽略。
        // 多机同链路时显式指定，避免锁错。
        uint8_t target_sysid = 0;
        uint8_t target_compid = 0;             // 0 = 不限定组件号
    };

    struct TimedImu {
        ImuSample imu;
        int64_t host_ns = 0;  // 收到该帧时的主机单调时钟
    };

    MavlinkLink() = default;
    ~MavlinkLink();

    MavlinkLink(const MavlinkLink&) = delete;
    MavlinkLink& operator=(const MavlinkLink&) = delete;

    bool start(const Config& cfg, std::string* error);
    void stop();

    bool running() const { return running_.load(); }
    bool connected() const { return connected_.load(); }

    // 【2026-10-25】飞控是否已解锁（HEARTBEAT.base_mode 的 SAFETY_ARMED 位）。
    // 用途：地面上（未解锁）允许开零速更新，空中（已解锁）绝不允许 —— 见 VioManager 的说明。
    // 没收到过心跳时返回 false（= 当成"地面"），这样纯手持/无飞控的场景行为不变。
    bool armed() const { return armed_.load(); }
    uint8_t fcuSysId() const { return fcu_sysid_.load(); }
    // 【2026-10-29】串口重连次数。每次"设备掉线后重新打开成功" +1。
    // vio_live 用它判定"新会话，时间基要重来"。
    uint64_t reconnectCount() const { return reconnects_.load(); }
    bool targetLocked() const { return locked_sysid_.load() != 0; }

    // 【2026-10-29】清空 IMU 队列（飞控重启/串口重连/时间基变化时用）。
    // 旧时间基的样本绝不能再喂给估计器。
    void clearImuQueue();

    // 取出最早的一帧 IMU；无数据返回 false。
    bool popImu(TimedImu& out);
    size_t imuQueueSize() const;
    uint64_t droppedImu() const { return dropped_imu_.load(); }
    uint64_t receivedImu() const { return imu_received_.load(); }
    uint64_t lastImuTimestampUs() const { return last_imu_us_.load(); }

    // 发送视觉位姿（MAVLink ODOMETRY）。未连接时返回 false。
    /// 【2026-10-22】发位姿。原来 pose_cov/vel_cov 填 NaN（未知）、quality=0、
    /// reset_counter=0 —— 飞控完全不知道数据可信度和"我们重置过"。
    /// 重建后位姿会跳变，不告诉飞控重置会导致它把跳变当成真实运动。
    /// @param quality        0-100 可信度（0 = 未知；本函数把越界值夹到 0..100）
    /// @param reset_counter  估计器重建次数（必须单调增长，飞控据此处理重锚定跳变）
    /// @param pos_sigma_m    位置标准差估计（米，<=0 表示未知 → 协方差填 NaN）
    /// @param vel_sigma_mps  速度标准差估计（m/s，<=0 表示未知 → 协方差填 NaN）
    /// 【2026-10-28】修掉字段顺序 bug。MAVLink 的 pack 签名最后三个参数是
    ///   reset_counter, estimator_type, quality
    /// 旧代码按 quality, estimator_type, reset_counter 传，结果：
    ///   想发"质量 90、重置 0" 实际发成"重置 90、质量 0"。
    /// 现场后果有两个：① 飞控看到的 reset_counter 随健康状态在 90/50 之间跳，
    /// 每跳一次都被当成"估计器重置"，触发位置/速度/姿态重锚定；
    /// ② 真正重建（reset_counter 1,2,3…）反而没告诉飞控，跳变被当成真实运动。
    /// 这条 bug 在 ArduPilot 的 VISP 日志里能直接看到（R 字段）。
    /// 【2026-10-23】飞控 EKF3 的速度估计（NED，m/s）。用作 VIO 的第二意见。
    struct Ekf3Velocity {
        bool valid = false;
        double vx = 0, vy = 0, vz = 0;
        uint64_t timestamp_us = 0;
        long long count = 0;
    };
    Ekf3Velocity ekf3Velocity() const;

    bool sendVisionPose(const VisionPose& pose, int quality = 0, uint8_t reset_counter = 0,
                        float pos_sigma_m = 0.0f, float vel_sigma_mps = 0.0f);

    // 把一帧 VisionPose 编码成一条 ODOMETRY 的**完整发送缓冲**（含 MAVLink 帧头与 CRC）。
    // 返回写入 buf 的字节数；buf 容量不足返回 0。sendVisionPose() 内部就走这条路径。
    //
    // 单独暴露的唯一理由：没有飞控、没有串口也能做「打包→解析→逐字段核对」的回归测试
    // （tests/test_odometry_pack.cpp）。改字段顺序这种 bug 只看发送返回值是发现不了的。
    static uint16_t encodeVisionPose(const VisionPose& pose, int quality, uint8_t reset_counter,
                                     float pos_sigma_m, float vel_sigma_mps, uint8_t sysid,
                                     uint8_t compid, uint8_t* buf, size_t buf_cap);

    // ---- 只读参数查询（PARAM_REQUEST_READ → PARAM_VALUE）----
    // 用途：确认飞控固件对 IMU 做了哪些预处理（低通/陷波/采样率）。
    // 这些值只读，不写回飞控。
    void requestParamByName(const std::string& name);
    // 已收到的参数（按名排序）
    std::vector<std::pair<std::string, float>> receivedParams() const;
    size_t receivedParamCount() const;

private:
    void recvLoop();
    void requestStreams();
    bool sendBytes(const uint8_t* buf, uint16_t len);
    bool writeAll(int fd, const uint8_t* buf, size_t len);
    void pushImu(const TimedImu& item);
    // 心跳是否来自目标飞控；首次匹配时锁定 sysid/compid
    bool acceptHeartbeat(uint8_t sysid, uint8_t compid, uint8_t autopilot, uint8_t type);
    void handleLinkLoss(const char* why);   // 关闭 fd、清状态/队列，等重连
    bool tryReopen(std::string* error);     // 尝试重新打开串口

    Config cfg_;
    int fd_ = -1;
    mutable std::mutex fd_mtx_;   // 保护 fd_ 的打开/关闭（发送线程 vs 重连线程）
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> armed_{false};
    std::atomic<uint8_t> fcu_sysid_{1};
    std::atomic<uint8_t> fcu_compid_{1};
    std::atomic<uint8_t> locked_sysid_{0};   // 0 = 还没锁定
    std::atomic<uint8_t> locked_compid_{0};
    std::atomic<uint64_t> reconnects_{0};
    std::atomic<uint64_t> last_imu_us_{0};
    std::atomic<uint64_t> imu_received_{0};
    std::atomic<uint64_t> dropped_imu_{0};

    mutable std::mutex q_mtx_;
    std::deque<TimedImu> imu_q_;

    mutable std::mutex param_mtx_;
    std::map<std::string, float> params_;

    // 【2026-10-23】飞控 EKF3 速度（第二意见，用来和 VIO 速度对比抓发散）
    mutable std::mutex ekf3_mtx_;
    double ekf3_vx_ = 0.0, ekf3_vy_ = 0.0, ekf3_vz_ = 0.0;
    uint64_t ekf3_time_us_ = 0;
    bool ekf3_valid_ = false;
    long long ekf3_count_ = 0;

    std::mutex send_mtx_;
    uint8_t sysid_ = 2;
    uint8_t compid_ = 1;

    // 仅 recvLoop 线程访问
    bool streams_requested_ = false;
    int64_t last_stream_request_ns_ = 0;
    int64_t last_heartbeat_recv_ns_ = 0;
    int64_t last_heartbeat_send_ns_ = 0;
};

}  // namespace pure_uav
