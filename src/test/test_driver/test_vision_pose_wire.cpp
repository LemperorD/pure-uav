// Communication 模块的"线上联调"回归测试：不需要飞控，用伪终端(PTY)当飞控。
//
// 它验证的是 VIO↔通信↔飞控 这条链上最容易出错、又最不容易在编译期发现的三件事：
//   1) pushVisionPose() 真的把 quality / reset_counter / 协方差写进了 ODOMETRY；
//   2) 逐帧 IMU 队列（popImu）按到达顺序交付，溢出计数不撒谎；
//   3) 心跳里的解锁位能被 isArmed() 读到（VIO 地面零速更新依赖它）。
//
// 做法：
//   主端(master) 扮演飞控：发 HEARTBEAT / HIGHRES_IMU；
//   从端(slave) 交给 Communicator，就像打开 /dev/ttyACM0 一样。
//   然后从 master 把 Communicator 发出来的字节解析回来逐字段核对。
//
// 编译/运行：见 src/test/CMakeLists.txt 的 test_vision_pose_wire
//   ./bin/test/test_vision_pose_wire

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "common/mavlink.h"
#include "communicator.hpp"

namespace {

int g_failed = 0;

int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) {                                                 \
            std::printf("  ❌ 第 %d 行不成立: %s\n", __LINE__, #cond); \
            ++g_failed;                                                \
        }                                                              \
    } while (0)

bool writeAll(int fd, const uint8_t* buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        const ssize_t n = ::write(fd, buf + off, len - off);
        if (n > 0) {
            off += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

bool sendPacked(int fd, const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return writeAll(fd, buf, len);
}

// master 端消息读取器。
//
// 注意：一次 read() 可能带回**多条**消息（本机心跳 + 消息流请求 + ODOMETRY 常常
// 挤在同一个 TCP/PTY 读里）。早先的写法只返回第一条、把同一次 read 里剩下的
// 消息直接丢了 —— 于是"偶尔读不到 ODOMETRY"，其实是测试自己的 bug。
// 这里用一个 pending 队列把所有解析出来的消息都留着，逐条交付。
struct MsgReader {
    int fd = -1;
    uint8_t chan = MAVLINK_COMM_1;
    std::deque<mavlink_message_t> pending;

    bool next(mavlink_message_t& out, int timeout_ms) {
        if (!pending.empty()) {
            out = pending.front();
            pending.pop_front();
            return true;
        }
        mavlink_status_t status{};
        uint8_t buf[1024];
        const int64_t deadline = nowMs() + timeout_ms;
        while (true) {
            const int64_t now = nowMs();
            if (now >= deadline) return false;

            struct pollfd pfd {};
            pfd.fd = fd;
            pfd.events = POLLIN;
            const int pr = ::poll(&pfd, 1, static_cast<int>(deadline - now));
            if (pr <= 0) return false;

            const ssize_t n = ::read(fd, buf, sizeof(buf));
            if (n <= 0) continue;
            for (ssize_t i = 0; i < n; ++i) {
                mavlink_message_t msg{};
                if (mavlink_parse_char(chan, buf[i], &msg, &status)) {
                    pending.push_back(msg);
                }
            }
            if (!pending.empty()) {
                out = pending.front();
                pending.pop_front();
                return true;
            }
        }
    }
};

// 未确认目标飞控时只发本机心跳，不能向默认/上次地址提前订阅。
bool noStreamRequests(MsgReader& reader, int timeout_ms = 120) {
    const int64_t deadline = nowMs() + timeout_ms;
    mavlink_message_t msg{};
    while (nowMs() < deadline && reader.next(msg, static_cast<int>(deadline - nowMs()))) {
        if (msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG) continue;
        mavlink_command_long_t cmd{};
        mavlink_msg_command_long_decode(&msg, &cmd);
        if (cmd.command == MAV_CMD_SET_MESSAGE_INTERVAL) return false;
    }
    return true;
}

// 按线上字节检查全部五种订阅，包含目标组件和请求间隔。
bool readStreamRequests(MsgReader& reader, uint8_t sysid, uint8_t compid, const pure_uav::CommConfig& cfg) {
    const uint32_t ids[] = {MAVLINK_MSG_ID_ATTITUDE, MAVLINK_MSG_ID_LOCAL_POSITION_NED, MAVLINK_MSG_ID_HIGHRES_IMU, MAVLINK_MSG_ID_BATTERY_STATUS, MAVLINK_MSG_ID_SYSTEM_TIME};
    const int intervals[] = {cfg.attitude_interval_us, cfg.local_position_interval_us, cfg.highres_imu_interval_us, cfg.battery_interval_us, cfg.system_time_interval_us};
    bool seen[5] = {};
    size_t count = 0;
    const int64_t deadline = nowMs() + 1200;
    while (count < 5 && nowMs() < deadline) {
        mavlink_message_t msg{};
        if (!reader.next(msg, static_cast<int>(deadline - nowMs()))) break;
        if (msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG) continue;
        mavlink_command_long_t cmd{};
        mavlink_msg_command_long_decode(&msg, &cmd);
        if (cmd.command != MAV_CMD_SET_MESSAGE_INTERVAL) continue;
        if (cmd.target_system != sysid || cmd.target_component != compid) return false;
        bool matched = false;
        for (size_t i = 0; i < 5; ++i) {
            if (cmd.param1 != static_cast<float>(ids[i])) continue;
            if (seen[i] || cmd.param2 != static_cast<float>(intervals[i])) return false;
            seen[i] = true;
            ++count;
            matched = true;
            break;
        }
        if (!matched) return false;
    }
    return count == 5;
}

// 等 master 端出现第一条 ODOMETRY（中间可能夹着本机心跳 / 消息流请求）
bool readOdometry(MsgReader& reader, mavlink_odometry_t& odo, int timeout_ms) {
    const int64_t deadline = nowMs() + timeout_ms;
    while (true) {
        mavlink_message_t msg{};
        const int64_t now = nowMs();
        if (now >= deadline) return false;
        if (!reader.next(msg, static_cast<int>(deadline - now))) return false;
        if (msg.msgid == MAVLINK_MSG_ID_ODOMETRY) {
            mavlink_msg_odometry_decode(&msg, &odo);
            return true;
        }
    }
}

template <typename Pred>
bool waitMs(Pred pred, int timeout_ms) {
    const auto deadline = nowMs() + timeout_ms;
    while (nowMs() < deadline) {
        if (pred()) return true;
        ::usleep(10000);
    }
    return pred();
}

bool sendHeartbeat(int fd, uint8_t sysid, uint8_t compid, uint8_t type, uint8_t autopilot, uint8_t base_mode) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(sysid, compid, &msg, type, autopilot, base_mode, 0, MAV_STATE_ACTIVE);
    return sendPacked(fd, msg);
}

bool sendHighresImu(int fd, uint8_t sysid, uint64_t t_us, float ax) {
    mavlink_message_t msg;
    mavlink_msg_highres_imu_pack(sysid, 1, &msg, t_us, ax, 0.0f, 9.8f, 0.0f, 0.0f, 0.0f, NAN, NAN, NAN, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    return sendPacked(fd, msg);
}

// 创建一对 PTY，返回 master fd；slave 路径写入 out。
bool makePty(int& master, std::string& slave_path) {
    master = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0) return false;
    if (::grantpt(master) != 0 || ::unlockpt(master) != 0) return false;
    const char* name = ::ptsname(master);
    if (name == nullptr) return false;
    slave_path = name;
    // master 端也设成非阻塞，readMsg 用 poll 控制阻塞。
    const int flags = ::fcntl(master, F_GETFL, 0);
    ::fcntl(master, F_SETFL, flags | O_NONBLOCK);
    return true;
}

}  // namespace

int main() {
    std::printf("===== Communicator 线上回归测试（PTY 模拟飞控）=====\n");

    int master = -1;
    std::string slave_path;
    if (!makePty(master, slave_path)) {
        std::printf("  ❌ 创建 PTY 失败\n");
        return 1;
    }
    std::printf("[准备] 模拟飞控口: %s\n", slave_path.c_str());

    MsgReader reader;
    reader.fd = master;

    pure_uav::Communicator comm;
    comm.setSystemId(2);
    comm.setComponentId(1);
    if (!comm.start(slave_path, 921600)) {
        std::printf("  ❌ Communicator 打不开 PTY 从端\n");
        return 1;
    }

    // ---- 1) 心跳：解锁位 ----
    {
        mavlink_message_t hb;
        mavlink_msg_heartbeat_pack(1, 1, &hb, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE);
        CHECK(sendPacked(master, hb));
        for (int i = 0; i < 300 && !(comm.isConnected() && comm.isArmed()); ++i) {
            ::usleep(10000);
        }
        CHECK(comm.isConnected());
        CHECK(comm.isArmed());
        const int64_t age = comm.lastHeartbeatAgeMs();
        CHECK(age >= 0 && age < 2000);
        std::printf("[1] 心跳 OK: armed=%d age=%lldms\n", static_cast<int>(comm.isArmed()), static_cast<long long>(age));
    }

    // ---- 1b) 心跳来源筛选：地面站/其他飞行器不得覆盖飞控状态 ----
    {
        CHECK(sendHeartbeat(master, 255, 190, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0));
        CHECK(sendHeartbeat(master, 255, 190, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0));
        ::usleep(200000);
        CHECK(comm.isArmed());  // 关键：不能被地面站的"未解锁"覆盖
        CHECK(comm.isConnected());
        CHECK(comm.getAutopilot() == MAV_AUTOPILOT_ARDUPILOTMEGA);  // 固件类型不被覆盖
        CHECK(comm.lastHeartbeatAgeMs() >= 0 && comm.lastHeartbeatAgeMs() < 2000);

        // 已锁定后，其他飞行器(sysid=42)的心跳也必须忽略
        CHECK(sendHeartbeat(master, 42, 1, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, 0));
        ::usleep(150000);
        CHECK(comm.isArmed());
        CHECK(comm.getAutopilot() == MAV_AUTOPILOT_ARDUPILOTMEGA);

        // 目标飞控自己发的未解锁心跳必须生效
        CHECK(sendHeartbeat(master, 1, 1, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, 0));
        for (int i = 0; i < 100 && comm.isArmed(); ++i) ::usleep(10000);
        CHECK(!comm.isArmed());
        CHECK(comm.isConnected());
        std::printf("[2] 心跳来源筛选 OK（地面站/其他飞行器忽略，飞控自己生效）\n");
    }

    // ---- 2) 逐帧 IMU：顺序 + 主机时间戳 + 溢出计数 ----
    {
        auto sendImu = [&](uint64_t t_us, float ax) {
            mavlink_message_t msg;
            mavlink_msg_highres_imu_pack(1, 1, &msg, t_us, ax, 0.0f, 9.8f,  // xacc yacc zacc（机体系 FRD，静止时 z=+g）
                                         0.0f, 0.0f, 0.0f,                  // xgyro ygyro zgyro
                                         NAN, NAN, NAN,                     // 磁力计（未用）
                                         0.0f, 0.0f, 0.0f, 0.0f,            // 气压/温度
                                         0, 0);                             // fields_updated, id
            CHECK(sendPacked(master, msg));
        };
        sendImu(1000000ULL, 1.0f);
        sendImu(1005000ULL, 2.0f);
        sendImu(1010000ULL, 3.0f);

        for (int i = 0; i < 300 && comm.getCountImu() < 3; ++i) ::usleep(10000);
        CHECK(comm.getCountImu() >= 3);
        // 队列里可能还留着，全部取出核对顺序
        std::vector<pure_uav::Communicator::TimedImu> got;
        pure_uav::Communicator::TimedImu item;
        while (comm.popImu(item)) got.push_back(item);
        CHECK(got.size() >= 3);
        CHECK(got[0].imu.timestamp_us == 1000000ULL);
        CHECK(got[1].imu.timestamp_us == 1005000ULL);
        CHECK(got[2].imu.timestamp_us == 1010000ULL);
        CHECK(std::fabs(got[0].imu.ax - 1.0f) < 1e-6f);
        CHECK(std::fabs(got[2].imu.ax - 3.0f) < 1e-6f);
        CHECK(got[0].host_ns > 0);
        CHECK(got[1].host_ns >= got[0].host_ns);
        CHECK(got[2].host_ns >= got[1].host_ns);
        std::printf("[2] 逐帧 IMU OK: %zu 帧，时间戳/主机时钟单调\n", got.size());

        // 容量 2，塞 3 帧 → 丢 1 帧，不能静默丢
        comm.setImuQueueCapacity(2);
        const uint64_t dropped_before = comm.droppedImu();
        sendImu(2000000ULL, 4.0f);
        sendImu(2005000ULL, 5.0f);
        sendImu(2010000ULL, 6.0f);
        for (int i = 0; i < 300 && comm.getCountImu() < 6; ++i) ::usleep(10000);
        ::usleep(20000);
        CHECK(comm.imuQueueSize() == 2);
        CHECK(comm.droppedImu() == dropped_before + 1);
        std::printf("[3] 队列溢出 OK: size=%zu dropped=%llu\n", comm.imuQueueSize(), static_cast<unsigned long long>(comm.droppedImu()));
        while (comm.popImu(item)) {
        }
        comm.setImuQueueCapacity(2000);
    }

    // ---- 3) ODOMETRY：质量 / 重置 / 协方差 ----
    {
        pure_uav::VisionPose p;
        p.timestamp_us = 1020000ULL;
        p.frame_id = pure_uav::VisionFrame::LOCAL_FRD;
        p.child_frame_id = pure_uav::VisionFrame::BODY_FRD;
        p.x = 0.75f;
        p.y = -0.25f;
        p.z = -1.5f;          // 向上 1.5 m（FRD: 上为负）
        p.q[0] = 0.9238795f;  // yaw=45°
        p.q[3] = 0.3826834f;
        p.vx = 0.1f;
        p.vy = 0.2f;
        p.vz = -0.3f;
        p.quality = 90;
        p.reset_counter = 7;
        p.pos_sigma_m = 0.15f;
        p.vel_sigma_mps = 0.25f;

        auto stale_pose = p;
        stale_pose.timestamp_us = 1;
        stale_pose.x = -999;
        CHECK(!comm.pushVisionPoseForSession(stale_pose, comm.reconnectCount() + 1));
        CHECK(comm.pushVisionPoseForSession(p, comm.reconnectCount()));

        mavlink_odometry_t odo{};
        CHECK(readOdometry(reader, odo, 3000));
        CHECK(odo.time_usec == p.timestamp_us);
        CHECK(odo.frame_id == 20);
        CHECK(odo.child_frame_id == 12);
        CHECK(std::fabs(odo.x - 0.75f) < 1e-6f);
        CHECK(std::fabs(odo.z + 1.5f) < 1e-6f);
        CHECK(std::fabs(odo.q[3] - 0.3826834f) < 1e-5f);
        // ★ 联调核心：这三项以前是写死的 0
        CHECK(odo.quality == 90);
        CHECK(odo.reset_counter == 7);
        CHECK(odo.estimator_type == MAV_ESTIMATOR_TYPE_VIO);
        CHECK(std::fabs(odo.pose_covariance[0] - 0.15f * 0.15f) < 1e-7f);
        CHECK(std::fabs(odo.velocity_covariance[11] - 0.25f * 0.25f) < 1e-7f);
        std::printf("[4] ODOMETRY OK: quality=%d reset=%u type=%u pos_cov0=%.4f\n", static_cast<int>(odo.quality), static_cast<unsigned>(odo.reset_counter), static_cast<unsigned>(odo.estimator_type), static_cast<double>(odo.pose_covariance[0]));

        // 未知 sigma → NaN（未知），不能是 0（完美已知）
        pure_uav::VisionPose p2;
        p2.timestamp_us = 1025000ULL;
        p2.quality = 50;
        CHECK(comm.pushVisionPose(p2));
        mavlink_odometry_t odo2{};
        CHECK(readOdometry(reader, odo2, 3000));
        CHECK(std::isnan(odo2.pose_covariance[0]));
        CHECK(std::isnan(odo2.velocity_covariance[0]));
        CHECK(odo2.quality == 50);
        std::printf("[5] 未知协方差 OK: NaN，quality=50\n");
    }

    // ---- 6) 停止 → 重新启动：旧 IMU/旧心跳状态不得跨会话 ----
    {
        const uint32_t imu_before = comm.getCountImu();  // 计数是累计的，不能和 1 比
        CHECK(sendHighresImu(master, 1, 777777ULL, 1.0f));
        for (int i = 0; i < 300 && comm.getCountImu() == imu_before; ++i) ::usleep(10000);
        CHECK(comm.getCountImu() > imu_before);
        CHECK(comm.imuQueueSize() >= 1);

        comm.stop();
        CHECK(comm.imuQueueSize() == 0);         // stop 必须清空队列
        CHECK(!comm.isArmed());                  // 旧解锁状态失效
        CHECK(comm.lastHeartbeatAgeMs() == -1);  // 旧心跳失效
        CHECK(comm.getCountImu() == 0);          // 会话计数复位

        CHECK(comm.start(slave_path, 921600));
        pure_uav::Communicator::TimedImu stale;
        CHECK(!comm.popImu(stale));  // 关键：重启后不能吐出上一次的帧
        CHECK(!comm.isArmed());
        CHECK(comm.lastHeartbeatAgeMs() == -1);

        // 新会话仍然能正常收数据
        CHECK(sendHeartbeat(master, 1, 1, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_FLAG_SAFETY_ARMED));
        for (int i = 0; i < 300 && !comm.isArmed(); ++i) ::usleep(10000);
        CHECK(comm.isArmed());
        CHECK(sendHighresImu(master, 1, 888888ULL, 2.0f));
        for (int i = 0; i < 300 && comm.getCountImu() < 1; ++i) ::usleep(10000);
        pure_uav::Communicator::TimedImu fresh;
        CHECK(comm.popImu(fresh));
        CHECK(fresh.imu.timestamp_us == 888888ULL);
        std::printf("[6] 停止→重启会话清理 OK（旧帧不可见，新会话正常）\n");
    }

    // ---- 7) 串口拔出→重新枚举：自动重连 + 清旧队列 + 重新锁定身份 ----
    {
        int master_a = -1;
        std::string slave_a;
        CHECK(makePty(master_a, slave_a));
        const std::string link_path = "/tmp/pure_uav_comm_link_" + std::to_string(::getpid());
        ::unlink(link_path.c_str());
        CHECK(::symlink(slave_a.c_str(), link_path.c_str()) == 0);

        pure_uav::Communicator comm2;
        comm2.setSystemId(2);
        comm2.setComponentId(1);
        CHECK(comm2.start(link_path, 921600));

        CHECK(sendHeartbeat(master_a, 1, 1, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_FLAG_SAFETY_ARMED));
        CHECK(waitMs([&] { return comm2.isConnected() && comm2.isArmed(); }, 2000));
        const uint32_t cnt_before = comm2.getCountImu();
        CHECK(sendHighresImu(master_a, 1, 500000ULL, 1.0f));
        CHECK(waitMs([&] { return comm2.getCountImu() > cnt_before; }, 2000));

        // 模拟设备拔出
        ::close(master_a);
        CHECK(waitMs([&] { return !comm2.isConnected(); }, 3000));

        // 模拟重新枚举：新 PTY + 稳定路径指过去
        int master_b = -1;
        std::string slave_b;
        CHECK(makePty(master_b, slave_b));
        ::unlink(link_path.c_str());
        CHECK(::symlink(slave_b.c_str(), link_path.c_str()) == 0);

        CHECK(waitMs([&] { return comm2.reconnectCount() >= 1; }, 6000));
        CHECK(comm2.imuQueueSize() == 0);  // 旧时间基的帧必须清掉

        MsgReader reconnect_reader;
        reconnect_reader.fd = master_b;
        CHECK(noStreamRequests(reconnect_reader));

        // 重连后重新学习身份：故意换 sysid=9，必须重新向它订阅数据流。
        CHECK(sendHeartbeat(master_b, 9, 1, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_FLAG_SAFETY_ARMED));
        CHECK(waitMs([&] { return comm2.isConnected() && comm2.isArmed(); }, 3000));
        CHECK(comm2.getAutopilot() == MAV_AUTOPILOT_ARDUPILOTMEGA);
        CHECK(readStreamRequests(reconnect_reader, 9, 1, pure_uav::CommConfig{}));
        const uint32_t cnt2 = comm2.getCountImu();
        CHECK(sendHighresImu(master_b, 9, 600000ULL, 2.0f));
        CHECK(waitMs([&] { return comm2.getCountImu() > cnt2; }, 2000));
        pure_uav::Communicator::TimedImu fresh2;
        CHECK(comm2.popImu(fresh2));
        CHECK(fresh2.imu.timestamp_us == 600000ULL);
        std::printf("[7] 串口拔出→自动重连 OK（清旧队列、重新锁定 sysid=9）\n");

        comm2.stop();
        ::unlink(link_path.c_str());
        ::close(master_b);
    }

    // ---- 8) 首次连接非默认飞控/组件：请求目标和间隔必须来自已确认身份 ----
    {
        int target_master = -1;
        std::string target_slave;
        if (!makePty(target_master, target_slave)) return 1;
        MsgReader target_reader;
        target_reader.fd = target_master;
        pure_uav::CommConfig cfg;
        cfg.device = target_slave;
        cfg.fcu_sysid = 9;
        cfg.fcu_compid = 42;
        cfg.highres_imu_interval_us = 5000;
        pure_uav::Communicator target;
        if (!target.start(cfg)) return 1;
        CHECK(noStreamRequests(target_reader));
        CHECK(sendHeartbeat(target_master, 9, 42, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, 0));
        CHECK(waitMs([&] { return target.isConnected(); }, 2000));
        CHECK(readStreamRequests(target_reader, 9, 42, cfg));
        std::printf("[8] 首次连接非默认飞控 OK（订阅目标 9/42，IMU 间隔 5000 us）\n");
        target.stop();
        ::close(target_master);
    }

    comm.stop();
    ::close(master);

    if (g_failed == 0) {
        std::printf("PASS: 心跳筛选/逐帧 IMU/ODOMETRY/会话清理/串口重连 全部通过\n");
        return 0;
    }
    std::printf("FAILED: %d 处不通过\n", g_failed);
    return 1;
}
