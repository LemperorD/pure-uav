// MavlinkLink 的"线上"回归测试：用伪终端(PTY)模拟一条飞控链路，不需要真机。
//
// 覆盖 2026-10-29 验收指出的两个问题：
//   1) 地面站/其他飞行器的心跳会覆盖飞控身份与解锁状态 —— 现在必须按 sysid/compid 锁定，
//      只有目标飞控的心跳能更新连接/解锁状态；
//   2) IMU 队列在时间基变化/重连时必须能整队清空（clearImuQueue）。
//
// 不覆盖"串口拔出后重新打开"——PTY 没有设备重新枚举的过程，那条路径只能在真机/
// USB 上验证（见联调方案第 6 节的重启/重连用例）。
//
// 运行：./bin/test_mavlink_link_wire

#define _GNU_SOURCE
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "common/mavlink.h"

#include "mavlink_link.hpp"

namespace {

int g_failed = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::printf("  ❌ 第 %d 行不成立: %s\n", __LINE__, #cond);    \
            ++g_failed;                                                  \
        }                                                                \
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

bool sendMsg(int fd, const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return writeAll(fd, buf, len);
}

bool sendHeartbeat(int fd, uint8_t sysid, uint8_t compid, uint8_t type, uint8_t autopilot,
                   uint8_t base_mode) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(sysid, compid, &msg, type, autopilot, base_mode, 0,
                               MAV_STATE_ACTIVE);
    return sendMsg(fd, msg);
}

bool sendImu(int fd, uint8_t sysid, uint64_t t_us, float ax) {
    mavlink_message_t msg;
    mavlink_msg_highres_imu_pack(
        sysid, 1, &msg, t_us,
        ax, 0.0f, 9.8f,                 // xacc yacc zacc
        0.0f, 0.0f, 0.0f,               // gyro
        NAN, NAN, NAN,                  // mag
        0.0f, 0.0f, 0.0f, 0.0f,         // pressure/temperature
        0, 0);
    return sendMsg(fd, msg);
}

bool makePty(int& master, std::string& slave_path) {
    master = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0) return false;
    if (::grantpt(master) != 0 || ::unlockpt(master) != 0) return false;
    const char* name = ::ptsname(master);
    if (name == nullptr) return false;
    slave_path = name;
    const int flags = ::fcntl(master, F_GETFL, 0);
    ::fcntl(master, F_SETFL, flags | O_NONBLOCK);
    return true;
}

template <typename Pred>
bool waitFor(Pred pred, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return pred();
}

}  // namespace

int main() {
    std::printf("===== MavlinkLink 心跳筛选/队列回归测试（PTY 模拟飞控）=====\n");

    int master = -1;
    std::string slave_path;
    if (!makePty(master, slave_path)) {
        std::printf("  ❌ 创建 PTY 失败\n");
        return 1;
    }
    std::printf("[准备] 模拟飞控口: %s\n", slave_path.c_str());

    pure_uav::MavlinkLink link;
    pure_uav::MavlinkLink::Config cfg;
    cfg.device = slave_path;
    cfg.baudrate = 921600;
    cfg.max_imu_queue = 16;
    std::string err;
    if (!link.start(cfg, &err)) {
        std::printf("  ❌ MavlinkLink 打不开 PTY 从端: %s\n", err.c_str());
        return 1;
    }

    // ---- 1) 目标飞控心跳：锁定 + 解锁 ----
    CHECK(sendHeartbeat(master, /*sysid=*/1, /*compid=*/1, MAV_TYPE_QUADROTOR,
                        MAV_AUTOPILOT_ARDUPILOTMEGA,
                        MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
    CHECK(waitFor([&] { return link.connected() && link.armed() && link.fcuSysId() == 1; },
                  2000));
    CHECK(link.targetLocked());
    std::printf("[1] 飞控心跳 OK: connected=%d armed=%d sysid=%u\n",
                static_cast<int>(link.connected()), static_cast<int>(link.armed()),
                static_cast<unsigned>(link.fcuSysId()));

    // ---- 2) 地面站心跳（sysid=255 / type=GCS / autopilot=INVALID）不得覆盖状态 ----
    CHECK(sendHeartbeat(master, /*sysid=*/255, /*compid=*/190, MAV_TYPE_GCS,
                        MAV_AUTOPILOT_INVALID, /*base_mode=*/0));
    CHECK(sendHeartbeat(master, /*sysid=*/255, /*compid=*/190, MAV_TYPE_GCS,
                        MAV_AUTOPILOT_INVALID, /*base_mode=*/0));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    CHECK(link.armed());          // 关键：不能被地面站的"未解锁"覆盖
    CHECK(link.connected());
    CHECK(link.fcuSysId() == 1);  // 关键：sysid 不能被改成 255
    std::printf("[2] 地面站心跳被忽略 OK: armed=%d sysid=%u\n",
                static_cast<int>(link.armed()), static_cast<unsigned>(link.fcuSysId()));

    // ---- 3) 已锁定后，其他飞行器(sysid=42)的心跳也不得改状态 ----
    CHECK(sendHeartbeat(master, /*sysid=*/42, /*compid=*/1, MAV_TYPE_QUADROTOR,
                        MAV_AUTOPILOT_ARDUPILOTMEGA, /*base_mode=*/0));
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    CHECK(link.armed());
    CHECK(link.fcuSysId() == 1);
    std::printf("[3] 其他飞行器心跳被忽略 OK\n");

    // ---- 4) 目标飞控自己发未解锁心跳：必须生效 ----
    CHECK(sendHeartbeat(master, /*sysid=*/1, /*compid=*/1, MAV_TYPE_QUADROTOR,
                        MAV_AUTOPILOT_ARDUPILOTMEGA, /*base_mode=*/0));
    CHECK(waitFor([&] { return !link.armed(); }, 1000));
    CHECK(link.connected());
    std::printf("[4] 飞控未解锁心跳生效 OK\n");

    // ---- 5) 逐帧 IMU 顺序 + clearImuQueue ----
    CHECK(sendImu(master, 1, 1000000ULL, 1.0f));
    CHECK(sendImu(master, 1, 1005000ULL, 2.0f));
    CHECK(sendImu(master, 1, 1010000ULL, 3.0f));
    CHECK(waitFor([&] { return link.receivedImu() >= 3; }, 2000));

    pure_uav::MavlinkLink::TimedImu item;
    CHECK(link.popImu(item));
    CHECK(item.imu.timestamp_us == 1000000ULL);
    CHECK(link.popImu(item));
    CHECK(item.imu.timestamp_us == 1005000ULL);
    CHECK(link.popImu(item));
    CHECK(item.imu.timestamp_us == 1010000ULL);
    CHECK(item.host_ns > 0);

    CHECK(sendImu(master, 1, 1015000ULL, 4.0f));
    CHECK(waitFor([&] { return link.imuQueueSize() >= 1; }, 2000));
    link.clearImuQueue();
    CHECK(link.imuQueueSize() == 0);
    CHECK(!link.popImu(item));
    std::printf("[5] IMU 顺序 + 整队清空 OK\n");

    // 非目标 sysid 的 IMU 不应入队（锁定后只收飞控数据流）
    CHECK(sendImu(master, /*sysid=*/42, 2000000ULL, 7.0f));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    CHECK(link.imuQueueSize() == 0);
    std::printf("[6] 其他 sysid 的 IMU 被过滤 OK\n");

    // ---- 7) 串口拔出→重新枚举：自动重连 + 清旧队列 + 重新锁定身份 ----
    // 用软链接模拟稳定的设备名（真实场景一般是 /dev/serial/by-id/... 或 udev 别名）；
    // 换 PTY 就等价于"设备重新枚举后路径不变"。
    {
        int master_a = -1;
        std::string slave_a;
        CHECK(makePty(master_a, slave_a));
        const std::string link_path = "/tmp/pure_uav_fc_link_" + std::to_string(::getpid());
        ::unlink(link_path.c_str());
        CHECK(::symlink(slave_a.c_str(), link_path.c_str()) == 0);

        pure_uav::MavlinkLink link2;
        pure_uav::MavlinkLink::Config cfg2;
        cfg2.device = link_path;
        cfg2.baudrate = 921600;
        cfg2.max_imu_queue = 16;
        std::string err2;
        CHECK(link2.start(cfg2, &err2));

        CHECK(sendHeartbeat(master_a, 1, 1, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                            MAV_MODE_FLAG_SAFETY_ARMED));
        CHECK(waitFor([&] { return link2.connected() && link2.armed(); }, 2000));
        const uint64_t rx_before = link2.receivedImu();
        CHECK(sendImu(master_a, 1, 3000000ULL, 1.0f));
        CHECK(waitFor([&] { return link2.receivedImu() > rx_before; }, 2000));
        CHECK(link2.imuQueueSize() >= 1);   // 故意不取走，留给断线清理检查

        // 模拟设备拔出
        ::close(master_a);
        CHECK(waitFor([&] { return !link2.connected(); }, 2000));

        // 模拟重新枚举：新 PTY + 把稳定路径指过去
        int master_b = -1;
        std::string slave_b;
        CHECK(makePty(master_b, slave_b));
        ::unlink(link_path.c_str());
        CHECK(::symlink(slave_b.c_str(), link_path.c_str()) == 0);

        CHECK(waitFor([&] { return link2.reconnectCount() >= 1; }, 6000));
        CHECK(link2.imuQueueSize() == 0);   // 旧时间基的帧必须清掉

        // 重连后重新学习身份：故意把 sysid 换成 7，证明锁定被重置
        CHECK(sendHeartbeat(master_b, 7, 1, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                            MAV_MODE_FLAG_SAFETY_ARMED));
        CHECK(waitFor([&] { return link2.connected() && link2.armed() && link2.fcuSysId() == 7; },
                      3000));
        const uint64_t rx2 = link2.receivedImu();
        CHECK(sendImu(master_b, 7, 4000000ULL, 2.0f));
        CHECK(waitFor([&] { return link2.receivedImu() > rx2; }, 2000));
        pure_uav::MavlinkLink::TimedImu reconnected;
        CHECK(link2.popImu(reconnected));
        CHECK(reconnected.imu.timestamp_us == 4000000ULL);
        std::printf("[7] 串口拔出→自动重连 OK（清旧队列、重新锁定 sysid=7）\n");

        link2.stop();
        ::unlink(link_path.c_str());
        ::close(master_b);
    }

    link.stop();
    ::close(master);

    if (g_failed == 0) {
        std::printf("PASS: 心跳筛选、解锁状态、IMU 队列、串口重连全部正确\n");
        return 0;
    }
    std::printf("FAILED: %d 处不通过\n", g_failed);
    return 1;
}
