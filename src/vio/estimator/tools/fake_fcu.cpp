// 假飞控（PTY）：没有飞机时，也能把"上位机 → 飞控"这段链路完整跑一遍。
//
// 它创建一个伪终端，打印从端路径（并可选建一个稳定软链接），然后：
//   · 1 Hz 发 HEARTBEAT（sysid=1 / compid=1 / ArduPilot / 可指定解锁位）；
//   · 200 Hz 发 HIGHRES_IMU（time_usec 每帧 +5000 us，模拟飞控 boot 时间）；
//   · 解析上位机发来的 ODOMETRY，统计帧数/频率/质量/reset/坐标系/时间戳，
//     Ctrl-C 或 --duration 到点后打印汇总。
//
// 配套用法（终端 1 跑假飞控，终端 2 跑发送器）：
//   ./bin/fake_fcu --link /tmp/fc_sim --duration 35
//   ./bin/odometry_sender --serial /tmp/fc_sim --rate 30
//
// 真飞机接上后，同样的 ODOMETRY 字段应该和这里的汇总一致；这一步能把
// "上位机发送成功" 和 "飞控真的融合" 分开验证。

#define _GNU_SOURCE
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "common/mavlink.h"

#include "odom_tool_core.hpp"

namespace {

std::atomic<bool> g_stop{false};
void onSigint(int) { g_stop.store(true); }

int64_t steadyNowNs() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

void usage(const char* argv0) {
    std::printf(
        "用法: %s [选项]\n"
        "\n"
        "  --link PATH      给 PTY 从端建一个稳定软链接（推荐，等价于 udev 固定路径）\n"
        "  --duration S     运行 S 秒后退出（默认一直跑）\n"
        "  --disarmed       发未解锁心跳（默认解锁）\n"
        "  --verbose        逐帧打印收到的 ODOMETRY\n"
        "  --save-raw PATH  把收到的原始字节存下来（给 odometry_probe --file 复核）\n"
        "  -h, --help\n",
        argv0);
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

}  // namespace

int main(int argc, char** argv) {
    std::string link_path;
    double duration = 0.0;
    bool armed = true;
    bool verbose = false;
    std::string save_raw;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* what) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s 需要一个参数\n", what);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") {
            usage(argv[0]);
            return 0;
        } else if (a == "--link") {
            link_path = need("--link");
        } else if (a == "--duration") {
            duration = std::stod(need("--duration"));
        } else if (a == "--disarmed") {
            armed = false;
        } else if (a == "--verbose") {
            verbose = true;
        } else if (a == "--save-raw") {
            save_raw = need("--save-raw");
        } else {
            std::fprintf(stderr, "未知参数: %s\n", a.c_str());
            usage(argv[0]);
            return 2;
        }
    }

    std::signal(SIGINT, onSigint);
    std::signal(SIGTERM, onSigint);

    int master = -1;
    std::string slave;
    if (!makePty(master, slave) || slave.empty()) {
        std::fprintf(stderr, "创建 PTY 失败\n");
        return 1;
    }
    std::string device = slave;
    if (!link_path.empty()) {
        ::unlink(link_path.c_str());
        if (::symlink(slave.c_str(), link_path.c_str()) != 0) {
            std::fprintf(stderr, "创建软链接失败: %s\n", link_path.c_str());
            return 1;
        }
        device = link_path;
    }
    std::printf("[fake_fcu] 模拟飞控串口: %s\n", device.c_str());
    std::printf("[fake_fcu] 心跳: sysid=1 compid=1 %s；IMU: 200 Hz\n",
                armed ? "已解锁" : "未解锁");
    std::printf("[fake_fcu] 上位机端请跑：odometry_sender --serial %s --rate 30\n", device.c_str());
    std::fflush(stdout);

    const int64_t t0_ns = steadyNowNs();
    uint64_t fcu_us = 1000000;   // 从 1.0 s 开始，避免 0 被当成"无效时间戳"
    int64_t next_imu_ns = t0_ns;
    int64_t next_hb_ns = t0_ns;
    const int64_t imu_period_ns = 5000000;  // 200 Hz
    mavlink_message_t msg{};
    mavlink_status_t status{};
    uint8_t buf[1024];
    pure_uav::odom_tool::ProbeStats stats;
    uint64_t odom_frames = 0;
    FILE* raw_fp = save_raw.empty() ? nullptr : std::fopen(save_raw.c_str(), "wb");
    if (!save_raw.empty() && raw_fp == nullptr) {
        std::fprintf(stderr, "打不开 --save-raw: %s\n", save_raw.c_str());
        return 1;
    }

    while (!g_stop.load()) {
        const int64_t now = steadyNowNs();
        if (duration > 0.0 &&
            static_cast<double>(now - t0_ns) * 1e-9 >= duration) {
            break;
        }

        // 200 Hz IMU
        if (now >= next_imu_ns) {
            mavlink_message_t m;
            mavlink_msg_highres_imu_pack(
                /*sysid=*/1, /*compid=*/1, &m, fcu_us,
                0.0f, 0.0f, 9.80665f,        // 静止：机体 z 轴 +g（FRD）
                0.0f, 0.0f, 0.0f,
                NAN, NAN, NAN,
                0.0f, 0.0f, 0.0f, 0.0f,
                0, 0);
            sendMsg(master, m);
            fcu_us += 5000;
            next_imu_ns += imu_period_ns;
            if (next_imu_ns < now - 1000000000LL) next_imu_ns = now;  // 落后太多就重对齐
        }

        // 1 Hz HEARTBEAT
        if (now >= next_hb_ns) {
            mavlink_message_t m;
            mavlink_msg_heartbeat_pack(
                1, 1, &m, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                armed ? (MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
                      : MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0, MAV_STATE_ACTIVE);
            sendMsg(master, m);
            next_hb_ns += 1000000000LL;
        }

        // 解析上位机发来的数据
        struct pollfd pfd{};
        pfd.fd = master;
        pfd.events = POLLIN;
        if (::poll(&pfd, 1, 1) > 0 && (pfd.revents & POLLIN)) {
            const ssize_t n = ::read(master, buf, sizeof(buf));
            if (raw_fp != nullptr && n > 0) std::fwrite(buf, 1, static_cast<size_t>(n), raw_fp);
            for (ssize_t i = 0; i < n; ++i) {
                mavlink_message_t m{};
                if (!mavlink_parse_char(MAVLINK_COMM_2, buf[i], &m, &status)) continue;
                if (m.msgid != MAVLINK_MSG_ID_ODOMETRY) continue;
                mavlink_odometry_t o{};
                mavlink_msg_odometry_decode(&m, &o);
                stats.add(o, 0);
                ++odom_frames;
                if (verbose) {
                    std::printf("[fake_fcu] ODO #%llu t=%.3fs pos=(%.3f,%.3f,%.3f) q=%d reset=%u\n",
                                static_cast<unsigned long long>(odom_frames),
                                static_cast<double>(o.time_usec) * 1e-6, o.x, o.y, o.z,
                                static_cast<int>(o.quality),
                                static_cast<unsigned>(o.reset_counter));
                    std::fflush(stdout);
                }
            }
        }
    }

    stats.countGaps();
    std::printf("\n[fake_fcu] 收到 ODOMETRY %llu 帧，实测 %.1f Hz\n",
                static_cast<unsigned long long>(stats.odometry), stats.rateHz());
    if (stats.odometry > 0) {
        std::printf("[fake_fcu] quality min/max = %d/%d，reset 变化 %llu 次\n",
                    static_cast<int>(stats.min_quality), static_cast<int>(stats.max_quality),
                    static_cast<unsigned long long>(stats.reset_changes));
        std::printf("[fake_fcu] 时间戳倒退 %llu 次，坐标系异常 %llu 帧，超长间隔 %llu 个\n",
                    static_cast<unsigned long long>(stats.time_backwards),
                    static_cast<unsigned long long>(stats.bad_frame),
                    static_cast<unsigned long long>(stats.gaps));
        std::printf("[fake_fcu] %s\n",
                    (stats.time_backwards == 0 && stats.bad_frame == 0 && stats.bad_quat == 0)
                        ? "✅ 字段结构正常（这一步只证明“上位机→链路”没问题；融合要看真飞控日志）"
                        : "⚠️ 有异常帧，先查编码/时间基准");
    } else {
        std::printf("[fake_fcu] 没有收到 ODOMETRY：检查上位机是否在发、波特率/路径是否一致。\n");
    }

    if (raw_fp != nullptr) std::fclose(raw_fp);
    if (!link_path.empty()) ::unlink(link_path.c_str());
    ::close(master);
    return 0;
}
