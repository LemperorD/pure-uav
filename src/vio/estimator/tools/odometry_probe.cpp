// ODOMETRY 线上探针：从串口或原始字节文件解码 MAVLink，核对"发出去的到底是什么"。
//
// 用途（和联调方案第 6 节对应）：
//   · ① 打包检查：可以喂一份抓到的原始字节，逐字段核对；
//   · ③ 飞控接收前：用 USB-TTL 分线/tee 出一路，独立看上位机流；
//   · 出问题时分清"没发出去"和"发了但飞控不用"。
//
// 用法：
//   ./bin/odometry_probe --serial /dev/ttyUSB0 --baud 921600
//   ./bin/odometry_probe --file raw_mavlink.bin
//   ./bin/odometry_probe --file raw.bin --csv pose.csv --quiet
//
// 只读：不发任何数据（不发心跳、不请求消息流），所以不会干扰正在跑的链路。

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "common/mavlink.h"

#include "odom_tool_core.hpp"

namespace {

std::atomic<bool> g_stop{false};
void onSigint(int) { g_stop.store(true); }

speed_t toSpeed(int baud) {
    switch (baud) {
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 3000000: return B3000000;
        default: return B0;
    }
}

int openSerial(const std::string& device, int baud, std::string* error) {
    const speed_t sp = toSpeed(baud);
    if (sp == B0) {
        if (error) *error = "不支持的波特率 " + std::to_string(baud);
        return -1;
    }
    const int fd = ::open(device.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        if (error) *error = "打开失败: " + std::string(std::strerror(errno));
        return -1;
    }
    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        if (error) *error = "tcgetattr 失败";
        ::close(fd);
        return -1;
    }
    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    cfsetispeed(&tty, sp);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        if (error) *error = "tcsetattr 失败";
        ::close(fd);
        return -1;
    }
    return fd;
}

void usage(const char* argv0) {
    std::printf(
        "用法: %s (--serial DEV | --file PATH) [选项]\n"
        "\n"
        "  --serial DEV     监听串口（只读，不发任何数据）\n"
        "  --file PATH      解析保存下来的原始 MAVLink 字节流\n"
        "  --baud N         波特率（默认 921600）\n"
        "  --csv PATH       把 ODOMETRY 写成 CSV\n"
        "  --duration S     跑 S 秒后退出（默认一直跑）\n"
        "  --quiet          不逐帧打印，只在结束时给统计\n"
        "  -h, --help\n",
        argv0);
}

struct CsvOut {
    FILE* fp = nullptr;
    ~CsvOut() {
        if (fp) std::fclose(fp);
    }
    void header() {
        if (!fp) return;
        std::fprintf(fp,
                     "time_us,x,y,z,qw,qx,qy,qz,vx,vy,vz,quality,reset_counter,"
                     "cov_x,cov_y,cov_z\n");
    }
    void row(const mavlink_odometry_t& o) {
        if (!fp) return;
        std::fprintf(fp, "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%u,%.6g,%.6g,%.6g\n",
                     static_cast<unsigned long long>(o.time_usec), o.x, o.y, o.z, o.q[0], o.q[1],
                     o.q[2], o.q[3], o.vx, o.vy, o.vz, static_cast<int>(o.quality),
                     static_cast<unsigned>(o.reset_counter), o.pose_covariance[0],
                     o.pose_covariance[6], o.pose_covariance[11]);
    }
};

void printSummary(const pure_uav::odom_tool::ProbeStats& st) {
    std::printf("\n===== ODOMETRY 统计 =====\n");
    std::printf("  ODOMETRY 帧数 : %llu\n", static_cast<unsigned long long>(st.odometry));
    std::printf("  实测频率      : %.1f Hz（中位间隔 %.0f us）\n", st.rateHz(), st.medianIntervalUs());
    std::printf("  quality       : min=%d max=%d（<50 的帧 %llu）\n", static_cast<int>(st.min_quality),
                static_cast<int>(st.max_quality), static_cast<unsigned long long>(st.bad_quality));
    std::printf("  reset 变化    : %llu 次\n", static_cast<unsigned long long>(st.reset_changes));
    std::printf("  时间戳倒退    : %llu 次\n", static_cast<unsigned long long>(st.time_backwards));
    std::printf("  四元数异常    : %llu 帧\n", static_cast<unsigned long long>(st.bad_quat));
    std::printf("  坐标系异常    : %llu 帧（期望 frame=20 / child=12）\n",
                static_cast<unsigned long long>(st.bad_frame));
    std::printf("  超长间隔      : %llu 个（>3 倍中位）\n", static_cast<unsigned long long>(st.gaps));
    if (st.odometry >= 4) {
        if (st.time_backwards > 0 || st.bad_frame > 0 || st.bad_quat > 0) {
            std::printf("  ⚠️ 有异常帧，先查编码/时间基准，不要直接上飞控。\n");
        } else {
            std::printf("  ✅ 帧结构正常。\n");
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string serial;
    std::string file;
    std::string csv_path;
    int baud = 921600;
    double duration = 0.0;
    bool quiet = false;

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
        } else if (a == "--serial") {
            serial = need("--serial");
        } else if (a == "--file") {
            file = need("--file");
        } else if (a == "--baud") {
            baud = std::stoi(need("--baud"));
        } else if (a == "--csv") {
            csv_path = need("--csv");
        } else if (a == "--duration") {
            duration = std::stod(need("--duration"));
        } else if (a == "--quiet") {
            quiet = true;
        } else {
            std::fprintf(stderr, "未知参数: %s\n", a.c_str());
            usage(argv[0]);
            return 2;
        }
    }
    if (serial.empty() && file.empty()) {
        std::fprintf(stderr, "必须给 --serial 或 --file\n");
        usage(argv[0]);
        return 2;
    }
    if (!serial.empty() && !file.empty()) {
        std::fprintf(stderr, "--serial 和 --file 只能二选一\n");
        return 2;
    }

    std::signal(SIGINT, onSigint);
    std::signal(SIGTERM, onSigint);

    std::string err;
    int fd = -1;
    if (!serial.empty()) {
        fd = openSerial(serial, baud, &err);
        if (fd < 0) {
            std::fprintf(stderr, "串口打开失败 %s: %s\n", serial.c_str(), err.c_str());
            return 1;
        }
        std::printf("[probe] 只读监听 %s @ %d\n", serial.c_str(), baud);
    }

    CsvOut csv;
    if (!csv_path.empty()) {
        csv.fp = std::fopen(csv_path.c_str(), "w");
        if (!csv.fp) {
            std::fprintf(stderr, "打不开 CSV: %s\n", csv_path.c_str());
            return 1;
        }
        csv.header();
    }

    pure_uav::odom_tool::ProbeStats stats;
    mavlink_message_t msg{};
    mavlink_status_t status{};
    uint8_t buf[4096];

    const bool from_file = !file.empty();
    FILE* in = from_file ? std::fopen(file.c_str(), "rb") : nullptr;
    if (from_file && !in) {
        std::fprintf(stderr, "打不开文件: %s\n", file.c_str());
        return 1;
    }

    // 统一成"读一段 → 逐字节喂 parser"
    auto feed = [&](const uint8_t* data, size_t n) {
        for (size_t i = 0; i < n; ++i) {
            mavlink_message_t m{};
            if (!mavlink_parse_char(MAVLINK_COMM_3, data[i], &m, &status)) continue;
            if (m.msgid == MAVLINK_MSG_ID_ODOMETRY) {
                mavlink_odometry_t o{};
                mavlink_msg_odometry_decode(&m, &o);
                stats.add(o, 0);
                csv.row(o);
                if (!quiet) {
                    const int64_t t = static_cast<int64_t>(o.time_usec);
                    const bool backwards = stats.prev_time_us >= 0 && t < stats.prev_time_us;
                    std::printf("[ODO] t=%.6fs frame=%u/%u pos=(%.3f,%.3f,%.3f) "
                                "q=(%.4f,%.4f,%.4f,%.4f) v=(%.3f,%.3f,%.3f) q=%d reset=%u "
                                "cov=(%.4g,%.4g,%.4g)%s%s%s\n",
                                static_cast<double>(o.time_usec) * 1e-6,
                                static_cast<unsigned>(o.frame_id),
                                static_cast<unsigned>(o.child_frame_id), o.x, o.y, o.z, o.q[0],
                                o.q[1], o.q[2], o.q[3], o.vx, o.vy, o.vz,
                                static_cast<int>(o.quality), static_cast<unsigned>(o.reset_counter),
                                o.pose_covariance[0], o.pose_covariance[6], o.pose_covariance[11],
                                backwards ? " ⚠️时间倒退" : "",
                                (o.frame_id != 20 || o.child_frame_id != 12) ? " ⚠️坐标系" : "",
                                std::fabs(o.q[0] * o.q[0] + o.q[1] * o.q[1] + o.q[2] * o.q[2] +
                                              o.q[3] * o.q[3] - 1.0f) > 1e-2f
                                    ? " ⚠️四元数"
                                    : "");
                    std::fflush(stdout);
                }
            } else if (m.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                ++stats.heartbeat;
            } else if (m.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
                ++stats.imu;
            } else {
                ++stats.other;
            }
        }
    };

    if (from_file) {
        size_t n = 0;
        while ((n = std::fread(buf, 1, sizeof(buf), in)) > 0) feed(buf, n);
        std::fclose(in);
        in = nullptr;
    } else {
        const auto t0 = std::chrono::steady_clock::now();
        while (!g_stop.load()) {
            if (duration > 0.0) {
                const double el =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                if (el >= duration) break;
            }
            struct pollfd pfd{};
            pfd.fd = fd;
            pfd.events = POLLIN;
            const int pr = ::poll(&pfd, 1, 50);
            if (pr <= 0) continue;
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                std::printf("[probe] 串口挂断（revents=0x%x），退出。\n",
                            static_cast<unsigned>(pfd.revents));
                break;
            }
            const ssize_t n = ::read(fd, buf, sizeof(buf));
            if (n > 0) feed(buf, static_cast<size_t>(n));
        }
        if (fd >= 0) ::close(fd);
    }

    stats.countGaps();
    printSummary(stats);
    return 0;
}
