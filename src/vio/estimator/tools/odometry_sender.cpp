// 无相机/无 VIO 时的合成 ODOMETRY 发送器（台架测试用）。
//
// 用途：把"飞控能不能收到定位、轴向/单位对不对"和"VIO 算得准不准"彻底解耦。
// 不接相机、不跑估计器，只按 scripts 里的轨迹以 30 Hz 发 ODOMETRY。
//
// 前提：串口另一头是飞控（会发 HEARTBEAT + HIGHRES_IMU）。时间戳必须落在飞控
// boot 时间域，所以这里用第一帧 HIGHRES_IMU 的 time_usec 做锚点，再用主机单调
// 时钟外推——和 vio_live 用的是同一套时间基准。
//
// 用法：
//   ./bin/odometry_sender --serial /dev/ttyACM0 --baud 921600
//   ./bin/odometry_sender --serial /dev/ttyACM0 --rate 30 --loop
//
// 阶段（共 27 s，`--loop` 会循环）：
//   静止 → 前移 1 m → 退回 → 右移 0.6 m → 退回 → 上抬 0.5 m → 落回 → 偏航 90°
// 每个阶段开始时打印"飞控端应该看到什么"，方便一个人操作、一个人看点。

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

#include "mavlink_link.hpp"
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
        "用法: %s --serial DEV [选项]\n"
        "\n"
        "  --serial DEV     飞控串口（默认 /dev/ttyACM0）\n"
        "  --baud N         波特率（默认 921600）\n"
        "  --rate HZ        发送频率（默认 30，必须 ≥ 4 才满足 ArduPilot 外部定位要求）\n"
        "  --loop           脚本跑完后循环\n"
        "  --quality N      ODOMETRY quality（默认 90）\n"
        "  --reset N        ODOMETRY reset_counter（默认 0）\n"
        "  --hz-wait S      等飞控心跳/IMU 的超时秒数（默认 10）\n"
        "  --duration S     只跑 S 秒（默认跑完整段脚本 ~27 s）\n"
        "  -h, --help\n"
        "\n"
        "脚本：静止 → 前移 1 m → 右移 0.6 m → 上抬 0.5 m → 偏航 +90°，共约 27 s。\n",
        argv0);
}

}  // namespace

int main(int argc, char** argv) {
    std::string serial = "/dev/ttyACM0";
    int baud = 921600;
    double rate_hz = 30.0;
    bool loop = false;
    int quality = 90;
    int reset_counter = 0;
    double wait_s = 10.0;
    double run_s = 0.0;   // >0 时只跑这么久（默认按脚本长度）

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
        } else if (a == "--baud") {
            baud = std::stoi(need("--baud"));
        } else if (a == "--rate") {
            rate_hz = std::stod(need("--rate"));
        } else if (a == "--loop") {
            loop = true;
        } else if (a == "--quality") {
            quality = std::stoi(need("--quality"));
        } else if (a == "--reset") {
            reset_counter = std::stoi(need("--reset"));
        } else if (a == "--hz-wait") {
            wait_s = std::stod(need("--hz-wait"));
        } else if (a == "--duration") {
            run_s = std::stod(need("--duration"));
        } else {
            std::fprintf(stderr, "未知参数: %s\n", a.c_str());
            usage(argv[0]);
            return 2;
        }
    }
    if (rate_hz < 4.0) {
        std::fprintf(stderr, "发送频率至少要 4 Hz（ArduPilot 外部定位要求），当前 %.1f\n", rate_hz);
        return 2;
    }
    reset_counter = reset_counter < 0 ? 0 : (reset_counter > 255 ? 255 : reset_counter);

    std::signal(SIGINT, onSigint);
    std::signal(SIGTERM, onSigint);

    pure_uav::MavlinkLink link;
    pure_uav::MavlinkLink::Config cfg;
    cfg.device = serial;
    cfg.baudrate = baud;
    cfg.target_sysid = 0;   // 自动锁定
    std::string err;
    if (!link.start(cfg, &err)) {
        std::fprintf(stderr, "打开串口失败: %s\n", err.c_str());
        return 1;
    }

    std::printf("[sender] 等待飞控心跳（%s @ %d）…\n", serial.c_str(), baud);
    const int64_t wait_deadline = steadyNowNs() + static_cast<int64_t>(wait_s * 1e9);
    while (!link.connected() && steadyNowNs() < wait_deadline && !g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!link.connected()) {
        std::fprintf(stderr, "[sender] %.0f s 内没收到飞控心跳，退出。检查串口/波特率/飞控是否上电。\n",
                     wait_s);
        link.stop();
        return 1;
    }
    std::printf("[sender] 飞控已连接 sysid=%u，等待 HIGHRES_IMU 建立时间基准…\n",
                static_cast<unsigned>(link.fcuSysId()));

    while (link.lastImuTimestampUs() == 0 && !g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (link.lastImuTimestampUs() == 0) {
        std::fprintf(stderr, "[sender] 没收到 HIGHRES_IMU，拿不到飞控 boot 时间，无法发 ODOMETRY。\n"
                             "         确认飞控在推 IMU（或先用 vio_live --dump-params 检查链路）。\n");
        link.stop();
        return 1;
    }

    const int64_t t0_host_ns = steadyNowNs();
    const uint64_t t0_fcu_us = link.lastImuTimestampUs();
    std::printf("[sender] 时间基准：飞控 %llu us / 主机 %lld ns。开始脚本（约 %.0f s）\n",
                static_cast<unsigned long long>(t0_fcu_us),
                static_cast<long long>(t0_host_ns), pure_uav::odom_tool::scriptDuration());
    std::fflush(stdout);

    const double period_s = 1.0 / rate_hz;
    int64_t next_send_ns = t0_host_ns;
    int last_phase = -1;
    uint64_t sent = 0, failed = 0;
    int64_t last_report_ns = t0_host_ns;

    while (!g_stop.load()) {
        const int64_t now_ns = steadyNowNs();
        double t_s = static_cast<double>(now_ns - t0_host_ns) * 1e-9;
        const double duration = pure_uav::odom_tool::scriptDuration();
        if (run_s > 0.0 && t_s >= run_s) {
            std::printf("\n[sender] 到达 --duration %.1f s，退出。\n", run_s);
            break;
        }
        if (!loop && t_s > duration + 1.0) {
            std::printf("\n[sender] 脚本跑完，退出。\n");
            break;
        }
        if (loop && t_s > duration) {
            // 循环：把时间折回脚本开头（时间戳继续前进，只是内容重复）
            const double cycles = std::floor(t_s / duration);
            t_s -= cycles * duration;
        }

        const int phase = pure_uav::odom_tool::scriptPhaseIndex(t_s);
        if (phase != last_phase) {
            last_phase = phase;
            const auto& st = pure_uav::odom_tool::scriptSteps()[phase];
            std::printf("\n[sender] 阶段 %d/%zu：%s  →  飞控端应看到：%s\n", phase + 1,
                        pure_uav::odom_tool::scriptSteps().size(), st.name, st.expect);
            std::fflush(stdout);
        }

        const auto sample = pure_uav::odom_tool::sampleScript(t_s);
        pure_uav::VisionPose pose;
        pose.timestamp_us = t0_fcu_us + static_cast<uint64_t>((now_ns - t0_host_ns) / 1000);
        pose.frame_id = pure_uav::VisionFrame::LOCAL_FRD;
        pose.child_frame_id = pure_uav::VisionFrame::BODY_FRD;
        pose.x = sample.x;
        pose.y = sample.y;
        pose.z = sample.z;
        std::memcpy(pose.q, sample.q, sizeof(pose.q));
        pose.vx = sample.vx;
        pose.vy = sample.vy;
        pose.vz = sample.vz;

        if (link.sendVisionPose(pose, quality, static_cast<uint8_t>(reset_counter), 0.15f, 0.25f)) {
            ++sent;
        } else {
            ++failed;
        }

        if (now_ns - last_report_ns > 2000000000LL) {
            last_report_ns = now_ns;
            std::printf("[sender] t=%.1fs 已发 %llu 帧（失败 %llu）quality=%d reset=%d\n", t_s,
                        static_cast<unsigned long long>(sent),
                        static_cast<unsigned long long>(failed), quality, reset_counter);
            std::fflush(stdout);
        }

        next_send_ns += static_cast<int64_t>(period_s * 1e9);
        const int64_t sleep_ns = next_send_ns - steadyNowNs();
        if (sleep_ns > 0) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
        } else {
            next_send_ns = steadyNowNs();  // 落后就重新对齐，不追帧
        }
    }

    std::printf("\n[sender] 结束：发送 %llu 帧，失败 %llu 帧\n",
                static_cast<unsigned long long>(sent),
                static_cast<unsigned long long>(failed));
    if (failed > 0) {
        std::printf("[sender] 有失败帧：优先检查串口是否被别的程序占用/线是否松。\n");
    }
    link.stop();
    return failed == 0 ? 0 : 1;
}
