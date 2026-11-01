#pragma once
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <deque>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <cstdlib>
#include <unistd.h>
#include "common/mavlink.h"

namespace wire_test {
using namespace std::chrono;
inline void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
template <typename Predicate> bool waitFor(Predicate predicate, int timeout_ms = 1500) {
    const auto deadline = steady_clock::now() + milliseconds(timeout_ms);
    do {
        if (predicate()) return true;
        std::this_thread::sleep_for(milliseconds(2));
    } while (steady_clock::now() < deadline);
    return predicate();
}
struct Pty {
    int master = -1;
    std::string slave;
    Pty() {
        master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        check(master >= 0, "posix_openpt");
        check(grantpt(master) == 0 && unlockpt(master) == 0, "grantpt/unlockpt");
        slave = ptsname(master);
    }
    ~Pty() { if (master >= 0) ::close(master); }
};
// 可替换的设备路径，用两对 PTY 模拟 USB 串口消失后重新出现。
struct DevicePath {
    std::string directory;
    std::string path;
    explicit DevicePath(const std::string& slave) {
        char pattern[] = "/tmp/pure-uav-wire-XXXXXX";
        char* created = mkdtemp(pattern); check(created != nullptr, "mkdtemp");
        directory = created; path = directory + "/serial";
        pointTo(slave);
    }
    void pointTo(const std::string& slave) {
        ::unlink(path.c_str());
        check(::symlink(slave.c_str(), path.c_str()) == 0, "device symlink");
    }
    ~DevicePath() { ::unlink(path.c_str()); ::rmdir(directory.c_str()); }
};
inline void send(int fd, const mavlink_message_t& message) {
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const size_t length = mavlink_msg_to_send_buffer(bytes, &message);
    size_t offset = 0;
    const auto deadline = steady_clock::now() + milliseconds(500);
    while (offset < length && steady_clock::now() < deadline) {
        const auto n = ::write(fd, bytes + offset, length - offset);
        if (n > 0) offset += n;
        else if (n < 0 && (errno == EAGAIN || errno == EINTR))
            std::this_thread::sleep_for(milliseconds(1));
        else throw std::runtime_error("PTY write failed");
    }
    check(offset == length, "PTY write timed out");
}
inline void heartbeat(int fd, uint8_t sys = 9, uint8_t comp = 42, bool armed = true,
                      uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(sys, comp, &message, MAV_TYPE_QUADROTOR, autopilot,
                              armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0, 0, MAV_STATE_ACTIVE);
    send(fd, message);
}
inline void imu(int fd, uint64_t time, uint8_t sys = 9, uint8_t comp = 42) {
    mavlink_message_t message{};
    mavlink_msg_highres_imu_pack(sys, comp, &message, time,
        0, 0, 9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    send(fd, message);
}
class Reader {
public:
    explicit Reader(int fd) : fd_(fd) {}
    bool next(mavlink_message_t& message, int timeout_ms = 500) {
        const auto deadline = steady_clock::now() + milliseconds(timeout_ms);
        do {
            if (!pending_.empty()) {
                message = pending_.front(); pending_.pop_front(); return true;
            }
            uint8_t bytes[4096];
            const auto n = ::read(fd_, bytes, sizeof(bytes));
            for (ssize_t i = 0; i < n; ++i) {
                mavlink_message_t out{};
                mavlink_status_t status{};
                if (mavlink_frame_char_buffer(&rx_, &status_, bytes[i], &out, &status) == MAVLINK_FRAMING_OK)
                    pending_.push_back(out);
            }
            if (n <= 0) std::this_thread::sleep_for(milliseconds(1));
        } while (!pending_.empty() || steady_clock::now() < deadline);
        return false;
    }
    void drain() { mavlink_message_t message{}; while (next(message, 5)) {} }
private:
    int fd_;
    mavlink_message_t rx_{};
    mavlink_status_t status_{};
    std::deque<mavlink_message_t> pending_;
};
} // namespace wire_test
