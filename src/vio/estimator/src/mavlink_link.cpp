#include "mavlink_link.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "common/mavlink.h"

namespace pure_uav {

namespace {

constexpr uint8_t kFcuCompId = 1;

int64_t steadyNowNs() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

speed_t toSpeed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 3000000: return B3000000;
        default: return B0;
    }
}

int openSerial(const std::string& device, int baud, std::string* error) {
    const speed_t speed = toSpeed(baud);
    if (speed == B0) {
        if (error) *error = "不支持的波特率: " + std::to_string(baud);
        return -1;
    }

    const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        if (error) *error = "打开串口失败 " + device + ": " + std::strerror(errno);
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        if (error) *error = std::string("tcgetattr 失败: ") + std::strerror(errno);
        ::close(fd);
        return -1;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        if (error) *error = std::string("tcsetattr 失败: ") + std::strerror(errno);
        ::close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

}  // namespace

MavlinkLink::~MavlinkLink() { stop(); }

bool MavlinkLink::start(const Config& cfg, std::string* error) {
    if (running_.load()) return true;
    cfg_ = cfg;

    fd_ = openSerial(cfg.device, cfg.baudrate, error);
    if (fd_ < 0) return false;

    streams_requested_ = false;
    last_stream_request_ns_ = 0;
    last_heartbeat_recv_ns_ = 0;
    last_heartbeat_send_ns_ = 0;

    running_.store(true);
    thread_ = std::thread(&MavlinkLink::recvLoop, this);
    return true;
}

void MavlinkLink::stop() {
    if (!running_.exchange(false)) {
        if (thread_.joinable()) thread_.join();
        return;
    }
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    connected_.store(false);
}

bool MavlinkLink::sendBytes(const uint8_t* buf, uint16_t len) {
    if (fd_ < 0) return false;
    std::lock_guard<std::mutex> lk(send_mtx_);
    return writeAll(buf, len);
}

bool MavlinkLink::writeAll(const uint8_t* buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        const ssize_t n = ::write(fd_, buf + off, len - off);
        if (n > 0) {
            off += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EINTR)) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd pfd{};
            pfd.fd = fd_;
            pfd.events = POLLOUT;
            ::poll(&pfd, 1, 100);
            continue;
        }
        return false;
    }
    return true;
}

void MavlinkLink::pushImu(const TimedImu& item) {
    std::lock_guard<std::mutex> lk(q_mtx_);
    if (imu_q_.size() >= cfg_.max_imu_queue) {
        imu_q_.pop_front();
        dropped_imu_.fetch_add(1);
    }
    imu_q_.push_back(item);
}

bool MavlinkLink::popImu(TimedImu& out) {
    std::lock_guard<std::mutex> lk(q_mtx_);
    if (imu_q_.empty()) return false;
    out = imu_q_.front();
    imu_q_.pop_front();
    return true;
}

size_t MavlinkLink::imuQueueSize() const {
    std::lock_guard<std::mutex> lk(q_mtx_);
    return imu_q_.size();
}

void MavlinkLink::requestStreams() {
    auto request = [&](uint32_t msg_id, int32_t interval_us) {
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_command_long_pack(
            sysid_, compid_, &msg, fcu_sysid_.load(), kFcuCompId,
            MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            static_cast<float>(msg_id), static_cast<float>(interval_us),
            0, 0, 0, 0, 0);
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        sendBytes(buf, len);
    };

    request(MAVLINK_MSG_ID_HIGHRES_IMU, cfg_.highres_imu_interval_us);
    request(MAVLINK_MSG_ID_SYSTEM_TIME, cfg_.system_time_interval_us);
    // 【2026-10-23】请求飞控自己的位置/速度估计（EKF3），用来做 VIO 的第二意见：
    // 两边速度持续背离就说明 VIO 发散了 —— 不需要真值就能抓"不报警地错"。
    request(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 100000);

    last_stream_request_ns_ = steadyNowNs();
    std::printf("[MAVLink] 已请求 HIGHRES_IMU @ %.0f Hz, SYSTEM_TIME @ %.0f Hz\n",
                1e6 / static_cast<double>(cfg_.highres_imu_interval_us),
                1e6 / static_cast<double>(cfg_.system_time_interval_us));
    // 【2026-10-23 实测 CUAV-7-Nano】请求 400 Hz，实测仍只有 200 Hz，而且时间戳间隔
    // 从"均匀 5.0 ms"退化成"2.68/7.30 ms 交替"（更差）。所以请求速率只是"上限"，
    // 真实速率一定要看状态行的 imu=(+N/2s)：N≈400 才是 200 Hz，N≈800 才是 400 Hz。
    if (cfg_.highres_imu_interval_us < 4000) {
        std::printf("[MAVLink]   ⚠️ 注意：CUAV-7-Nano 实测最高只给到约 200 Hz（请求 400 也拿不到，"
                    "且时间戳更抖）。以状态行 imu=(+N/2s) 的实测值为准。\n");
    }
    std::fflush(stdout);
}

void MavlinkLink::recvLoop() {
    mavlink_message_t msg{};
    mavlink_status_t status{};
    uint8_t buf[512];
    int64_t last_imu_recv_ns = 0;

    while (running_.load()) {
        struct pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;
        const int pr = ::poll(&pfd, 1, 50);
        if (pr > 0 && (pfd.revents & (POLLIN | POLLERR | POLLHUP))) {
            const ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n > 0) {
                for (ssize_t i = 0; i < n; ++i) {
                    if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                        continue;
                    }
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_HEARTBEAT: {
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
                            fcu_sysid_.store(msg.sysid);
                            if (!connected_.exchange(true)) {
                                std::printf("[MAVLink] 飞控心跳 sysid=%u autopilot=%u\n",
                                            static_cast<unsigned>(msg.sysid),
                                            static_cast<unsigned>(hb.autopilot));
                                std::fflush(stdout);
                            }
                            last_heartbeat_recv_ns_ = steadyNowNs();
                            if (!streams_requested_) {
                                requestStreams();
                                streams_requested_ = true;
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_HIGHRES_IMU: {
                            mavlink_highres_imu_t imu;
                            mavlink_msg_highres_imu_decode(&msg, &imu);
                            TimedImu item;
                            item.host_ns = steadyNowNs();
                            item.imu.timestamp_us = imu.time_usec;
                            item.imu.gx = imu.xgyro;
                            item.imu.gy = imu.ygyro;
                            item.imu.gz = imu.zgyro;
                            item.imu.ax = imu.xacc;
                            item.imu.ay = imu.yacc;
                            item.imu.az = imu.zacc;
                            pushImu(item);
                            last_imu_us_.store(imu.time_usec);
                            imu_received_.fetch_add(1);
                            last_imu_recv_ns = item.host_ns;
                            break;
                        }
                        case MAVLINK_MSG_ID_SYSTEM_TIME: {
                            // 仅用于确认 FCU 时间基准（boot ms / unix us）
                            mavlink_system_time_t st;
                            mavlink_msg_system_time_decode(&msg, &st);
                            (void)st;
                            break;
                        }
                        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                            mavlink_local_position_ned_t lp;
                            mavlink_msg_local_position_ned_decode(&msg, &lp);
                            {
                                std::lock_guard<std::mutex> lk(ekf3_mtx_);
                                ekf3_vx_ = lp.vx;
                                ekf3_vy_ = lp.vy;
                                ekf3_vz_ = lp.vz;
                                ekf3_time_us_ = steadyNowNs() / 1000;
                                ekf3_valid_ = true;
                                ++ekf3_count_;
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_PARAM_VALUE: {
                            mavlink_param_value_t pv;
                            mavlink_msg_param_value_decode(&msg, &pv);
                            char name[17] = {0};
                            std::memcpy(name, pv.param_id, 16);
                            {
                                std::lock_guard<std::mutex> lk(param_mtx_);
                                params_[std::string(name)] = pv.param_value;
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_COMMAND_ACK: {
                            mavlink_command_ack_t ack;
                            mavlink_msg_command_ack_decode(&msg, &ack);
                            (void)ack;
                            break;
                        }
                        default:
                            break;
                    }
                }
            } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
                // 串口异常：等待重连（保持线程活着）
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        const int64_t now = steadyNowNs();

        // 心跳超时
        if (connected_.load() && last_heartbeat_recv_ns_ > 0 &&
            now - last_heartbeat_recv_ns_ > 3000000000LL) {
            connected_.store(false);
            streams_requested_ = false;
            std::printf("[MAVLink] 心跳超时，等待重连…\n");
            std::fflush(stdout);
        }

        // 请求了 IMU 但 2 秒没收到：重发请求（部分固件/端口会丢 SET_MESSAGE_INTERVAL）
        if (connected_.load() && last_stream_request_ns_ > 0 &&
            now - last_stream_request_ns_ > 2000000000LL &&
            (last_imu_recv_ns == 0 || now - last_imu_recv_ns > 2000000000LL)) {
            requestStreams();
        }

        // 本机心跳（让飞控认识这个 component）
        if (cfg_.send_heartbeat &&
            now - last_heartbeat_send_ns_ > cfg_.heartbeat_interval_ms * 1000000LL) {
            mavlink_message_t hb_msg;
            uint8_t hb_buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_heartbeat_pack(sysid_, compid_, &hb_msg, MAV_TYPE_ONBOARD_CONTROLLER,
                                       MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
            const uint16_t len = mavlink_msg_to_send_buffer(hb_buf, &hb_msg);
            sendBytes(hb_buf, len);
            last_heartbeat_send_ns_ = now;
        }
    }
}

void MavlinkLink::requestParamByName(const std::string& name) {
    if (fd_ < 0) return;
    mavlink_message_t msg;
    mavlink_msg_param_request_read_pack(static_cast<uint8_t>(sysid_), static_cast<uint8_t>(compid_),
                                        &msg, fcu_sysid_.load(), MAV_COMP_ID_AUTOPILOT1,
                                        name.c_str(), -1);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    std::lock_guard<std::mutex> lk(send_mtx_);
    writeAll(buf, len);
}

MavlinkLink::Ekf3Velocity MavlinkLink::ekf3Velocity() const {
    std::lock_guard<std::mutex> lk(ekf3_mtx_);
    Ekf3Velocity v;
    v.valid = ekf3_valid_;
    v.vx = ekf3_vx_;
    v.vy = ekf3_vy_;
    v.vz = ekf3_vz_;
    v.timestamp_us = ekf3_time_us_;
    v.count = ekf3_count_;
    return v;
}

std::vector<std::pair<std::string, float>> MavlinkLink::receivedParams() const {
    std::lock_guard<std::mutex> lk(param_mtx_);
    std::vector<std::pair<std::string, float>> out(params_.begin(), params_.end());
    return out;
}

size_t MavlinkLink::receivedParamCount() const {
    std::lock_guard<std::mutex> lk(param_mtx_);
    return params_.size();
}

bool MavlinkLink::sendVisionPose(const VisionPose& pose, int quality, uint8_t reset_counter,
                                 float pos_sigma_m, float vel_sigma_mps) {
    if (!running_.load() || fd_ < 0) return false;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    //=======================================================================
    // 【2026-10-22】协方差/质量/重置计数不再是"未知"
    //
    // 原来 pose_cov[0] 和 vel_cov[0] 填 NaN（= 未知），quality=0、reset_counter=0。
    // 后果：飞控不知道这份数据的可信度，也**不知道我们重建过**——重建后位姿会
    // 重锚定跳变，飞控会把跳变当成真实运动。
    //
    // ODOMETRY 的 21 个协方差元素是 6x6 上三角逐行，对角线在下标 0/6/11/15/18/20。
    //   pose: x,y,z / roll,pitch,yaw    vel: vx,vy,vz / wx,wy,wz
    // 这里只填位置和速度的对角（姿态/角速度沿用 0 = 未知，交给飞控处理）。
    // 数值偏保守：宁可让飞控少信一点，也不要它过度自信。
    //=======================================================================
    float pose_cov[21] = {0.0f};
    float vel_cov[21] = {0.0f};
    if (pos_sigma_m > 0.0f && vel_sigma_mps > 0.0f) {
        const float pv = pos_sigma_m * pos_sigma_m;
        const float vv = vel_sigma_mps * vel_sigma_mps;
        pose_cov[0] = pose_cov[6] = pose_cov[11] = pv;
        vel_cov[0] = vel_cov[6] = vel_cov[11] = vv;
    } else {
        // 没给估计值就沿用老行为（未知）
        pose_cov[0] = std::nanf("");
        vel_cov[0] = std::nanf("");
    }

    mavlink_msg_odometry_pack(
        sysid_, compid_, &msg,
        pose.timestamp_us,
        static_cast<uint8_t>(pose.frame_id),
        static_cast<uint8_t>(pose.child_frame_id),
        pose.x, pose.y, pose.z, pose.q,
        pose.vx, pose.vy, pose.vz,
        0.0f, 0.0f, 0.0f,
        pose_cov, vel_cov,
        static_cast<uint8_t>(quality < 0 ? 0 : (quality > 100 ? 100 : quality)),
        MAV_ESTIMATOR_TYPE_VIO,
        reset_counter);

    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return sendBytes(buf, len);
}

}  // namespace pure_uav
