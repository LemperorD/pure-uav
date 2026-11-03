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

    std::lock_guard<std::mutex> lk(fd_mtx_);
    fd_ = openSerial(cfg.device, cfg.baudrate, error);
    if (fd_ < 0) return false;

    // 新会话：身份锁定、连接态、时间戳全部从头开始
    streams_requested_ = false;
    last_stream_request_ns_ = 0;
    last_heartbeat_recv_ns_ = 0;
    last_heartbeat_send_ns_ = 0;
    connected_.store(false);
    armed_.store(false);
    locked_sysid_.store(0);
    locked_compid_.store(0);
    reconnects_.store(0);
    clearImuQueue();

    running_.store(true);
    thread_ = std::thread(&MavlinkLink::recvLoop, this);
    return true;
}

void MavlinkLink::stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
    {
        std::lock_guard<std::mutex> lk(fd_mtx_);
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    connected_.store(false);
    armed_.store(false);
    clearImuQueue();
}

void MavlinkLink::clearImuQueue() {
    std::lock_guard<std::mutex> lk(q_mtx_);
    imu_q_.clear();
}

bool MavlinkLink::sendBytes(const uint8_t* buf, uint16_t len) {
    std::lock_guard<std::mutex> lk(send_mtx_);
    std::lock_guard<std::mutex> fdlk(fd_mtx_);
    if (fd_ < 0) return false;
    return writeAll(fd_, buf, len);
}

bool MavlinkLink::writeAll(int fd, const uint8_t* buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        const ssize_t n = ::write(fd, buf + off, len - off);
        if (n > 0) {
            off += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EINTR)) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd pfd{};
            pfd.fd = fd;
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
            sysid_, compid_, &msg, fcu_sysid_.load(), fcu_compid_.load(),
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

bool MavlinkLink::acceptHeartbeat(uint8_t sysid, uint8_t compid, uint8_t autopilot, uint8_t type) {
    // 自己的心跳（某些路由会回环）不能当成飞控
    if (sysid == sysid_) return false;

    // 1) 显式配置优先
    if (cfg_.target_sysid != 0) {
        if (sysid != cfg_.target_sysid) return false;
        if (cfg_.target_compid != 0 && compid != cfg_.target_compid) return false;
        locked_sysid_.store(sysid);
        locked_compid_.store(compid);
        return true;
    }

    // 2) 已锁定：只认同一个 sysid/compid
    const uint8_t locked = locked_sysid_.load();
    if (locked != 0) {
        return sysid == locked && compid == locked_compid_.load();
    }

    // 3) 自动锁定第一个"像飞控"的心跳：
    //    · 排除地面站（MAV_TYPE_GCS）和机载计算机（ONBOARD_CONTROLLER）
    //    · 排除没有自动驾驶类型的组件（autopilot=INVALID）
    //    这正是"地面站心跳覆盖飞控状态"那个 bug 的修法：地面站心跳进不来。
    if (autopilot == MAV_AUTOPILOT_INVALID) return false;
    if (type == MAV_TYPE_GCS || type == MAV_TYPE_ONBOARD_CONTROLLER) return false;
    if (cfg_.target_compid != 0 && compid != cfg_.target_compid) return false;

    locked_sysid_.store(sysid);
    locked_compid_.store(compid);
    std::printf("[MAVLink] 锁定目标飞控心跳 sysid=%u compid=%u autopilot=%u\n",
                static_cast<unsigned>(sysid), static_cast<unsigned>(compid),
                static_cast<unsigned>(autopilot));
    std::fflush(stdout);
    return true;
}

void MavlinkLink::handleLinkLoss(const char* why) {
    int old = -1;
    {
        std::lock_guard<std::mutex> lk(fd_mtx_);
        old = fd_;
        fd_ = -1;
    }
    if (old >= 0) ::close(old);

    // 注意：**不清 armed_**。链路断开时若把"已解锁"误判成"地面"，VIO 会打开零速更新；
    // 空中丢链路是可能的，这时宁可保持最后一次已知状态（发送已经被 connected() 挡住）。
    connected_.store(false);
    streams_requested_ = false;
    last_heartbeat_recv_ns_ = 0;
    last_stream_request_ns_ = 0;
    clearImuQueue();
    std::printf("[MAVLink] 链路断开（%s）：停止发送位姿，等待串口重连…\n", why);
    std::fflush(stdout);
}

bool MavlinkLink::tryReopen(std::string* error) {
    const int nfd = openSerial(cfg_.device, cfg_.baudrate, error);
    if (nfd < 0) return false;
    {
        std::lock_guard<std::mutex> lk(fd_mtx_);
        fd_ = nfd;
    }
    // 新会话：重新学习飞控身份，旧队列/旧连接态全部作废
    connected_.store(false);
    locked_sysid_.store(0);
    locked_compid_.store(0);
    streams_requested_ = false;
    clearImuQueue();
    reconnects_.fetch_add(1);
    return true;
}

void MavlinkLink::recvLoop() {
    mavlink_message_t msg{};
    mavlink_status_t status{};
    uint8_t buf[512];
    int64_t last_imu_recv_ns = 0;
    long long ignored_heartbeats = 0;
    int reopen_failures = 0;

    while (running_.load()) {
        int fd = -1;
        {
            std::lock_guard<std::mutex> lk(fd_mtx_);
            fd = fd_;
        }

        // ---- 设备掉线：每 500 ms 尝试重新打开（FC 重启时 USB 常常会重新枚举）----
        if (fd < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if (!running_.load()) break;
            std::string err;
            if (tryReopen(&err)) {
                reopen_failures = 0;
                last_imu_recv_ns = 0;
                last_stream_request_ns_ = 0;
                last_heartbeat_recv_ns_ = 0;
                last_heartbeat_send_ns_ = 0;
                last_imu_us_.store(0);
                std::printf("[MAVLink] 串口已重连 %s（第 %llu 次），等待飞控心跳…\n",
                            cfg_.device.c_str(),
                            static_cast<unsigned long long>(reconnects_.load()));
                std::fflush(stdout);
            } else if (++reopen_failures % 10 == 1) {
                std::printf("[MAVLink] 串口重连失败（%s），继续尝试…\n", err.c_str());
                std::fflush(stdout);
            }
            continue;
        }

        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        const int pr = ::poll(&pfd, 1, 50);
        bool fatal = false;
        if (pr > 0) {
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) fatal = true;
            ssize_t n = 0;
            {
                std::lock_guard<std::mutex> lk(fd_mtx_);
                if (fd_ != fd) continue;   // 期间被停止/重连换掉了
                n = ::read(fd_, buf, sizeof(buf));
            }
            if (n > 0) {
                for (ssize_t i = 0; i < n; ++i) {
                    if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                        continue;
                    }
                    // 锁定飞控后，只接受该 sysid 的数据流。否则同链路如果路由转发了
                    // 别的飞行器/地面站的 ODOMETRY/LOCAL_POSITION，会把状态搅乱。
                    const uint8_t locked = locked_sysid_.load();
                    if (msg.msgid != MAVLINK_MSG_ID_HEARTBEAT && locked != 0 &&
                        msg.sysid != locked) {
                        continue;
                    }
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_HEARTBEAT: {
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
                            if (!acceptHeartbeat(msg.sysid, msg.compid, hb.autopilot, hb.type)) {
                                ++ignored_heartbeats;
                                if (ignored_heartbeats <= 3 || ignored_heartbeats % 200 == 0) {
                                    std::printf("[MAVLink] 忽略非目标心跳 sysid=%u compid=%u"
                                                " type=%u autopilot=%u（累计 %lld）\n",
                                                static_cast<unsigned>(msg.sysid),
                                                static_cast<unsigned>(msg.compid),
                                                static_cast<unsigned>(hb.type),
                                                static_cast<unsigned>(hb.autopilot),
                                                ignored_heartbeats);
                                    std::fflush(stdout);
                                }
                                break;
                            }
                            fcu_sysid_.store(msg.sysid);
                            fcu_compid_.store(msg.compid);
                            // 【2026-10-25】解锁状态：地面可开零速更新，空中必须关
                            armed_.store((hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0);
                            if (!connected_.exchange(true)) {
                                std::printf("[MAVLink] 飞控心跳 sysid=%u compid=%u autopilot=%u\n",
                                            static_cast<unsigned>(msg.sysid),
                                            static_cast<unsigned>(msg.compid),
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
                fatal = true;
            } else if (n == 0 && (pfd.revents & POLLHUP)) {
                fatal = true;
            }
        }

        if (fatal) {
            handleLinkLoss("串口读错误/设备拔出");
            continue;
        }

        const int64_t now = steadyNowNs();

        // 心跳超时：断开连接态；同时清掉身份锁定，允许飞控重启后重新锁定。
        if (connected_.load() && last_heartbeat_recv_ns_ > 0 &&
            now - last_heartbeat_recv_ns_ > 3000000000LL) {
            connected_.store(false);
            streams_requested_ = false;
            locked_sysid_.store(0);
            locked_compid_.store(0);
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
    mavlink_message_t msg;
    mavlink_msg_param_request_read_pack(static_cast<uint8_t>(sysid_), static_cast<uint8_t>(compid_),
                                        &msg, fcu_sysid_.load(), fcu_compid_.load(),
                                        name.c_str(), -1);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    std::lock_guard<std::mutex> lk(send_mtx_);
    std::lock_guard<std::mutex> fdlk(fd_mtx_);
    if (fd_ < 0) return;
    writeAll(fd_, buf, len);
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

uint16_t MavlinkLink::encodeVisionPose(const VisionPose& pose, int quality, uint8_t reset_counter,
                                       float pos_sigma_m, float vel_sigma_mps, uint8_t sysid,
                                       uint8_t compid, uint8_t* buf, size_t buf_cap) {
    if (buf == nullptr || buf_cap < MAVLINK_MAX_PACKET_LEN) return 0;

    //=======================================================================
    // 协方差：MAVLink ODOMETRY 的 21 个元素是 6x6 上三角逐行，对角线在下标
    // 0/6/11/15/18/20。这里只填位置和速度的对角；姿态/角速度没有估计值，保持
    // 0（飞控侧会 constrain 到 VISO_YAW_M_NSE 之类的参数下限）。
    // 没给 sigma 时按老行为把 [0] 填 NaN（= 未知）。
    //=======================================================================
    // 两个 sigma 必须**各自独立**判断：只给位置不给速度时，位置的不确定性
    // 不能因为速度未知就一起丢掉（2026-10-29 验收指出的问题）。
    float pose_cov[21] = {0.0f};
    float vel_cov[21] = {0.0f};
    if (pos_sigma_m > 0.0f) {
        const float pv = pos_sigma_m * pos_sigma_m;
        pose_cov[0] = pose_cov[6] = pose_cov[11] = pv;
    } else {
        // 未知就把三个对角都标成 NaN。只标 [0] 的话，飞控按 0/6/11 组合时
        // 另外两个 0 会被当成"完美已知"，语义就混了。
        pose_cov[0] = pose_cov[6] = pose_cov[11] = std::nanf("");
    }
    if (vel_sigma_mps > 0.0f) {
        const float vv = vel_sigma_mps * vel_sigma_mps;
        vel_cov[0] = vel_cov[6] = vel_cov[11] = vv;
    } else {
        vel_cov[0] = vel_cov[6] = vel_cov[11] = std::nanf("");
    }

    const int q = quality < 0 ? 0 : (quality > 100 ? 100 : quality);

    mavlink_message_t msg;
    // ⚠️ 最后三个参数的顺序是 (reset_counter, estimator_type, quality)，这是
    //    MAVLink 生成的函数签名，**不是** (quality, estimator_type, reset_counter)。
    //    2026-10-28 之前这里传反了，见 mavlink_link.hpp 的说明。
    mavlink_msg_odometry_pack(
        sysid, compid, &msg,
        pose.timestamp_us,
        static_cast<uint8_t>(pose.frame_id),
        static_cast<uint8_t>(pose.child_frame_id),
        pose.x, pose.y, pose.z, pose.q,
        pose.vx, pose.vy, pose.vz,
        0.0f, 0.0f, 0.0f,
        pose_cov, vel_cov,
        reset_counter,                  // ← 第 1 个：重置计数
        MAV_ESTIMATOR_TYPE_VIO,         // ← 第 2 个：估计器类型
        static_cast<int8_t>(q));        // ← 第 3 个：质量 0-100

    return mavlink_msg_to_send_buffer(buf, &msg);
}

bool MavlinkLink::sendVisionPose(const VisionPose& pose, int quality, uint8_t reset_counter,
                                 float pos_sigma_m, float vel_sigma_mps) {
    if (!running_.load() || fd_ < 0) return false;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = encodeVisionPose(pose, quality, reset_counter, pos_sigma_m, vel_sigma_mps,
                                          sysid_, compid_, buf, sizeof(buf));
    if (len == 0) return false;
    return sendBytes(buf, len);
}

}  // namespace pure_uav
