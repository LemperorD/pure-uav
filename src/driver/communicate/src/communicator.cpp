#include "communicator.hpp"

#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>

// mavlink c_library_v2 纯头文件：include 路径指向 third_party/mavlink
#include "common/mavlink.h"
#include "mavlink_helpers.h"

// yaml-cpp：配置解析
#include <yaml-cpp/yaml.h>

namespace pure_uav {

namespace {
std::mutex pack_mutex;
template <typename F, typename... Args>
auto packMessage(F pack, Args&&... args) {
    std::lock_guard<std::mutex> lk(pack_mutex);
    return pack(std::forward<Args>(args)...);
}

// 单调时钟毫秒
uint32_t timeBootMs() {
    using namespace std::chrono;
    return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

// 单调时钟纳秒（IMU 到达时刻用，FcuClock 做主机↔飞控时钟映射）
int64_t steadyNowNs() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

bool telemetryFresh(int64_t stamp_ns, int max_age_ms) {
    if (stamp_ns == 0) return false;
    const int64_t elapsed_ns = steadyNowNs() - stamp_ns;
    return elapsed_ns >= 0 && (max_age_ms < 0 || elapsed_ns <= static_cast<int64_t>(max_age_ms) * 1000000LL);
}

// 欧拉角 → 四元数 wxyz
void eulerToQuat(float roll, float pitch, float yaw, float q[4]) {
    const float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
    const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    const float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
    q[0] = cr * cp * cy + sr * sp * sy;  // w
    q[1] = sr * cp * cy - cr * sp * sy;  // x
    q[2] = cr * sp * cy + sr * cp * sy;  // y
    q[3] = cr * cp * sy - sr * sp * cy;  // z
}
}  // namespace

Communicator::~Communicator() {
    stop();
}

bool Communicator::start(const std::string& device, int baud) {  // 打开串口
    if (running_.load()) {
        return true;
    }
    // 记录设备参数：串口掉线后 recvLoop 要用同一个路径重新打开
    cfg_.device = device;
    cfg_.baudrate = baud;
    if (!port_.open(device, baud)) {
        printf("打开串口失败: %s\n", device.c_str());
        return false;
    }
    port_.flush();  // 清空缓冲

    // 【2026-10-29】每次 start() 都是新会话：清掉上一次连接的 IMU 队列、
    // 缓存数据、解锁状态和身份锁定，避免把旧会话的数据混进新连接。
    resetSessionState();

    // 先"打招呼"：飞控（ArduPilot/PX4）要看到地面站发过数据才开始推流，
    // 只读不写会双方互等，所以这里立刻发心跳并请求一次消息流。
    sendGcsHeartbeat();
    // 数据流请求由首个匹配心跳触发；此时目标 sysid/compid 尚未确认。

    running_.store(true);
    recv_thread_ = std::thread(&Communicator::recvLoop, this);  // 开启接收线程
    return true;
}

void Communicator::stop() {  // 关闭接收线程
    running_.store(false);
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
    port_.close();
    // 会话结束：队列里的数据不能再被 popImu() 取到；解锁/心跳/缓存全部失效
    resetSessionState();
}

// 【2026-10-29】心跳来源筛选：只认目标飞控。
bool Communicator::acceptHeartbeat(uint8_t sysid, uint8_t compid, uint8_t autopilot, uint8_t type) {
    if (sysid == sysid_) return false;  // 自己的心跳（回环）

    if (autopilot == MAV_AUTOPILOT_INVALID || type == MAV_TYPE_GCS || type == MAV_TYPE_ONBOARD_CONTROLLER) return false;
    if (locked_sysid_.load() != 0) return sysid == locked_sysid_.load() && compid == locked_compid_.load();

    if (cfg_.fcu_sysid != 0) {  // 显式配置优先
        if (sysid != cfg_.fcu_sysid) return false;
        if (cfg_.fcu_compid != 0 && compid != cfg_.fcu_compid) return false;
        locked_sysid_.store(sysid);
        locked_compid_.store(compid);
        return true;
    }

    const uint8_t locked = locked_sysid_.load();
    if (locked != 0) {
        return sysid == locked && compid == locked_compid_.load();
    }

    // 自动锁定第一个"像飞控"的心跳：排除地面站、机载计算机和无自动驾驶类型的组件。
    if (autopilot == MAV_AUTOPILOT_INVALID) return false;
    if (type == MAV_TYPE_GCS || type == MAV_TYPE_ONBOARD_CONTROLLER) return false;
    if (cfg_.fcu_compid != 0 && compid != cfg_.fcu_compid) return false;

    locked_sysid_.store(sysid);
    locked_compid_.store(compid);
    printf("[COMM] 锁定目标飞控心跳 sysid=%u compid=%u autopilot=%u\n", static_cast<unsigned>(sysid), static_cast<unsigned>(compid), static_cast<unsigned>(autopilot));
    return true;
}

// 【2026-10-29】会话级复位：start()/stop() 都调用。
// 目的：停止后再启动时，popImu() 绝不能返回上一次连接留下的帧；
// 旧心跳、解锁状态、固件类型、缓存数据也都必须失效。
void Communicator::resetSessionState() {
    connected_.store(false);
    invalidateTelemetry();
    {
        std::lock_guard<std::mutex> lk(imu_queue_mutex_);
        imu_queue_.clear();
    }
    dropped_imu_.store(0);
    {
        std::lock_guard<std::mutex> lk(imu_mutex_);
        imu_ = ImuData{};
    }
    {
        std::lock_guard<std::mutex> lk(attitude_mutex_);
        attitude_ = Attitude{};
    }
    {
        std::lock_guard<std::mutex> lk(battery_mutex_);
        battery_ = Battery{};
    }
    {
        std::lock_guard<std::mutex> lk(local_position_mutex_);
        local_pos_ = LocalPosition{};
    }
    {
        std::lock_guard<std::mutex> lk(system_time_mutex_);
        system_time_ = SystemTime{};
    }
    connected_.store(false);
    armed_.store(false);
    streams_requested_.store(false);
    last_heartbeat_ns_.store(0);
    locked_sysid_.store(0);
    locked_compid_.store(0);
    reconnects_.store(0);
    fcu_sysid_.store(1);
    fcu_compid_.store(1);
    autopilot_.store(0);
    custom_mode_.store(0);
    count_attitude_.store(0);
    count_local_position_.store(0);
    count_highres_imu_.store(0);
    count_battery_.store(0);
    count_system_time_.store(0);
    last_ack_command_.store(0);
    last_ack_result_.store(UINT8_MAX);
}

void Communicator::invalidateTelemetry() {
    {
        std::lock_guard<std::mutex> lock(landed_mutex_);
        landed_ = {};
    }
    {
        std::lock_guard<std::mutex> lock(attitude_mutex_);
        last_attitude_ns_.store(0);
    }
    {
        std::lock_guard<std::mutex> lock(local_position_mutex_);
        last_local_position_ns_.store(0);
    }
    {
        std::lock_guard<std::mutex> lock(system_time_mutex_);
        last_system_time_ns_.store(0);
    }
}

void Communicator::clearImuQueue() {
    std::lock_guard<std::mutex> lk(imu_queue_mutex_);
    imu_queue_.clear();
}

// 接收线程
void Communicator::recvLoop() {
    using namespace std::chrono;
    const auto heartbeat_timeout = milliseconds(cfg_.heartbeat_timeout_ms);

    mavlink_message_t msg{};
    mavlink_status_t status{};
    mavlink_message_t rx_message{};
    mavlink_status_t rx_status{};
    uint8_t buf[1024];
    long long ignored_heartbeats = 0;
    auto last_heartbeat = steady_clock::now();
    auto last_gcs_heartbeat = steady_clock::now();
    int read_errors = 0;      // 连续读错误计数（掉线判定）
    int reopen_failures = 0;  // 打印节流

    while (running_.load()) {
        // 心跳超时 → 判定断开
        if (connected_.load() && steady_clock::now() - last_heartbeat > heartbeat_timeout) {
            std::lock_guard<std::mutex> tx_lock(send_mutex_);
            connected_.store(false);
            invalidateTelemetry();
            clearImuQueue();
            {
                std::lock_guard<std::mutex> lk(imu_mutex_);
                imu_ = ImuData{};
            }
            last_ack_command_.store(0);
            last_ack_result_.store(UINT8_MAX);
            reconnects_.fetch_add(1);
            streams_requested_.store(false);  // 掉线重置，飞控恢复后重新请求消息流
            // 清身份锁定，允许飞控重启后重新锁定（sysid 理论上可能变）
            locked_sysid_.store(0);
            locked_compid_.store(0);
        }

        // ---- 串口已关闭：每 500ms 用同一路径尝试重开（飞控重启会重新枚举 USB）----
        if (!port_.isOpen()) {
            std::this_thread::sleep_for(milliseconds(500));
            if (!running_.load()) break;
            if (port_.open(cfg_.device, cfg_.baudrate)) {
                {
                    std::lock_guard<std::mutex> tx_lock(send_mutex_);
                    rx_message = {};
                    rx_status = {};
                    status = {};
                    port_.flush();
                    reopen_failures = 0;
                    read_errors = 0;
                    connected_.store(false);
                    invalidateTelemetry();
                    streams_requested_.store(false);
                    locked_sysid_.store(0);
                    locked_compid_.store(0);
                    last_heartbeat_ns_.store(0);
                    clearImuQueue();
                    reconnects_.fetch_add(1);
                    last_heartbeat = steady_clock::now();
                    last_gcs_heartbeat = steady_clock::now();
                }
                // 新会话主动打招呼，否则飞控可能不推流
                sendGcsHeartbeat();
                // 新会话等待目标心跳后重新订阅，不能沿用上次连接的飞控地址。
                printf("[COMM] 串口已重连 %s（第 %llu 次），等待飞控心跳…\n", cfg_.device.c_str(), static_cast<unsigned long long>(reconnects_.load()));
                fflush(stdout);
            } else if (++reopen_failures % 10 == 1) {
                printf("[COMM] 串口重连失败（%s），继续尝试…\n", cfg_.device.c_str());
                fflush(stdout);
            }
            continue;
        }

        // 先用 poll 看设备是否还在：USB 拔出/飞控重启时 master 端关闭，
        // 读会一直返回 0/错误，光靠心跳超时无法触发"重新打开串口"。
        short revents = 0;
        const int pr = port_.pollIn(1, &revents);
        if (pr > 0 && (revents & (POLLERR | POLLHUP | POLLNVAL))) {
            printf("[COMM] 串口挂断（poll revents=0x%x）：设备掉线，等待重连…\n", static_cast<unsigned>(revents));
            fflush(stdout);
            std::lock_guard<std::mutex> tx_lock(send_mutex_);
            port_.close();
            connected_.store(false);
            invalidateTelemetry();
            streams_requested_.store(false);
            locked_sysid_.store(0);
            locked_compid_.store(0);
            clearImuQueue();
            read_errors = 0;
            continue;
        }
        ssize_t n = port_.read(buf, sizeof(buf));
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            if (++read_errors >= 5) {
                printf("[COMM] 串口读错误（%s）：设备掉线，停止解析，等待重连…\n", std::strerror(errno));
                fflush(stdout);
                std::lock_guard<std::mutex> tx_lock(send_mutex_);
                port_.close();
                connected_.store(false);
                invalidateTelemetry();
                streams_requested_.store(false);
                locked_sysid_.store(0);
                locked_compid_.store(0);
                clearImuQueue();
                read_errors = 0;
            }
        } else if (n >= 0) {
            read_errors = 0;
        }

        if (n > 0) {
            for (ssize_t i = 0; i < n; ++i) {
                if (mavlink_frame_char_buffer(&rx_message, &rx_status, buf[i], &msg, &status) == MAVLINK_FRAMING_OK) {
                    // 锁定飞控后只接受该 sysid 的数据；同链路路由转发过来的其他
                    // 飞行器/地面站消息不能污染姿态、位置、IMU 缓存。
                    const uint8_t locked_sysid = locked_sysid_.load();
                    if (msg.msgid != MAVLINK_MSG_ID_HEARTBEAT && (!connected_.load() || locked_sysid == 0 || msg.sysid != locked_sysid || msg.compid != locked_compid_.load())) {
                        continue;
                    }
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_HEARTBEAT: {  // 心跳和飞行模式
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
                            // 【2026-10-29】只认目标飞控的心跳。地面站/机载计算机/
                            // 其他飞行器的心跳不能覆盖飞控身份、解锁状态和固件类型。
                            if (!acceptHeartbeat(msg.sysid, msg.compid, hb.autopilot, hb.type)) {
                                ++ignored_heartbeats;
                                if (ignored_heartbeats <= 3 || ignored_heartbeats % 200 == 0) {
                                    printf(
                                        "[COMM] 忽略非目标心跳 sysid=%u compid=%u type=%u"
                                        " autopilot=%u（累计 %lld）\n",
                                        static_cast<unsigned>(msg.sysid), static_cast<unsigned>(msg.compid), static_cast<unsigned>(hb.type), static_cast<unsigned>(hb.autopilot), ignored_heartbeats);
                                }
                                break;
                            }
                            fcu_sysid_.store(msg.sysid);  // 学习飞控 sysid
                            fcu_compid_.store(msg.compid);
                            autopilot_.store(hb.autopilot);  // 学习飞控固件类型
                            custom_mode_.store(hb.custom_mode);
                            // 【2026-10-28】解锁状态：VIO 地面开零速更新、空中必须关
                            armed_.store((hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0);
                            last_heartbeat_ns_.store(steadyNowNs());
                            connected_.store(true);
                            last_heartbeat = steady_clock::now();
                            // 首次连上飞控，请求需要的消息流
                            if (!streams_requested_.exchange(true)) {
                                requestDefaultStreams();
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
                            mavlink_extended_sys_state_t state;
                            mavlink_msg_extended_sys_state_decode(&msg, &state);
                            std::lock_guard<std::mutex> lock(landed_mutex_);
                            landed_.state = state.landed_state;
                            landed_.received_ns = steadyNowNs();
                            break;
                        }
                        case MAVLINK_MSG_ID_ATTITUDE: {  // 姿态
                            count_attitude_.fetch_add(1);
                            mavlink_attitude_t att;
                            mavlink_msg_attitude_decode(&msg, &att);
                            std::lock_guard<std::mutex> lk(attitude_mutex_);
                            attitude_.time_boot_ms = att.time_boot_ms;
                            attitude_.roll = att.roll;
                            attitude_.pitch = att.pitch;
                            attitude_.yaw = att.yaw;
                            eulerToQuat(att.roll, att.pitch, att.yaw, attitude_.q);
                            last_attitude_ns_.store(steadyNowNs());
                            break;
                        }
                        case MAVLINK_MSG_ID_BATTERY_STATUS: {  // 电量
                            count_battery_.fetch_add(1);
                            mavlink_battery_status_t bat;
                            mavlink_msg_battery_status_decode(&msg, &bat);
                            std::lock_guard<std::mutex> lk(battery_mutex_);
                            // 累加所有有效 cell 电压得到总电压（UINT16_MAX = 该 cell 无效）
                            float total_mv = 0.0f;
                            for (int i = 0; i < 10; ++i) {
                                if (bat.voltages[i] != UINT16_MAX) {
                                    total_mv += bat.voltages[i];
                                }
                            }
                            battery_.voltage = total_mv / 1000.0f;       // mV → V
                            battery_.remaining = bat.battery_remaining;  // %
                            break;
                        }
                        case MAVLINK_MSG_ID_COMMAND_ACK: {  // ACK
                            mavlink_command_ack_t ack;
                            mavlink_msg_command_ack_decode(&msg, &ack);
                            last_ack_command_.store(ack.command);
                            last_ack_result_.store(ack.result);
                            break;
                        }
                        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {  // 位置和速度
                            count_local_position_.fetch_add(1);
                            mavlink_local_position_ned_t lp;
                            mavlink_msg_local_position_ned_decode(&msg, &lp);
                            std::lock_guard<std::mutex> lk(local_position_mutex_);
                            local_pos_.time_boot_ms = lp.time_boot_ms;
                            local_pos_.x = lp.x;
                            local_pos_.y = lp.y;
                            local_pos_.z = lp.z;
                            local_pos_.vx = lp.vx;
                            local_pos_.vy = lp.vy;
                            local_pos_.vz = lp.vz;
                            last_local_position_ns_.store(steadyNowNs());
                            break;
                        }
                        case MAVLINK_MSG_ID_SYSTEM_TIME: {  // 时间戳
                            count_system_time_.fetch_add(1);
                            mavlink_system_time_t st;
                            mavlink_msg_system_time_decode(&msg, &st);
                            std::lock_guard<std::mutex> lk(system_time_mutex_);
                            system_time_.time_unix_usec = st.time_unix_usec;
                            system_time_.time_boot_ms = st.time_boot_ms;
                            last_system_time_ns_.store(steadyNowNs());
                            break;
                        }
                        case MAVLINK_MSG_ID_HIGHRES_IMU: {  // IMU原数数据
                            count_highres_imu_.fetch_add(1);
                            mavlink_highres_imu_t imu;
                            mavlink_msg_highres_imu_decode(&msg, &imu);
                            const int64_t host_ns = steadyNowNs();

                            ImuData sample;
                            sample.timestamp_us = imu.time_usec;
                            sample.gx = imu.xgyro;
                            sample.gy = imu.ygyro;
                            sample.gz = imu.zgyro;
                            sample.ax = imu.xacc;
                            sample.ay = imu.yacc;
                            sample.az = imu.zacc;
                            {
                                std::lock_guard<std::mutex> lk(imu_mutex_);
                                imu_ = sample;  // 兼容旧接口：最新一帧
                            }
                            // 【2026-10-28】逐帧入队给 VIO。接收线程只做入队，
                            // 保持这里短小；VIO 的运算在它自己的线程里做。
                            const size_t cap = imu_queue_capacity_.load();
                            if (cap > 0) {
                                TimedImu item;
                                item.session = reconnects_.load();
                                item.imu = sample;
                                item.host_ns = host_ns;
                                std::lock_guard<std::mutex> lk(imu_queue_mutex_);
                                while (imu_queue_.size() >= cap) {
                                    imu_queue_.pop_front();
                                    dropped_imu_.fetch_add(1);
                                }
                                imu_queue_.push_back(item);
                            }
                            break;
                        }
                        default:
                            break;
                    }
                }
            }
        }

        // 每秒发一次 GCS 心跳，保持上位机"在线"状态（飞控看到 GCS 在线才持续推流）
        if (steady_clock::now() - last_gcs_heartbeat >= seconds(1)) {
            sendGcsHeartbeat();
            last_gcs_heartbeat = steady_clock::now();
        }

        if (n <= 0) {
            std::this_thread::sleep_for(milliseconds(1));
        }
    }
}

// 发送底层
bool Communicator::sendMessage(const uint8_t* buf, uint16_t len, bool require_connected, std::optional<uint64_t> expected_session) {
    std::lock_guard<std::mutex> lk(send_mutex_);
    if (require_connected && (!running_.load() || !connected_.load())) return false;
    if (expected_session && reconnects_.load() != *expected_session) return false;
    size_t off = 0;
    while (off < len) {
        ssize_t n = port_.write(buf + off, len - off);
        if (n > 0) {
            off += static_cast<size_t>(n);
        } else {
            return false;
        }
    }
    return true;
}

// 发一条 GCS 心跳（MAV_TYPE_GCS），宣告"上位机在线"
void Communicator::sendGcsHeartbeat() {
    mavlink_message_t msg{};
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    packMessage(mavlink_msg_heartbeat_pack, sysid_, compid_, &msg,
                MAV_TYPE_GCS,           // 地面站
                MAV_AUTOPILOT_INVALID,  // 不是飞控
                0,                      // base_mode
                0,                      // custom_mode
                MAV_STATE_ACTIVE);      // 在线
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendMessage(buf, len);
}

// 飞行模式 / ACK 结果 中文名映射
const char* Communicator::px4ModeNameCn(uint32_t custom_mode) const {
    const uint8_t main_mode = (custom_mode >> 16) & 0xFF;
    const uint8_t sub_mode = (custom_mode >> 24) & 0xFF;
    switch (main_mode) {
        case 1:
            return "手动";
        case 2:
            return "定高";
        case 3:
            if (sub_mode == 1) return "环绕";
            if (sub_mode == 2) return "慢速定点";
            return "定点";
        case 4:
            switch (sub_mode) {
                case 1:
                    return "自动·就绪";
                case 2:
                    return "自动·起飞";
                case 3:
                    return "自动·悬停";
                case 4:
                    return "自动·任务";
                case 5:
                    return "自动·返航";
                case 6:
                    return "自动·降落";
                case 8:
                    return "自动·跟随";
                case 9:
                    return "自动·精准降落";
                default:
                    return "自动";
            }
        case 5:
            return "特技";
        case 6:
            return "外部控制";
        case 7:
            return "自稳";
        case 10:
            return "故障终止";
        case 11:
            return "高度巡航";
        default:
            return "未知";
    }
}

const char* Communicator::arducopterModeNameCn(uint8_t custom_mode) const {
    switch (custom_mode) {
        case arducopter_mode::STABILIZE:
            return "自稳";
        case arducopter_mode::ACRO:
            return "特技";
        case arducopter_mode::ALT_HOLD:
            return "定高";
        case arducopter_mode::AUTO:
            return "自动任务";
        case arducopter_mode::GUIDED:
            return "引导";
        case arducopter_mode::LOITER:
            return "悬停";
        case arducopter_mode::RTL:
            return "返航";
        case arducopter_mode::CIRCLE:
            return "环绕";
        case arducopter_mode::LAND:
            return "降落";
        case arducopter_mode::POSHOLD:
            return "定点";
        case arducopter_mode::SMARTRTL:
            return "智能返航";
        default:
            return "未知";
    }
}

const char* Communicator::ackResultNameCn(uint8_t result) const {
    switch (result) {
        case MAV_RESULT_ACCEPTED:
            return "已接受";  // 0
        case MAV_RESULT_TEMPORARILY_REJECTED:
            return "临时拒绝";  // 1
        case MAV_RESULT_DENIED:
            return "拒绝";  // 2
        case MAV_RESULT_UNSUPPORTED:
            return "不支持";  // 3
        case MAV_RESULT_FAILED:
            return "失败";  // 3
        case MAV_RESULT_IN_PROGRESS:
            return "进行中";  // 4
        default:
            return "未知";
    }
}

// 各模块向飞控通讯
bool Communicator::pushVisionPose(const VisionPose& p) {
    return sendVisionPose(p, std::nullopt);
}
bool Communicator::pushVisionPoseForSession(const VisionPose& p, uint64_t expected_session) {
    return sendVisionPose(p, expected_session);
}
bool Communicator::sendVisionPose(const VisionPose& p, std::optional<uint64_t> expected_session) {  // vio数据
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    //=======================================================================
    // 【2026-10-28】把质量/重置/协方差一起发出去（原来是写死的 0 和 NaN）。
    //
    // 为什么必须真的发：
    //   · ArduPilot 的 AP_VisualOdom 用 quality 和 VISO_QUAL_MIN 比较，
    //     低于门限的帧直接丢（不进 EKF），VISP 日志里能看到 reject 标志；
    //   · reset_counter 变化时飞控会重锚定位置/速度/姿态。VIO 重建后位姿
    //     会跳变，不告诉飞控"我重置了"，它会把跳变当成真实运动。
    //
    // 协方差布局：6x6 上三角逐行，对角线下标 0/6/11/15/18/20。
    // 只填位置/速度对角；姿态没有估计值就保持 0（飞控会 constrain 到
    // VISO_YAW_M_NSE 之类的配置下限）。没给 sigma → NaN（MAVLink 约定 = 未知）。
    //=======================================================================
    float pose_cov[21] = {0.0f};
    float vel_cov[21] = {0.0f};
    if (p.pos_sigma_m > 0.0f) {
        pose_cov[0] = pose_cov[6] = pose_cov[11] = p.pos_sigma_m * p.pos_sigma_m;
    } else {
        // 未知：三个对角都标 NaN（只标 [0] 会让另外两个 0 被当成"完美已知"）
        pose_cov[0] = pose_cov[6] = pose_cov[11] = std::nanf("");
    }
    if (p.vel_sigma_mps > 0.0f) {
        vel_cov[0] = vel_cov[6] = vel_cov[11] = p.vel_sigma_mps * p.vel_sigma_mps;
    } else {
        vel_cov[0] = vel_cov[6] = vel_cov[11] = std::nanf("");
    }

    int quality = p.quality;
    if (quality < 0) quality = 0;
    if (quality > 100) quality = 100;

    // ⚠️ 最后三个参数的顺序是 (reset_counter, estimator_type, quality)，
    //    这是 MAVLink 生成函数的签名。写反了编译器不会报错（都是整数）。
    packMessage(mavlink_msg_odometry_pack, sysid_, compid_, &msg, p.timestamp_us, static_cast<uint8_t>(p.frame_id), static_cast<uint8_t>(p.child_frame_id), p.x, p.y, p.z, p.q, p.vx, p.vy, p.vz, 0.0f, 0.0f, 0.0f, pose_cov, vel_cov,
                p.reset_counter,                // 1) 重置计数
                MAV_ESTIMATOR_TYPE_VIO,         // 2) 估计器类型
                static_cast<int8_t>(quality));  // 3) 质量 0-100
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return sendMessage(buf, len, true, expected_session);
}

bool Communicator::pushGuidanceCmd(const GuidanceCmd& c) {
    return sendGuidanceCmd(c, std::nullopt);
}
bool Communicator::pushGuidanceCmdForSession(const GuidanceCmd& c, uint64_t expected_session) {
    return sendGuidanceCmd(c, expected_session);
}
bool Communicator::sendGuidanceCmd(const GuidanceCmd& c, std::optional<uint64_t> expected_session) {  // 发送制导律
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    //   制导律不输出航向，所有模式都完全忽略 yaw + yaw_rate
    //   POSITION：忽略速度 + 加速度，只发位置
    //   VELOCITY：忽略位置 + 加速度，只发速度
    //   POS_WITH_VEL_FEEDFORWARD：忽略加速度，位置 + 速度
    //   ACCELERATION：忽略位置 + 速度，只发加速度
    uint16_t type_mask = POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    if (c.mode == GuidanceMode::POSITION) {
        // 纯位置闭环：忽略速度 + 加速度
        type_mask |= POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE;
    } else if (c.mode == GuidanceMode::VELOCITY) {
        // 纯速度闭环：忽略位置 + 加速度
        type_mask |= POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE;
    } else if (c.mode == GuidanceMode::POS_WITH_VEL_FEEDFORWARD) {
        // 位置目标 + 速度前馈：忽略加速度
        type_mask |= POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE;
    } else if (c.mode == GuidanceMode::ACCELERATION) {
        // 加速度 setpoint：忽略位置 + 速度，只保留加速度
        type_mask |= POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE | POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE;
    } else {
        return false;  // 未知 mode，拒绝发送
    }
    packMessage(mavlink_msg_set_position_target_local_ned_pack, sysid_, compid_, &msg, timeBootMs(), fcu_sysid_.load(), fcu_compid_.load(), MAV_FRAME_LOCAL_NED, type_mask, c.x, c.y, c.z, c.vx, c.vy, c.vz, c.afx, c.afy, c.afz, 0.0f, 0.0f);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return sendMessage(buf, len, true, expected_session);
}

// 任务调度没有后台命令队列，取消后不会延迟执行旧解锁请求。
// 与同步 send* API 分开，避免 timeout_ms=0 把“已发出”误报为失败。
bool Communicator::requestFlightCommand(uint16_t command, float p1, float p2, uint64_t expected_session) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    packMessage(mavlink_msg_command_long_pack, sysid_, compid_, &msg, fcu_sysid_.load(), fcu_compid_.load(), command, 0, p1, p2, 0, 0, 0, 0, 0);
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return sendMessage(buf, len, true, expected_session);
}
bool Communicator::requestOffboardForSession(uint64_t expected_session) {
    if (autopilot_.load() != MAV_AUTOPILOT_PX4) return false;
    return requestFlightCommand(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, expected_session);
}
bool Communicator::requestArmForSession(uint64_t expected_session) {
    return requestFlightCommand(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, expected_session);
}
bool Communicator::requestLandForSession(uint64_t expected_session) {
    return requestFlightCommand(MAV_CMD_NAV_LAND, 0, 0, expected_session);
}
bool Communicator::getLandedState(LandedState& out, int max_age_ms) const {
    std::lock_guard<std::mutex> lock(landed_mutex_);
    if (!connected_.load() || !telemetryFresh(landed_.received_ns, max_age_ms) || landed_.state == 0) return false;
    out = landed_;
    return true;
}

bool Communicator::sendTakeoff(float altitude_m, int timeout_ms, uint8_t* result) {  // 命令PX4起飞到目标高度
    last_ack_command_.store(0);
    last_ack_result_.store(UINT8_MAX);
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    packMessage(mavlink_msg_command_long_pack, sysid_, compid_, &msg, fcu_sysid_.load(), fcu_compid_.load(), MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude_m);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (!sendMessage(buf, len, true)) return false;
    return waitForAck(MAV_CMD_NAV_TAKEOFF, timeout_ms, result);
}

bool Communicator::sendLand(int timeout_ms, uint8_t* result) {  // 命令PX4自动降落
    last_ack_command_.store(0);
    last_ack_result_.store(UINT8_MAX);
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    packMessage(mavlink_msg_command_long_pack, sysid_, compid_, &msg, fcu_sysid_.load(), fcu_compid_.load(), MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (!sendMessage(buf, len, true)) return false;
    return waitForAck(MAV_CMD_NAV_LAND, timeout_ms, result);
}

bool Communicator::sendSetMode(uint32_t custom_mode, int timeout_ms, uint8_t* result) {  // 设置飞行模式
    last_ack_command_.store(0);
    last_ack_result_.store(UINT8_MAX);
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    if (autopilot_.load() == MAV_AUTOPILOT_ARDUPILOTMEGA) {
        // ArduPilot：param2 直接填模式数字，param3 不用
        packMessage(mavlink_msg_command_long_pack, sysid_, compid_, &msg, fcu_sysid_.load(), fcu_compid_.load(), MAV_CMD_DO_SET_MODE, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode, 0, 0, 0, 0, 0);
    } else {
        // PX4：拆主/子模式填 param2/param3
        uint8_t main_mode = (custom_mode >> 16) & 0xFF;
        uint8_t sub_mode = (custom_mode >> 24) & 0xFF;
        packMessage(mavlink_msg_command_long_pack, sysid_, compid_, &msg, fcu_sysid_.load(), fcu_compid_.load(), MAV_CMD_DO_SET_MODE, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main_mode, sub_mode, 0, 0, 0, 0);
    }
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (!sendMessage(buf, len, true)) return false;
    return waitForAck(MAV_CMD_DO_SET_MODE, timeout_ms, result);
}

bool Communicator::sendArm(bool arm, int timeout_ms, uint8_t* result) {  // 解锁1/加锁0
    last_ack_command_.store(0);
    last_ack_result_.store(UINT8_MAX);
    // 普通解锁/加锁遵守飞控安全检查，不能默认使用强制解锁或空中强制加锁。
    const float force = 0.0f;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    packMessage(mavlink_msg_command_long_pack, sysid_, compid_, &msg, fcu_sysid_.load(), fcu_compid_.load(), MAV_CMD_COMPONENT_ARM_DISARM, 0, arm ? 1.0f : 0.0f, force, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (!sendMessage(buf, len, true)) return false;
    return waitForAck(MAV_CMD_COMPONENT_ARM_DISARM, timeout_ms, result);
}

bool Communicator::requestMessageInterval(uint32_t msgid, int32_t interval_us) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // MAV_CMD_SET_MESSAGE_INTERVAL(511)：param1=消息ID，param2=间隔(微秒，-1禁用，0默认)
    packMessage(mavlink_msg_command_long_pack, sysid_, compid_, &msg, fcu_sysid_.load(), fcu_compid_.load(), MAV_CMD_SET_MESSAGE_INTERVAL, 0, static_cast<float>(msgid), static_cast<float>(interval_us), 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return sendMessage(buf, len, true);
}

// 等某条命令的 COMMAND_ACK（带超时）。必须在 sendMessage 之后调用。
bool Communicator::waitForAck(uint16_t command, int timeout_ms, uint8_t* result) {
    using namespace std::chrono;
    const auto deadline = steady_clock::now() + milliseconds(timeout_ms);
    while (steady_clock::now() < deadline) {
        if (last_ack_command_.load() == command && last_ack_result_.load() != UINT8_MAX) {
            if (result) *result = last_ack_result_.load();
            return true;
        }
        std::this_thread::sleep_for(milliseconds(5));
    }
    return false;  // 超时
}

void Communicator::requestDefaultStreams() {
    // 向PX4请求以下数据（间隔来自 config/driver.yaml 的 stream 段）
    requestMessageInterval(MAVLINK_MSG_ID_ATTITUDE, cfg_.attitude_interval_us);
    requestMessageInterval(MAVLINK_MSG_ID_LOCAL_POSITION_NED, cfg_.local_position_interval_us);
    requestMessageInterval(MAVLINK_MSG_ID_HIGHRES_IMU, cfg_.highres_imu_interval_us);
    requestMessageInterval(MAVLINK_MSG_ID_BATTERY_STATUS, cfg_.battery_interval_us);
    requestMessageInterval(MAVLINK_MSG_ID_SYSTEM_TIME, cfg_.system_time_interval_us);
    requestMessageInterval(MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 1000000);
}

// 各模块拿数据
bool Communicator::getAttitude(Attitude& out) const {
    return getAttitude(out, -1);
}

bool Communicator::getAttitude(Attitude& out, int max_age_ms) const {
    return getAttitude(out, max_age_ms, nullptr);
}
bool Communicator::getAttitude(Attitude& out, int max_age_ms, int64_t* received_ns) const {
    std::lock_guard<std::mutex> lk(attitude_mutex_);
    if (!connected_.load() || !telemetryFresh(last_attitude_ns_.load(), max_age_ms)) return false;
    if (received_ns) *received_ns = last_attitude_ns_.load();
    out = attitude_;
    return true;
}

bool Communicator::getBattery(Battery& out) const {
    std::lock_guard<std::mutex> lk(battery_mutex_);
    out = battery_;
    return true;
}

bool Communicator::getLocalPosition(LocalPosition& out) const {
    return getLocalPosition(out, -1);
}

bool Communicator::getLocalPosition(LocalPosition& out, int max_age_ms) const {
    return getLocalPosition(out, max_age_ms, nullptr);
}
bool Communicator::getLocalPosition(LocalPosition& out, int max_age_ms, int64_t* received_ns) const {
    std::lock_guard<std::mutex> lk(local_position_mutex_);
    if (!connected_.load() || !telemetryFresh(last_local_position_ns_.load(), max_age_ms)) return false;
    if (received_ns) *received_ns = last_local_position_ns_.load();
    out = local_pos_;
    return true;
}

bool Communicator::getImu(ImuData& out) const {
    std::lock_guard<std::mutex> lk(imu_mutex_);
    out = imu_;
    return true;
}

// 【2026-10-28】逐帧 IMU 交付给 VIO。
bool Communicator::popImu(TimedImu& out) {
    std::lock_guard<std::mutex> lk(imu_queue_mutex_);
    if (imu_queue_.empty()) return false;
    out = imu_queue_.front();
    imu_queue_.pop_front();
    return true;
}

size_t Communicator::imuQueueSize() const {
    std::lock_guard<std::mutex> lk(imu_queue_mutex_);
    return imu_queue_.size();
}

void Communicator::setImuQueueCapacity(size_t cap) {
    {
        std::lock_guard<std::mutex> lk(imu_queue_mutex_);
        imu_queue_capacity_.store(cap);
        // 缩小容量时立刻裁掉多余部分，避免"设置了却还占着内存"
        while (imu_queue_.size() > cap) {
            imu_queue_.pop_front();
            dropped_imu_.fetch_add(1);
        }
    }
}

int64_t Communicator::lastHeartbeatAgeMs() const {
    const int64_t last = last_heartbeat_ns_.load();
    if (last == 0) return -1;
    return (steadyNowNs() - last) / 1000000;
}

bool Communicator::getSystemTime(SystemTime& out) const {
    return getSystemTime(out, -1);
}

bool Communicator::getSystemTime(SystemTime& out, int max_age_ms) const {
    std::lock_guard<std::mutex> lk(system_time_mutex_);
    if (!connected_.load() || !telemetryFresh(last_system_time_ns_.load(), max_age_ms)) return false;
    out = system_time_;
    return true;
}

int64_t Communicator::attitudeAgeMs() const {
    std::lock_guard<std::mutex> lock(attitude_mutex_);
    const auto stamp = last_attitude_ns_.load();
    return connected_.load() && stamp != 0 ? (steadyNowNs() - stamp) / 1000000LL : -1;
}

int64_t Communicator::localPositionAgeMs() const {
    std::lock_guard<std::mutex> lock(local_position_mutex_);
    const auto stamp = last_local_position_ns_.load();
    return connected_.load() && stamp != 0 ? (steadyNowNs() - stamp) / 1000000LL : -1;
}

int64_t Communicator::systemTimeAgeMs() const {
    std::lock_guard<std::mutex> lock(system_time_mutex_);
    const auto stamp = last_system_time_ns_.load();
    return connected_.load() && stamp != 0 ? (steadyNowNs() - stamp) / 1000000LL : -1;
}

const char* Communicator::getFlightModeNameCn() const {
    if (autopilot_.load() == MAV_AUTOPILOT_ARDUPILOTMEGA) {
        return arducopterModeNameCn(static_cast<uint8_t>(custom_mode_.load()));
    }
    return px4ModeNameCn(custom_mode_.load());  // 默认按 PX4
}

const char* Communicator::getAutopilotNameCn() const {
    switch (autopilot_.load()) {
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            return "ArduPilot";
        case MAV_AUTOPILOT_PX4:
            return "PX4";
        default:
            return "未知";
    }
}

const char* Communicator::getLastAck() const {
    const uint8_t result = last_ack_result_.load();
    if (result == UINT8_MAX) return "未收到";  // 尚未收到 ACK
    return ackResultNameCn(result);
}

// 从 YAML 加载通信配置（config/driver.yaml）
bool loadCommConfig(const std::string& yaml_path, CommConfig& cfg) {
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root.IsMap() || !root["communicate"]) {
            return false;  // 没有 communicate 段
        }
        YAML::Node comm = root["communicate"];
        for (const char* name : {"fcu_sysid", "fcu_compid"}) {
            if (!comm[name]) continue;
            const int id = comm[name].as<int>();
            if (id < 0 || id > 255) return false;
            if (std::strcmp(name, "fcu_sysid") == 0)
                cfg.fcu_sysid = static_cast<uint8_t>(id);
            else
                cfg.fcu_compid = static_cast<uint8_t>(id);
        }
        if (comm["file"]) cfg.device = comm["file"].as<std::string>();
        if (comm["device"]) cfg.device = comm["device"].as<std::string>();
        if (comm["port"]) cfg.device = comm["port"].as<std::string>();
        if (comm["baudrate"]) cfg.baudrate = comm["baudrate"].as<int>();
        if (comm["baud"]) cfg.baudrate = comm["baud"].as<int>();
        if (comm["heartbeat_timeout_ms"]) cfg.heartbeat_timeout_ms = comm["heartbeat_timeout_ms"].as<int>();
        if (comm["stream"]) {
            YAML::Node s = comm["stream"];
            if (s["attitude_us"]) cfg.attitude_interval_us = s["attitude_us"].as<int>();
            if (s["local_position_us"]) cfg.local_position_interval_us = s["local_position_us"].as<int>();
            if (s["highres_imu_us"]) cfg.highres_imu_interval_us = s["highres_imu_us"].as<int>();
            if (s["battery_us"]) cfg.battery_interval_us = s["battery_us"].as<int>();
            if (s["system_time_us"]) cfg.system_time_interval_us = s["system_time_us"].as<int>();
        }
        return true;
    } catch (const YAML::Exception& e) {
        fprintf(stderr, "加载 yaml 配置失败: %s\n", e.what());
        return false;
    }
}

// 自动查找配置文件
std::string findConfigFile(const std::string& rel_path) {
    // 1. 当前工作目录直接找
    if (access(rel_path.c_str(), F_OK) == 0) return rel_path;
    // 2. 从可执行文件位置逐层往上找
    char buf[PATH_MAX];
    ssize_t n = readlink("/proc/self/exe", buf, PATH_MAX);
    if (n > 0 && n < (ssize_t)PATH_MAX) {
        buf[n] = '\0';
        std::string dir(buf);
        dir = dir.substr(0, dir.find_last_of('/'));
        for (int i = 0; i < 6; i++) {
            std::string cand = dir + "/" + rel_path;
            if (access(cand.c_str(), F_OK) == 0) return cand;
            auto pos = dir.find_last_of('/');
            if (pos == std::string::npos) break;
            dir = dir.substr(0, pos);
        }
    }

    return rel_path;
}

}  // namespace pure_uav
