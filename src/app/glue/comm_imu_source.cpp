#include "glue/comm_imu_source.hpp"

#include <chrono>
#include <thread>
namespace pure_uav::app {
HeartbeatWaitResult waitForHeartbeat(const Communicator& comm, const std::atomic<bool>& stop, double timeout_s) {
    using Clock = std::chrono::steady_clock;
    const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);
    while (!stop.load() && !comm.isConnected() && Clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (stop.load()) return HeartbeatWaitResult::stopped;
    return comm.isConnected() ? HeartbeatWaitResult::connected : HeartbeatWaitResult::timeout;
}
bool CommImuSource::pop(TimedImu& out) {
    Communicator::TimedImu item;
    if (!comm_.popImu(item)) return false;
    out.imu.timestamp_us = item.imu.timestamp_us;
    out.imu.gx = item.imu.gx;
    out.imu.gy = item.imu.gy;
    out.imu.gz = item.imu.gz;
    out.imu.ax = item.imu.ax;
    out.imu.ay = item.imu.ay;
    out.imu.az = item.imu.az;
    out.host_ns = item.host_ns;
    out.session = item.session;
    return true;
}
}  // namespace pure_uav::app
