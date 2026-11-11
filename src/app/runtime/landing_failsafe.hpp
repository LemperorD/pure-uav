#pragma once

#include <atomic>
#include <functional>

namespace pure_uav {
class Communicator;
}

namespace pure_uav::app {

/// 失去相机/VIO 或任务中止后的最后一道控制权回收：
/// 以固定间隔发送 NAV_LAND，直到飞控确认 landed && !armed 或超时。
/// 不启动线程；由 main 在进入故障分支时同步调用。
/// keepalive 在等待期间持续调用，用于保持 VIO ODOMETRY 发布；可为空。
bool runLandingFailsafe(Communicator& comm, double timeout_s, double request_interval_s, const std::atomic<bool>& interrupted, const std::function<bool()>& keepalive = {});

}  // namespace pure_uav::app
