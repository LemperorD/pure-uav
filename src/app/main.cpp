#include "communicator.hpp"

#include <cstdio>
#include <unistd.h>


using Comm = pure_uav::Communicator;
using CommConfig = pure_uav::CommConfig;
using Attitude = pure_uav::Attitude;
using Battery = pure_uav::Battery;
using LocalPosition = pure_uav::LocalPosition;
using ImuData = pure_uav::ImuData;
using SystemTime = pure_uav::SystemTime;


using pure_uav::loadCommConfig;
using pure_uav::findConfigFile;

int main() {
    Comm comm;
    comm.setSystemId(2);   // 上位机 system id

    CommConfig cfg;
    loadCommConfig(findConfigFile(), cfg);   // 自动找 config/driver.yaml，失败用默认值
    if (!comm.start(cfg)) {
        printf("打开串口失败\n");
        return 1;
    }

    printf("等待飞控心跳...\n");
    while (!comm.isConnected()) usleep(100000);
    printf("已连接，固件：%s，模式：%s\n",
           comm.getAutopilotNameCn(), comm.getFlightModeNameCn());

    // 读一次姿态示例
    Attitude a;
    if (comm.getAttitude(a)) {
        printf("姿态 roll=%.3f pitch=%.3f yaw=%.3f ts=%u ms\n",
               a.roll, a.pitch, a.yaw, a.time_boot_ms);
    }
    return 0;
}
