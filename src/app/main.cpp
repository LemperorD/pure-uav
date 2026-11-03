#include "communicator.hpp"

#include <cstdio>
#include <unistd.h>

using namespace pure_uav;

int main() {
    Communicator comm;
    comm.setSystemId(2);   // 上位机 system id

    CommConfig cfg;
    loadCommConfig(findConfigFile(), cfg);   // 自动找 config/driver.yaml，失败用默认值
    if (!comm.start(cfg)) {
        std::printf("打开串口失败\n");
        return 1;
    }

    std::printf("等待飞控心跳...\n");
    while (!comm.isConnected()) usleep(100000);
    std::printf("已连接，固件：%s，模式：%s\n",
                comm.getAutopilotNameCn(), comm.getFlightModeNameCn());

    // 读一次姿态示例
    Attitude a;
    if (comm.getAttitude(a)) {
        std::printf("姿态 roll=%.3f pitch=%.3f yaw=%.3f\n",
                    a.roll, a.pitch, a.yaw);
    }
    return 0;
}
