#include "communicator.hpp"
#include <cstdio>
#include <unistd.h>

int main() {
    pure_uav::Communicator comm;
    comm.setSystemId(2);

    pure_uav::CommConfig cfg;
    if (!pure_uav::loadCommConfig(pure_uav::findConfigFile(), cfg)) {
        printf("加载配置失败，使用默认值 %s @ %d\n", cfg.device.c_str(), cfg.baudrate);
    }
    if (!comm.start(cfg)) {
        printf("打开串口失败\n");
        return 1;
    }

    printf("等待心跳...\n");
    while (!comm.isConnected()) usleep(100000);
    printf("已连接，固件：%s，当前模式：%s (0x%08X)\n",
           comm.getAutopilotNameCn(), comm.getFlightModeNameCn(), comm.getCustomMode());

    // 离板控制模式：ArduPilot=GUIDED(4)，PX4=OFFBOARD(0x00060000)
    uint32_t guided_mode = (comm.getAutopilot() == 3)  // 3 = MAV_AUTOPILOT_ARDUPILOTMEGA
                           ? pure_uav::arducopter_mode::GUIDED
                           : pure_uav::px4_mode::OFFBOARD;
    printf("发送切模式指令 → 离板控制（GUIDED/Offboard）...\n");
    uint8_t result = UINT8_MAX;
    if (comm.sendSetMode(guided_mode, 2000, &result)) {
        printf("收到 ACK：%s(%u)\n", comm.getLastAck(), result);
    } else {
        printf("切模式失败或超时（未收到 ACK）\n");
    }
    printf("切后模式：%s (0x%08X)\n",
           comm.getFlightModeNameCn(), comm.getCustomMode());
    return 0;
}
