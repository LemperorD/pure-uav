#include "communicator.hpp"
#include <cstdio>
#include <unistd.h>

int main() {
    pure_uav::Communicator comm;
    comm.setSystemId(2);  

    pure_uav::CommConfig cfg;
    pure_uav::loadCommConfig("config/driver.yaml", cfg);  // 失败就用默认值
    if (!comm.start(cfg)) {  // 飞控 USB 虚拟串口
        printf("打开串口失败\n");
        return 1;
    }
    printf("等待飞控心跳...\n");

    while (true) {
        printf("连接=%d 模式=%s(0x%X) ACK=%u | ",
               comm.isConnected(), comm.getFlightModeNameCn(), comm.getCustomMode(), comm.lastAckResult());

        pure_uav::Attitude a;
        if (comm.getAttitude(a))
            printf("姿态 roll=%.2f pitch=%.2f yaw=%.2f | ", a.roll, a.pitch, a.yaw);

        pure_uav::Battery b;
        if (comm.getBattery(b))
            printf("电量=%.1fV %d%% | ", b.voltage, b.remaining);

        pure_uav::LocalPosition lp;
        if (comm.getLocalPosition(lp))
            printf("位置 x=%.2f y=%.2f z=%.2f | ", lp.x, lp.y, lp.z);

        pure_uav::ImuData imu;
        if (comm.getImu(imu))
            printf("IMU gx=%.2f ts=%llu | ", imu.gx, (unsigned long long)imu.timestamp_us);

        printf("\n");
        usleep(500000);  // 500ms
    }
    return 0;
}
