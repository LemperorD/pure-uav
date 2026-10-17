#include "communicator.hpp"
#include <cstdio>
#include <unistd.h>

int main() {
    pure_uav::Communicator comm;
    comm.setSystemId(2);  

    pure_uav::CommConfig cfg;
    pure_uav::loadCommConfig(pure_uav::findConfigFile(), cfg);  // 自动找配置，失败就用默认值
    if (!comm.start(cfg)) {  // 飞控 USB 虚拟串口
        printf("打开串口失败\n");
        return 1;
    }
    printf("等待飞控心跳...\n");

    while (true) {
        printf("[固件=%s 连接=%d 模式=%s(0x%08X) ACK=%s(%u)]\n",
               comm.getAutopilotNameCn(), comm.isConnected(), comm.getFlightModeNameCn(), comm.getCustomMode(),
               comm.getLastAck(), comm.lastAckResult());

        pure_uav::Attitude a;
        if (comm.getAttitude(a))
            printf("  姿态: roll=%.3f pitch=%.3f yaw=%.3f | 四元数wxyz=(%.3f %.3f %.3f %.3f)\n",
                   a.roll, a.pitch, a.yaw, a.q[0], a.q[1], a.q[2], a.q[3]);

        pure_uav::Battery b;
        if (comm.getBattery(b))
            printf("  电量: %.2fV %d%%\n", b.voltage, b.remaining);

        pure_uav::LocalPosition lp;
        if (comm.getLocalPosition(lp))
            printf("  位置: x=%.2f y=%.2f z=%.2f | 速度: vx=%.2f vy=%.2f vz=%.2f\n",
                   lp.x, lp.y, lp.z, lp.vx, lp.vy, lp.vz);

        pure_uav::ImuData imu;
        if (comm.getImu(imu))
            printf("  IMU: 角速度=(%.3f %.3f %.3f) 加速度=(%.3f %.3f %.3f) ts=%llu\n",
                   imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az,
                   (unsigned long long)imu.timestamp_us);

        pure_uav::SystemTime st;
        if (comm.getSystemTime(st))
            printf("  时间: unix=%llu us  boot=%u ms\n",
                   (unsigned long long)st.time_unix_usec, st.time_boot_ms);

        printf("----------------------------------------\n");
        usleep(500000);  // 500ms
    }
    return 0;
}
