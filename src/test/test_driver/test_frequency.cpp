#include "communicator.hpp"
#include <cstdio>
#include <unistd.h>

int main() {
    pure_uav::Communicator comm;
    comm.setSystemId(2);

    pure_uav::CommConfig cfg;
    if (!pure_uav::loadCommConfig(pure_uav::findConfigFile(), cfg)) {
        printf("加载配置失败，使用默认值\n");
    }
    if (!comm.start(cfg)) {
        printf("打开串口失败\n");
        return 1;
    }

    printf("等待飞控心跳...\n");
    while (!comm.isConnected()) usleep(100000);
    printf("已连接，固件：%s，开始统计 100 秒...\n", comm.getAutopilotNameCn());

    const double window_sec = 100.0;

    // 读初始计数
    uint32_t c0_att = comm.getCountAttitude();
    uint32_t c0_pos = comm.getCountLocalPosition();
    uint32_t c0_imu = comm.getCountImu();
    uint32_t c0_bat = comm.getCountBattery();
    uint32_t c0_tm  = comm.getCountSystemTime();

    // 睡 100 秒，分 10 段打印进度
    for (int i = 0; i < 10; i++) {
        sleep(10);
        printf("进度 %d/100 秒\n", (i + 1) * 10);
    }

    // 读结束计数
    uint32_t c1_att = comm.getCountAttitude();
    uint32_t c1_pos = comm.getCountLocalPosition();
    uint32_t c1_imu = comm.getCountImu();
    uint32_t c1_bat = comm.getCountBattery();
    uint32_t c1_tm  = comm.getCountSystemTime();

    // 实际频率 = 帧数增量 / 窗口秒数
    double f_att = (c1_att - c0_att) / window_sec;
    double f_pos = (c1_pos - c0_pos) / window_sec;
    double f_imu = (c1_imu - c0_imu) / window_sec;
    double f_bat = (c1_bat - c0_bat) / window_sec;
    double f_tm  = (c1_tm  - c0_tm)  / window_sec;

    // 设置频率 = 1e6 / 间隔(微秒)
    double s_att = 1e6 / cfg.attitude_interval_us;
    double s_pos = 1e6 / cfg.local_position_interval_us;
    double s_imu = 1e6 / cfg.highres_imu_interval_us;
    double s_bat = 1e6 / cfg.battery_interval_us;
    double s_tm  = 1e6 / cfg.system_time_interval_us;

    printf("\n========== 频率统计结果（窗口 %.0f 秒）==========\n", window_sec);
    printf("%-18s %10s %10s\n", "消息", "实际(Hz)", "设置(Hz)");
    printf("%-18s %10.2f %10.1f\n", "姿态 attitude",     f_att, s_att);
    printf("%-18s %10.2f %10.1f\n", "位置 local_pos",    f_pos, s_pos);
    printf("%-18s %10.2f %10.1f\n", "IMU highres_imu",   f_imu, s_imu);
    printf("%-18s %10.2f %10.1f\n", "电量 battery",      f_bat, s_bat);
    printf("%-18s %10.2f %10.1f\n", "时间 system_time",  f_tm,  s_tm);
    printf("================================================\n");

    return 0;
}
