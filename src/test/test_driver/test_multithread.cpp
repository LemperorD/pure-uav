#include "communicator.hpp"

#include <atomic>
#include <cstdio>
#include <thread>
#include <unistd.h>

// 多线程读频率测试：验证「每种数据一把锁」下，多线程并发读互不阻塞。
// 5 个线程各读一种数据，紧密循环统计 getter 调用次数，输出每个线程的读吞吐。
int main() {
    pure_uav::Communicator comm;
    comm.setSystemId(2);

    pure_uav::CommConfig cfg;
    pure_uav::loadCommConfig(pure_uav::findConfigFile(), cfg);
    if (!comm.start(cfg)) {
        printf("打开串口失败\n");
        return 1;
    }

    printf("等待飞控心跳...\n");
    while (!comm.isConnected()) usleep(100000);
    printf("已连接，固件：%s，开始多线程读频率测试（50 秒）...\n", comm.getAutopilotNameCn());

    const int run_sec = 50;
    std::atomic<bool> stop{false};
    std::atomic<uint64_t> cnt_att{0}, cnt_bat{0}, cnt_pos{0}, cnt_imu{0}, cnt_tm{0};

    // 5 个线程，每个读一种数据，紧密循环统计读次数
    std::thread t_att([&] {
        pure_uav::Attitude a;
        while (!stop.load()) { comm.getAttitude(a); cnt_att.fetch_add(1); }
    });
    std::thread t_bat([&] {
        pure_uav::Battery b;
        while (!stop.load()) { comm.getBattery(b); cnt_bat.fetch_add(1); }
    });
    std::thread t_pos([&] {
        pure_uav::LocalPosition p;
        while (!stop.load()) { comm.getLocalPosition(p); cnt_pos.fetch_add(1); }
    });
    std::thread t_imu([&] {
        pure_uav::ImuData imu;
        while (!stop.load()) { comm.getImu(imu); cnt_imu.fetch_add(1); }
    });
    std::thread t_tm([&] {
        pure_uav::SystemTime st;
        while (!stop.load()) { comm.getSystemTime(st); cnt_tm.fetch_add(1); }
    });

    sleep(run_sec);
    stop.store(true);
    t_att.join();
    t_bat.join();
    t_pos.join();
    t_imu.join();
    t_tm.join();

    printf("\n========== 多线程读频率（%d 秒）==========\n", run_sec);
    printf("%-8s %14s %14s\n", "线程", "读次数", "读频率(次/秒)");
    printf("%-8s %14llu %14.0f\n", "姿态", (unsigned long long)cnt_att.load(), cnt_att.load() / (double)run_sec);
    printf("%-8s %14llu %14.0f\n", "电量", (unsigned long long)cnt_bat.load(), cnt_bat.load() / (double)run_sec);
    printf("%-8s %14llu %14.0f\n", "位置", (unsigned long long)cnt_pos.load(), cnt_pos.load() / (double)run_sec);
    printf("%-8s %14llu %14.0f\n", "IMU", (unsigned long long)cnt_imu.load(), cnt_imu.load() / (double)run_sec);
    printf("%-8s %14llu %14.0f\n", "时间", (unsigned long long)cnt_tm.load(), cnt_tm.load() / (double)run_sec);
    printf("============================================\n");

    return 0;
}
