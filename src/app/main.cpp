#include "communicator.hpp"

#include "pure_uav/detection/detector.hpp"
#include "pure_uav/detection/target_bridge.hpp"

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

    detection::DetectorConfig cfg;
    cfg.model_path = "assets/models/yolo26n.nb";   // 其余用默认值

    detection::Detector det;
    std::string err;
    if (!det.start(cfg, err)) { /* err 含相机/NBG 失败原因 */ }

    detection::FrameResult r;
    if (det.poll(r)) {                              // 非阻塞；只返回最新结果
        app::TargetSample t;
        // boot_us：用通信模块 SystemTime 把 r.host_us 换算到飞控 boot 时钟
        detection::to_target_sample(r, boot_us, t);
        // t.present=false ⇒ 主循环可进入 SEARCH
    }
    det.stop();                                     // 停线程、释放相机和 awnn 上下文
    return 0;
}
