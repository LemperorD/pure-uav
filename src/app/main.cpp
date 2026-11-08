#include <unistd.h>

#include <cstdio>

#include "communicator.hpp"
#include "pure_uav/detection/detector.hpp"
#include "pure_uav/detection/target_bridge.hpp"

using namespace pure_uav;

int main() {
    Communicator comm;
    comm.setSystemId(2);  // 上位机 system id

    CommConfig cfg;
    loadCommConfig(findConfigFile(), cfg);  // 自动找 config/driver.yaml，失败用默认值
    if (!comm.start(cfg)) {
        std::printf("打开串口失败\n");
        return 1;
    }

    std::printf("等待飞控心跳...\n");
    while (!comm.isConnected()) {
        usleep(100000);
    }
    std::printf("已连接，固件：%s，模式：%s\n", comm.getAutopilotNameCn(), comm.getFlightModeNameCn());

    // 读一次姿态示例
    Attitude a;
    if (comm.getAttitude(a)) {
        std::printf("姿态 roll=%.3f pitch=%.3f yaw=%.3f\n", a.roll, a.pitch, a.yaw);
    }

    detection::DetectorConfig det_cfg;
#ifdef PURE_DETECTION_BACKEND_CPU
    det_cfg.model_path = "assets/models/yolo26n.onnx";
#else
    det_cfg.model_path = "assets/models/yolo26n.nb";
#endif

    detection::Detector det;
    std::string err;
    if (!det.start(det_cfg, err)) {
        std::printf("目标检测启动失败：%s\n", err.c_str());
        return 1;
    }

    detection::FrameResult r;
    if (det.poll(r)) {  // 非阻塞；只返回最新结果
        // boot_us：用通信模块 SystemTime 把 r.host_us 换算到飞控 boot 时钟
        uint64_t boot_us = 0;
        SystemTime st;
        if (comm.getSystemTime(st) && st.time_unix_usec > 0) {
            const int64_t offset_us = static_cast<int64_t>(st.time_unix_usec) - (static_cast<int64_t>(st.time_boot_ms) * 1000);
            if (offset_us >= 0 && r.host_us > static_cast<uint64_t>(offset_us)) {
                boot_us = r.host_us - static_cast<uint64_t>(offset_us);
            }
        }
        guidance::flight::GuidanceTargetSample t;
        detection::to_target_sample(r, boot_us, t);
        // t.present=false ⇒ 主循环可进入 SEARCH
    }
    det.stop();  // 停线程、释放相机和推理后端
    return 0;
}
