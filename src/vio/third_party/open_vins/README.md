# OpenVINS（裁剪，ROS-free）

来源：`~/vio_ws/src/open_vins`（OpenVINS v2.7.0，和 session05 评估所用版本一致；
仅 ROS2Visualizer 的 QoS 补丁不影响本工程，因为 ROS 源已被裁掉）。

保留：
- `ov_core/`  特征跟踪、相机模型、数学工具
- `ov_init/`  静态/动态初始化（含 Ceres 因子）
- `ov_msckf/`  MSCKF 滤波器

去掉：ROS1/ROS2 节点、仿真器、可视化、ov_eval、ov_data、docs。

许可证：GPLv3（见 `LICENSE`）。本工程只做内部集成；对外分发二进制时注意 GPL 义务。
