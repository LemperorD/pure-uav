# detect 与 VIO 合并记录

日期：2026-11-10。

本次从 `main` 的 `ca86226` 建立 `codex/merge-detect-vio`，合入
`jianyuewushuang/pure-uav` 的 `detect` 分支 `dfdacbc`。
远程 detect 的文件树与本地 `origin/detect`（`894b112`）相同；共同祖先为 `73f9c6e`。
合并保留两边历史，不以下载文件夹整体覆盖仓库。

## 冲突处理

| 文件 | 处理方式 |
|---|---|
| `src/app/main.cpp` | 保留 VIO 配置、通信、时基与发布门控，接入检测子进程启动和独立监督线程 |
| `src/vio/estimator/apps/vio_live_main.cpp` | 保留经过校验的 ShmCameraSource 输入，避免重复启动两个共享内存读取线程 |
| `src/vio/estimator/CMakeLists.txt` | 保留 pure_vio_runtime 与输入测试；相机服务复用 vio_camera 驱动 |
| `src/test/CMakeLists.txt` | 保留 VIO/共享内存测试，并加入检测配置、进程管理和异步监督测试 |
| `assets/docs/app/camera-ipc.md` | 统一相机服务、VIO 和检测进程的实际分工、启动和停止顺序 |
| `assets/docs/app/usage.md` | 保留 app.yaml/vio.yaml 分工，补充检测进程参数与退出行为 |
| `assets/docs/app/README.md` | 同步依赖方向和当前入口能力 |

Git 自动合并的 `src/common/CMakeLists.txt` 中另有重复 `pure_camera_ipc` 定义，已去重。
相机服务、检测算法配置与彩色通道消费者均从 detect 分支合入。
`main` 不链接推理实现，通过 fork/exec 运行 `detect_rt`；VIO 继续使用同一个 Communicator。

## 行为

先启动 `camera_server`，再运行 `main`。心跳、相机通道和 VIO 就绪后启动检测进程，
监督循环放在独立线程中，不阻塞 VIO 位姿发布。检测异常最多重启 3 次；检测正常结束
或重启超限后 main 结束。main 停止、相机掉线或 VIO 故障时均回收检测子进程。
`--no-detection` 保留仅 VIO 的验收路径。相机服务继续由外部托管。

`main` 和 `vio_live` 二选一使用同一个飞控串口，不应一起启动。
制导控制和应用状态机未在此次合并中接入。实际命令见 [camera-ipc.md](camera-ipc.md)。

## 验证与限制

- 全仓库构建通过，包含 main、camera_server、vio_live、detect_rt。
- 29 项 CTest 中 27 项通过。guidance 与 state_adapter 两项失败。
- 用合并前 `ca86226` 的制导源码在独立目录重新编译并测试，两项失败完全复现：
  guidance 为 288 个断言通过、50 个失败，state_adapter 为 1218 个通过、3 个失败。
  此次合并未修改制导算法与这两个测试，保留原有问题。
- 应用配置测试包含 46 项 CLI/YAML 校验，均不打开实际设备。
- 通过模拟飞控（PTY）、真实 V1 共享内存写者和假检测子进程验证实际 main：
  VIO 持续处理图像期间检测进程仍在运行；SIGTERM 正常退出且回收检测进程；
  相机通道关闭时 main 返回失败并回收检测进程。
- 三个程序的配置校验从 `/tmp` 执行通过；四个程序均无缺失动态库。
  main 和 vio_live 不依赖 OrbbecSDK 动态库。
- 未连接真实相机/飞控，也未验证实机检测精度、定位精度或 EKF 融合。
