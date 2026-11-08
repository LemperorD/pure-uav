# 验收遗留问题修复记录

日期：2026-11-05。所有源码修改与构建均在测试副本中进行。
未打开真实相机、串口或 NPU；未修改原工程、模型、生产标定链和 Orbbec SDK 库。

本记录只说明本轮实际修复与软件验证，不表示真机/飞行已完成验收。

## P0：全量构建复现与恢复

先新建此前不存在的 build-fix，执行全量构建，第一处失败确认为
test_state_adapter.cpp 的作用域提前结束，cases/worst 不可见，随后产生级联语法错误。
日志：build-fix/initial-configure.log、build-fix/initial-full-build.log。

没有覆盖参考分支文件，也没有删测试或放宽断言。修复额外花括号、残留旧类型名，
以及重复拼接测试块中引用错误作用域统计量的汇总输出。
适配器测试现有 1221 个断言通过、0 失败。

为避免二十路 C++ 编译在约 9 GB 可用内存下先触发 OOM，本轮构建统一用 -j2。
这是并行度调整，不改变构建目标或测试集合。

最终从 clean-first 全量构建，显式禁用没有真实依赖的 NPU 分支：

```bash
cmake -S . -B build-fix -DCMAKE_BUILD_TYPE=Release -DPURE_DETECTION_BUILD=OFF
cmake --build build-fix --clean-first -j2
ctest --test-dir build-fix --output-on-failure
```

结果：全量构建成功，项目代码无编译告警；MAVLink 第三方生成头既有 packed-member 告警保留。
全部注册 CTest 为 27 项，27/27 通过，没有用 -R 排除失败项，没有移除/跳过失败测试。
相机、通信频率、多线程台架等可执行目标参与构建，但会访问设备的程序原本就不注册到 CTest，未执行它们。
需要离线数据集的两项测试按既有 PURE_VIO_TEST_DATASET 条件注册，本轮未提供数据集，不计入 27 项。

## P1：通信取数新鲜度

- attitude、local_position、system_time 各保存本会话最近接收的 steady_clock 纳秒。
- start/stop、心跳超时、设备挂断、读错误、重新打开设备时清除这三个收到标志。
- 原单参数 getter 保留；新增带 max_age_ms 的重载，负数表示不限制年龄，过期/未收到/断线返回 false。
- getter 失败不改写输出，AgeMs 未收到/失联返回 -1；消息中的飞控采样时间戳不被当作主机接收时刻。
- pushVisionPose、会话复核发送、popImu 及会话计数语义未改动。
- loadCommConfig 现在也解析并校验 fcu_sysid/fcu_compid，不再仅新 app_config 生效。

test_telemetry_freshness 通过真实 Communicator + PTY 验证初始无数据、收到数据、年龄限制、
心跳超时、新会话、物理挂断/重开、stop/start、输出不变及身份配置。

注意：Communicator 类增加成员改变了布局，必须全量重编译所有使用者。
本轮分批编译时一次旧 vision_pose_wire 二进制与新库混用曾崩溃；全量重建后通过，
最终 clean-first 构建和完整 CTest 不存在混用产物。不要直接替换新 libcommunicate.so 而继续用旧可执行文件。

## P1：制导库输入正确性（未接入 main）

- GuidanceState 增加 has_range，adapter 显式保留缺失测距语义，不再把缺失与真实零距离等同。
  该字段默认 true 兼容原先手工填 range 的核心调用，外部原始输入必须由 adapter 填 has_range。
- PPN 缺距离时使用 LOS 差分；三种冻结判据均不会因缺失测距而触发 HOLD。
  t_go 使用有限最大值表示不可用，不凭空估计真实剩余时间。
- TPN/APN 缺距离时 health=kNoRange、valid=false；核心函数也拒绝这类输入。
- 像素反投影前检查 has_attitude，缺失时 health=kNoAttitude，不能以默认单位四元数冒充有效姿态。
- INS/目标时间为 0 拒绝；目标领先 INS 超过 50 ms 拒绝；新目标时间倒退也拒绝。
  50 ms 容差包含边界，仅允许小幅异步，不把任意未来采样当作年龄 0。
- 这些硬输入故障不会以保持上一条好指令的形式继续报告 valid=true。
  缺姿态无论降级策略如何均不调用 sink，直接返回输入异常；kStop 也不调用 sink。
  其它硬故障按既有非停止策略输出 invalid 的零速度安全请求，非有效制导加速度。
  这里不等于保证真机悬停，实际结果仍取决于飞控状态与控制能力。

test_guidance 覆盖三类冻结判据、缺测距 PPN 非零输出、TPN/APN 拒绝、缺姿态、
5 秒未来目标、零时间、时间倒退、50 ms 边界与 stop 不发送；原有正常、回放和闭环测试继续通过。
主程序仍不链接 guidance，未新增任何飞行控制命令。

## P1：NPU 清理与量化

- 参数数组 input_params/output_params 纳入 __destroy_network，释放并置空。
- 所有句柄数组/输出指针数组用 calloc，部分初始化后清理只销毁实际非空项。
- 已 prepare 网络由同一清理函数 finish/destroy，失败路径与正常销毁不再各自重复 finish。
- 配置查询、数组分配及部分资源分配失败能向单一清理路径返回。
- 抽出生产代码实际调用的 buildInputLut：UINT8+NONE 全 256 值恒等；INT8+NONE 返回错误；
  FP32 不使用 LUT；TF_ASYMM 与 DFP 保持标准量化行为，并校验非法 scale/定点位置。
  DFP 使用 ldexp，避免负定点位置导致未定义移位。

test_input_quantization 在 x86 上直接测试该纯函数，没有伪造 vip_lite.h 或声称执行 NPU。
真实 awnn_lib.c/yolo_npu.cpp 的板端编译、链接、每个分配步骤失败注入、重复加载/销毁与泄漏检查仍需板端完成。

## P2：legacy 路径与文档

- target_bridge 拒绝非有限像素、非正框尺寸、非法置信度/类别和过量 count，输出保持无目标。
  test_target_bridge 无需 NPU 或相机，覆盖正常和十种非法/无目标情形。
- 旧 Detector::alive 同时要求推理运行标志和 camera.running；采集异常/退出清除 running，
  再启动前回收旧采集线程。源码路径已修，真实运行仍待板端验证。
- 文档区分历史 14/16 项选择性测试和本轮 27 项全量测试，不把过去日志改成新的验证结果。
- VISO_TYPE 不再写死某个数字；要求按实际固件枚举与官方文档核对，未修改生产参数或标定。
- 全量构建暴露的旧 _GNU_SOURCE 重定义和共享内存样例未检查系统调用告警一并修复。
  共享内存只读样例不再对只读句柄执行无效 ftruncate；这些样例未实际运行。

## 明确遗留：初始化超时不能伪修

VioRuntime::start 和 DetectionRuntime::start 当前仍等待底层初始化完成，未声称具备 30 秒硬期限。
已有估计器构造和供应商 NPU load 没有安全取消接口；wait_for(30s) 后调用 stop/join 仍可能无限等待。
分离线程会悬挂引用外部传感器与对象，强制取消可能破坏 OpenCV、Ceres 或驱动锁，均未采用。

在本轮“不增线程、不改线程模型”的约束内无法给永久阻塞的供应商调用提供可靠有界回收。
需要先有底层协作取消/期限 API，或单独批准进程级隔离/监督设计；本轮仅如实记录该缺口。
正常启动、线程异常、空闲 stop/start 的既有测试仍通过，但它们不能证明驱动永久阻塞可回收。

## CLI 与原工程校验

- main --help、main --check-config、main --no-send --check-config 均返回 0。
- main --config config/app-detection.yaml --check-config 在 NPU OFF 构建返回 2，明确提示缺依赖。
- CLI 均从 /tmp 执行，不打开硬件；测试副本的配置/标定路径解析正确。
- 原工程的全部 1015 个文件/符号链接与复制前基线一致，副本 28 项模型/标定/旧占位配置未变。

可复核记录：

```text
build-fix/verification-fixes.json
build-fix/initial-full-build.log
build-fix/final-clean-build-off.log
build-fix/final-ctest.log
build-fix/source-copy-diff-fixes.txt
```

本轮文件清单与累计副本差异详见 verification-fixes.json。仍未验证真实 NPU、相机、
VIO 精度、ArduPilot 接收/EKF 融合和飞行表现；不能据此宣布整体系统已完成验收。
