# 深度测距输出：TargetSample 与 VIO→飞控 DISTANCE_SENSOR

## 输出契约

使用 `pure_uav::fillTargetDepthRange(target, projector)` 填充现有
`app::TargetSample.has_range` 和 `range_m`。`sample.hpp` 的字段布局不变。
另有一条独立输出：`vio_live --range-pixel` 会把同一测距以 MAVLink
`DISTANCE_SENSOR` 发给飞控（见「和 VIO 同时输出」）。

- `range_m` 是**彩色相机中心到目标所在表面的直线距离**，单位米，不是深度图中的光轴 Z。
- 输入 `(u_px,v_px)` 必须是原始彩色图像坐标。检测侧若缩放、裁剪或 letterbox，先还原坐标。
- 必须先有检测结果/指定查询像素。测距接口不会自行检测目标，也不会修改 `present`、框或置信度。
- 空图、0/65535、错误格式/尺寸、缺标定、时间差超过 50ms、有效邻域不足或明显深度边界，
  都输出 `has_range=false, range_m=0`，并清除旧距离。不会用旧帧距离冒充新观测。
- 默认深度范围 0.15～10m，可通过 `DepthRangeProjector::Options` 调整；这是软件过滤范围，
  不是对相机实际有效量程的保证。
- 距离代表检测中心附近的可见表面。检测框中心落在孔洞或背景上时，需要检测侧调整查询点/ROI；
  本接口不能由一个框判断目标的真实几何中心。

## 实现和边界

相机回调提供**原生深度图**。此前注释写成“已对齐”，但采集线程没有调用 Align。
本实现先按 SDK `getValueScale()` 转为毫米，再使用深度/彩色内参、Brown 畸变以及深度→彩色外参
投影到彩色像素域。同一彩色像素保留最近表面，然后查询 9×9 邻域中位数并检查深度跨度。
9×9 用于容纳 640×400 深度投影到 1080p 后的稀疏点；至少 5 个有效点的门槛保持不变。

深度内参和两相机外参来自实际 SDK 流档位；彩色标定可以显式指定为当前相机/分辨率的 Kalibr cam0 文件。
`vio_live --range-pixel` 自动复用 `--config-dir` 下的 `kalibr_imucam_chain.yaml`；
`depth_range_probe` 用 `--color-calib` 指定。不指定文件时仍用 SDK 彩色工厂标定。
文件必须是 pinhole/radtan、与实际原始彩色尺寸一致，否则拒绝启动，不猜缩放或裁剪。
`T_imu_cam` 不参与 D2C；两相机外参仍从 SDK 获取。
旧设备不支持档位外参时，尝试 pipeline 的工厂参数。
工厂参数必须对应实际两路分辨率。读取失败时拒绝测距，不使用“两相机重合”的兼容假设。
这条测距链与 OpenVINS 深度初始化的兼容处理相互独立。

原始深度转毫米的函数先排除 0/65535，避免缩放把无效码变成普通距离。
目标和深度时间戳须在同一个时钟域；VIO 中按两路设备时间差将深度换到飞控 boot 时钟，
不会在深度时间未知时直接冒用彩色时间。

## 检测/主循环同学如何调用

CMake 链接 `pure_vio_target_depth`，在**视觉处理线程**完成投影和查询，之后再交给已有的目标数据源。
不要在 `ITargetSource::poll()` 的制导周期里做图像计算。

```cpp
#include "pure_uav/target_depth_range.hpp"

pure_uav::DepthRangeProjector projector;
// 相机 start() 后调用一次；分辨率或标定变更时重配。
if (!projector.configure(camera.rangeCalibration())) {
    // 缺少完整标定：测距保持不可用，仍可继续做检测/VIO。
}

// 相机/视觉线程：每个彩色-深度帧集处理一次。
const uint64_t depth_boot_us = pure_uav::depthTimestampFromColor(
    color_boot_us, color_device_us, depth_device_us);
projector.update(depth_mm, depth_boot_us); // 即使本帧深度为空也调用，清掉旧距离。

pure_uav::app::TargetSample target;
// 检测侧填 target.present、u_px、v_px、boot_us 等；boot_us 是这张彩色图的曝光时间。
// target = detection_result;
pure_uav::fillTargetDepthRange(target, projector);
// 将 target 送给现有 ITargetSource；主循环已经会转发 has_range/range_m。
```

多个同帧目标可以复用同一次 `update()`，然后分别查询。
`DepthRangeProjector` 由单个视觉线程持有；跨线程交付完成的 `TargetSample`，不要共享可变投影器。
会话变化、相机停止或重开时调用 `clear()`；相机不再来帧时，数据源还必须保持原有超时机制。
这次没有实现检测器或把 `src/app` 主循环改成真实相机数据源。

## 仅接相机的验收工具（不需要 IMU）

从仓库根目录构建：

```bash
cmake -S . -B build -DPURE_VIO_BUILD_LIVE=ON
cmake --build build --target depth_range_probe test_depth_range -j2
ctest --test-dir build --output-on-failure -R '^depth_range$'
./bin/depth_range_probe --pixel 960 540 --duration 10 --out out/depth_range.csv \
  --color-calib src/vio/estimator/config/gemini_1920x1080/kalibr_imucam_chain.yaml
```

默认彩色 1920×1080，`960 540` 仅是明确指定的中心测距点。换分辨率时同时修改 `--width/--height`
与像素。工具每秒打印 `has_range/range_m` 并逐帧记录 CSV。
CSV 的 `color_device_us/depth_device_us` 是**相机设备时钟**，不是飞控 boot 时间；
不能直接拿去填真机 `TargetSample.boot_us`。没有有效测距时工具返回非零。

## 旧款 Gemini 的 1080p 工厂标定缺失

本机 SV1301S_U3 的 SDK 工厂彩色标定仅有 640×480。1920×1080/1280×720 档位会报告
`Can not find matched camera param`，pipeline 可能返回全零参数；这不是编译失败。
使用上面的 `--color-calib` 复用已有的 1920×1080 彩色标定，深度内参和 D2C 外参仍读取 SDK。
换相机必须换为那台相机的实际标定。不要把 640×480 内参直接按宽高比例拉伸到 16:9。

也可以仅为相机测距验收选择有工厂标定的 640×480：

```bash
./bin/depth_range_probe --width 640 --height 480 --pixel 320 240 \
  --duration 10 --out out/depth_range_640.csv
```

此命令只用于单独验收，VIO 仍使用其配置对应的原始分辨率。
SDK 在停止流时可能输出 USB cancel 警告；首先查看停止之前是否完成了测距与有效帧计数。

## 和 VIO 同时输出（含发飞控）

相机和飞控 IMU 都连接后，可在原有 `vio_live` 命令上增加：

```bash
./bin/vio_live --config-dir src/vio/estimator/config/gemini_1920x1080 \
  --range-pixel 960 540 --range-out out/live_range.csv
```

`--range-pixel` 为固定像素联调。启用后：

- 逐帧写 `out/live_range.csv`：`boot_us,depth_boot_us,u_px,v_px,has_range,range_m`；
- 和 ODOMETRY 走同一条飞控链路，在 `has_range=true` 时发 `MAVLink DISTANCE_SENSOR`
  （`MavlinkLink::sendDistanceSensor`）。`--no-send` 时两样都不发；`has_range=false`
  时不发，不用旧值/0 冒充有效观测；
- `DISTANCE_SENSOR` 字段：`time_boot_ms` = 目标帧曝光时刻（飞控 boot ms）、
  `current_distance` = `range_m*100` cm、`min/max = 15/1000` cm、`id=0`（RNGFND1）、
  `orientation=NONE`（前视；下视改 25）、`covariance=0xFF`（未知）、`type=Laser`。

时间戳在飞控时钟域，无需等 VIO 初始化完成就能测距，但 `vio_live` 本身仍需要飞控 IMU
来建立时钟映射。真实检测接入时使用上面的接口传入实际目标像素，不要继续使用固定的
960/540；固定点只能做联调/日志验证，不是目标几何中心的测距。

### 飞控端配置（ArduPilot）

- 要把对应实例配成 MAVLink 测距仪（`RNGFNDx_TYPE=10`，`id=0` ↔ `RNGFND1`；
  `RNGFNDx_ORIENT=0` 前视）才会接收。没配时飞控会忽略该消息，不影响 ODOMETRY。
- 配好后距离会进 `RANGEFINDER`/测距日志和遥测。Copter 默认只把**向下**安装的测距仪
  转发到 GCS；前视测距如果要在 GCS/制导侧看到，需要另配或从机上读。
- 默认 `EK3_SRCx_POSZ` 不是测距仪，不会直接参与 EKF 高度；如果不想它影响降落/地形，
  先在飞控参数里确认。
- 这是“跟着 VIO 一起发”的观测值，不是 ODOMETRY 的一部分；制导侧的 target range 仍走
  `TargetSample` 那条链路（见上文接口）。
- 同步注意：深度帧比彩色帧旧 27–48ms（实测），`time_boot_ms` 用的是目标帧曝光时刻；
  这个差值是测距的固有延迟，长跑时两路时间戳的偏移还会漂移。`DISTANCE_SENSOR` 的
  `time_boot_ms` 只表达“这条 range 属于哪一帧目标”，不消除该延迟。
- 正式单进程集成（`INTEGRATION_STATUS_20261101.md`）时，这条发送要搬到 `Communicator`：
  加 `pushDistanceSensor()` 并让 `Communicator` 独占串口；当前只在 `vio_live` 的
  `MavlinkLink` 上验证，没有改通信模块的串口归属。

## 当前验证与实机待验

无需硬件的测试覆盖：单位缩放/无效码、离轴距离、两路内外参/分辨率、畸变、遮挡最近面、
邻域空洞与深度边界、时间匹配与溢出、错误输入清旧值，以及真实 TargetSample 字段赋值。
`test_odometry_pack` 另覆盖 `DISTANCE_SENSOR` 的编码/解码、线上字节布局和越界拒绝。

有相机后用尺量静止物体在中心和图像边缘的距离，检查两路标定可读取、投影位置、深度帧时间差
与有效率。先单独跑 `depth_range_probe`，再跑 `vio_live`；只有后一阶段需要飞控 IMU。
2026-11-01 已在本机相机上完成短时验证：640×480 工厂标定路径 5 秒采集 132 帧、132 帧有效；
1920×1080 显式彩色标定路径 5 秒采集 130 帧、130 帧有效。两次只接相机，没有 IMU。
这些结果验证标定读取与距离输出可用，不等于尺量精度、目标中心投影或运动场景验收。
