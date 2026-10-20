#pragma once
// =============================================================================
//  数据源接口（拉模型：主循环主动取"最近一帧"）
//
//  【为什么拆成两个接口】
//    "飞控惯导"和"视觉目标"是两个完全独立的来源：频率不同（约 50 Hz / 30 Hz）、
//    到达时刻不同、掉线互不影响。合成一个 `ISensorSource` 会让每次 poll 都返回
//    "另一半没有新数据"，既难读也容易写错。⇒ 拆成 IInsSource / ITargetSource。
//
//  【为什么用拉模型而不是回调】
//    机载主循环是**定周期**的（建议 250 Hz）。回调式会把数据到达时刻的抖动带进
//    制导周期（dt 不稳 → ω 差分噪声）。拉模型让循环节奏完全由我们自己决定：
//    每拍问一次"有没有新帧"，没有就复用上一帧。
//
//  【实现约束（写实现时必须遵守）】
//    1. `poll_*()` **非阻塞**、**不分配内存**、**不 printf**；拿不到就返回 false。
//    2. 每次返回**最新**一帧（旧帧可以丢，制导只要最新）。
//    3. 时间戳填**采样时刻**（飞控 boot 时钟 µs），不是"读到的时刻"——见 sample.hpp。
//    4. ⚠ 本接口层**不允许 include 通信模块/检测模块的头文件**（`communicator.hpp`、
//       `src/common/msg.hpp`、检测输出头等）；那是具体实现（`*_source.cpp`）的事。
//       这样接口层不会被他们的改名/重构拖着一起改，也避免循环依赖。
// =============================================================================

#include "sample.hpp"

namespace pure_uav {
namespace app {

// ---------------------------------------------------------------- 接口
class IInsSource {
public:
    virtual ~IInsSource() = default;

    // 取一帧飞控惯导。返回 true = 有新帧（已写入 out）；false = 没有新帧（out 不动）。
    virtual bool poll(InsSample &out) = 0;

    // 数据是否仍然新鲜/在线（例如串口是否打开、心跳是否还在）
    virtual bool alive() const = 0;
};

class ITargetSource {
public:
    virtual ~ITargetSource() = default;

    // 取一帧目标观测。返回 true = 有新帧；false = 没看到（可能只是这一拍没新帧，
    // 也可能目标丢失 —— 用 `TargetSample::present` 区分）。
    virtual bool poll(TargetSample &out) = 0;

    virtual bool alive() const = 0;
};

// ---------------------------------------------------------------- 具体实现（只声明）
//
// ⚠ 下面都是**声明**，实现放在对应的 .cpp 里，并且**只有 .cpp 才 include 通信/检测的头文件**。
//    接口层保持"零外部依赖"，方便仿真与真机共用同一份主循环。

// 仿真：从 Gazebo/PX4 真值（或离线真值表）取位姿
class SimTruthInsSource final : public IInsSource {
public:
    SimTruthInsSource() = default;
    ~SimTruthInsSource() override = default;
    bool poll(InsSample &out) override;
    bool alive() const override;
};

// 真机：从通信模块取 LocalPosition + Attitude（两者都是 time_boot_ms）
class MavlinkInsSource final : public IInsSource {
public:
    explicit MavlinkInsSource(/* Communicator& */);   // ⚠ 具体类型在 .cpp 里补，接口层先留声明
    ~MavlinkInsSource() override = default;
    bool poll(InsSample &out) override;
    bool alive() const override;
};

// 仿真：目标真值（含 v_T ⇒ 可用真值法算 ω）
class SimTruthTargetSource final : public ITargetSource {
public:
    SimTruthTargetSource() = default;
    ~SimTruthTargetSource() override = default;
    bool poll(TargetSample &out) override;
    bool alive() const override;
};

// 真机：从检测/视觉取 **曝光时刻 + 检测框中心像素 (u,v)**（+ 可选框尺寸/置信度/距离）；
// "像素 → 相机系 → 机体 → 世界"的换算由 `GuidanceLoop` 用本帧 `q` 完成
// （内参/外参见 `LoopConfig::cam`，标定结果在 VIO 分支的 results/final/ 下）。
// ⚠ 检测模块目前没有代码，这个类等对接口径定了再落实现
class VisionTargetSource final : public ITargetSource {
public:
    VisionTargetSource() = default;
    ~VisionTargetSource() override = default;
    bool poll(TargetSample &out) override;
    bool alive() const override;
};

// 离线：从录制 CSV 回放（回归测试/复盘用，不连任何硬件）
class CsvTargetSource final : public ITargetSource {
public:
    explicit CsvTargetSource(const char *path);
    ~CsvTargetSource() override = default;
    bool poll(TargetSample &out) override;
    bool alive() const override;
};

}  // namespace app
}  // namespace pure_uav
