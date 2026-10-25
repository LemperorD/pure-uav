# IMU 噪声标定（Allan 方差）

给 OpenVINS / Kalibr 提供 `*_noise_density` 和 `*_random_walk`。

---

## 怎么用（PyCharm 点绿三角，不需要参数）

```
第 1 步   飞控平放桌上，USB 供电，不接电机不装桨
          → 点 src/record_static_imu.py        （默认录 8 小时）

第 2 步   点 src/analyze_static_noise.py        （自动找最新的 CSV）
          → 出图 + noise_results.json + kalibr_imu_chain.yaml

第 3 步   把 kalibr_imu_chain.yaml 覆盖到：
            ~/vio_ws/final_config/kalibr_imu_chain.yaml
            ~/calib/ov_config_session05/kalibr_imu_chain.yaml

第 4 步   重跑 session05 的 OpenVINS，对比新旧参数
```

**第一次先试短的**：把 `record_static_imu.py` 顶部 `DURATION_H` 改成 `0.1`（6 分钟），
跑通整条链再改成 8 小时。也可以直接 `python record_static_imu.py 0.1`。

`--probe` 只探 5 秒，确认链路通不通，不正式录。

---

## 为什么录 MAVLink 的 200 Hz 流，而不是 SD 卡里的原始日志

```
OpenVINS 的 /imu0  =  飞控 HIGHRES_IMU @ 200 Hz
                     （已过滤波、降采样、USB 传输）
```

Kalibr / OpenVINS 的 `*_noise_density` 描述的**正是这条流**。
录它，测出来的参数就能直接填进配置；而且不用 SD 卡、不用管日志下载（之前反复失败过）。

---

## 时长要求（决定要不要熬夜）

| 要测的量 | 需要多长 |
|---|---|
| 噪声密度 | **10 分钟**就够 |
| **随机游走** | **4~8 小时**（要 τ 到百秒量级才看得到 +1/2 区） |

只录 10 分钟的话，随机游走会显示"测不出"，那就只能沿用旧值。

---

## 文件

| 文件 | 作用 |
|---|---|
| `src/record_static_imu.py` | 录 8 小时静置数据 → `data/static_<时间戳>.csv` |
| `src/analyze_static_noise.py` | Allan 分析 → 图 + JSON + YAML |
| `src/vendor/` | 逐字节拷贝的已验证模块（`allan.py` / `logparse.py` / `mavlink_min.py`），见其中的 `PROVENANCE.md` |
| `IMU噪声测试方案.md` | 现状盘点 + 证据链 + 补测方案 |
| `噪声参数约定.md` | **`noise_density` 到底是什么量** —— 附三条源码级证据 |
| `src/analyze_noise.py` | ⚠️ **旧的、坏掉的脚本**（调用未定义的 `analyze_group`），已被 `analyze_static_noise.py` 取代，留着做参考 |

---

## 输出长什么样

```
  时间基准    2,160,000 样本，跨度 10800.0 s
  采样间隔    中位 5.0000 ms → 200.00 Hz；抖动 0.00%
  用最长段    10800.0 s / 2,160,000 样本

  分析 gx ……  σ=1.5505e-04  K=1.3416e-06
  ...

  gyroscope_noise_density      1.5507e-04   三轴离散度 1.00×   旧值 1.5700e-03（相差 10.1×）
```

**它会拒绝给数**。以下情况一律报"测不出"，并说明理由：

- 曲线里没有 -1/2 区（白噪声不主导）
- 拟合窗口的值和最短 τ 处直接读数差 10 倍以上（说明窗口落在漂移区）
- 大 τ 端没超出白噪声预言（看不到随机游走）
- 时间基准抖动 > 10%

这是故意的：**宁可不出数，也不要出一个看起来合理但是错的数。**

---

## 分析前先过自检

`analyze_static_noise.py` 每次都会先跑合成自检（约 40 秒）：
用**已知的** σ 和 K 造数据，看能不能反解回来。反解不回来就拒绝分析真实数据。

```
  [PASS] 库函数 vs 独立实现，最大相对差 3.89%  (容差 5%)
  [PASS] 纯白噪声 σ：got=9.9816e-05  want=1.000e-04
  [PASS] 纯白噪声 K 应判为【测不出】
  [PASS] σ：got=1.0004e-04  want=1.000e-04
  [PASS] K：got=1.0104e-06  want=1.000e-06
  [PASS] 纯随机游走 K：got=1.0053e-05  want=1.000e-05
```

端到端实测（3 小时合成数据，反解误差）：σ 误差 **0.03%**，三轴离散度 **1.00×**。

---

## ⚠️ 两个必须知道的坑

### 1. 别用旧值，它大了约 10~45 倍

工作区里现用的 `1.57e-3` 是用错误约定算的（详见 `噪声参数约定.md`）。
正确约定：**`noise_density = AD(1 s)`**，不是 `AD(1)·√fs`。

### 2. 别直接把 imu_utils 的 `gyr_n` 填进 OpenVINS

`imu_utils` 的公式是 `√fs·AD(1)`，等于**离散样本 std**，
而 Kalibr/OpenVINS 要的是连续密度 —— 两者差 √fs 倍。

---

## 环境

用 `vins` 这个 conda 环境的 Python：

```
/home/kxy/miniforge3/envs/vins/bin/python
```

PyCharm 里把解释器指到它即可。依赖：`numpy`；绘图另需 `matplotlib`（已装）。
读飞控 DataFlash 日志（`.bin`，交叉验证用）不需要额外依赖。
