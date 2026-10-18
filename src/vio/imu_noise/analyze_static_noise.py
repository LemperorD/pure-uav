#!/usr/bin/env python3
"""IMU 噪声分析（Allan 方差）—— 静置数据 → OpenVINS / Kalibr 噪声参数。

怎么用（PyCharm 里直接点绿三角）:
    python analyze_static_noise.py                  # 自动取 imu/data/ 里最新的 CSV
    python analyze_static_noise.py 某个文件.csv      # 指定 CSV
    python analyze_static_noise.py log_8.bin        # 也能读飞控 DataFlash 日志（交叉验证用）
    python analyze_static_noise.py --skip-selfcheck # 跳过合成自检（不建议）

输出（到 imu/results/<时间戳>/）:
    allan_gyro.png / allan_accel.png     Allan 偏差曲线 + 拟合线
    noise_results.json                   全部数字（含拟合窗口、R²、判据）
    kalibr_imu_chain.yaml                可直接给 OpenVINS 的配置

=========================================================================
 约定 —— 这一段是整份工具的关键，之前的实现就是在这里错的
=========================================================================

本工具用的公式（与 Kalibr / OpenVINS 一致）：

    离散噪声 std = noise_density / sqrt(dt) = noise_density * sqrt(fs)
    ⟹ noise_density = σ_d / sqrt(fs)

    Allan 偏差（重叠式，簇长 m = τ·fs）:
        AD(τ) = σ_d / sqrt(τ·fs)          σ_d = 单样本 std
    代入 σ_d = noise_density·sqrt(fs):
        AD(τ) = noise_density / sqrt(τ)   ← 【与 fs 无关】，物理量必须如此

    所以：
        白噪声区:   noise_density = AD(τ) · sqrt(τ)
        随机游走区: random_walk  = AD(τ) · sqrt(3/τ)

⚠️ 工作区里旧的 `allan.py::estimate_axis()` 用的是 `σ = AD(1)·√fs`，
   那算出来的是【离散样本 std】而不是噪声密度，**偏大 √fs 倍**。
   它自己的 `selfcheck.py` 就因此跑不过（4 个用例一律偏差 3062~3065%）。
   本文件不用它，自己实现，并在跑真实数据前用合成数据反解验证。
=========================================================================
"""
from __future__ import annotations

import csv
import json
import math
import os
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
VENDOR = HERE / "vendor"
sys.path.insert(0, str(VENDOR))
from allan import allan_deviation, tau_grid  # noqa: E402

# ==========================================================================
#  参数
# ==========================================================================
INPUT = None             # None = 自动取 imu/data/static_*.csv 里最新的
PTS_PER_DECADE = 10      # τ 网格密度
MAX_RATIO = 0.25         # 最大簇长占样本数的比例
SLOPE_TOL = 0.12         # 局部斜率与 ±0.5 的容差
MIN_RUN_POINTS = 5       # 一段拟合窗口至少几个 τ 点
MIN_RUN_DECADES = 0.4    # 一段拟合窗口至少跨几个十倍程
GAP_FACTOR = 5.0         # 间隔 > GAP_FACTOR×中位 就算缝隙
#  拟合窗口的 σ 最多能是最短 τ 处直接读数的几倍。
#  为什么会有倍数：飞控若做了低通，τ_min 处的 AD 反映的是【滤波后】的 std，
#  比真值小 √(fs/(2B))（B=噪声带宽）。最极端的情况（B≈5 Hz @ fs≈2 kHz）约 14 倍。
#  超过这个量级，说明曲线在短 τ 端是【上升】的 —— 那是漂移的形状，不是白噪声。
#  实测 log_8：比值 53~185。
#  实测 EuRoC MH_01（飞行中、有电机振动）：比值 19~102。
#  ⚠️ 但实测【8 小时真数据】的加计是 13 —— 刚好被 10 卡掉，而那是好数据！
#     原因：加计低通 10 Hz、陀螺低通 20 Hz，加计压得更狠 → 比值天然更大。
#     用固定阈值区分"被低通"和"在漂移"，本来就是错的。
#     所以真正的判据改成 TREND_MAX（见下），RATIO_MAX 只做粗筛。
RATIO_MAX = 25.0

# 白噪声区的 σ(τ)=AD(τ)·√τ 必须是【平台】，不能系统性上升。
# 漂移区的 σ(τ) 是单调爬升的（低频功率随 τ 增长），这是它和白噪声区的本质区别。
# 实测：8h 加计 ay/az 的真平台在 1.2 个十倍程内只变 5%（斜率 0.04）；
#       同一份数据的 ax（漂移污染）斜率 0.26；log_8 的漂移区 0.27。
# 0.15 把两者干净地分开。
TREND_MAX = 0.15

# 物理合理性上界：实测 σ 不能超过手册值这么多倍。
# 为什么必须有：**宽带机械振动和传感器白噪声在 Allan 图上是同一种形状**，
# 任何基于曲线形状的判据都分不开它们。实测 EuRoC MH_01（飞行中、电机振动）
# 会报出手册的 845 倍（陀螺）/ 59 倍（加计）—— 一颗 MEMS 不可能这么差，
# 那读到的是振动，不是噪声。有了这道闸就变成"测不出"，而不是一个荒谬的数。
# ⚠️ 这是**本设备**（IIM-42652）的合理性上界，不是通用判据；换 IMU 要改 DATASHEET。
PLAUSIBLE_MAX_RATIO = 5.0
PLOT = True
SELFCHECK = True

# 目前工作区里在用的值，用来做对比（不参与计算）
CURRENT = dict(
    gyroscope_noise_density=1.5700e-03,
    accelerometer_noise_density=2.5000e-03,
    gyroscope_random_walk=1.0000e-05,
    accelerometer_random_walk=3.0000e-04,
    source="~/vio_ws/final_config/kalibr_imu_chain.yaml",
)
# 数据手册典型值，只用于"是否离谱"的对照，不参与计算。
# 出处：IIM-42652 datasheet DS-000440 Rev 1.1
#   Table 1 Gyroscope Specifications:
#       Rate Noise Spectral Density @ 10 Hz = 0.0038 º/s/√Hz   (TYP, note 1 = tested in production)
#       → 0.0038 × π/180 = 6.632e-05 rad/s/√Hz
#   Table 2 Accelerometer Specifications:
#       Power Spectral Density @ 10 Hz = 70 µg/√Hz            (TYP, note 1)
#       → 70e-6 × 9.80665 = 6.865e-04 m/s²/√Hz
#   手册自洽性检查：Total RMS Noise = 0.038 º/s-rms @ BW=100Hz
#       → 0.0038×√100 = 0.038 ✓ ；RMS Noise = 0.70 mg-rms → 70e-6×√100 = 0.70 mg ✓
DATASHEET = dict(
    gyroscope_noise_density=6.632e-05,
    accelerometer_noise_density=6.865e-04,
)


# ==========================================================================
#  1. 合成自检 —— 必须先通过，否则不许碰真实数据
# ==========================================================================

def synth(sigma: float, K: float, n: int, fs: float, seed: int) -> np.ndarray:
    """白噪声（密度 sigma）+ 零偏随机游走（强度 K）。"""
    r = np.random.default_rng(seed)
    dt = 1.0 / fs
    white = r.normal(0.0, sigma / math.sqrt(dt), n)   # 离散 std = σ/√dt
    steps = r.normal(0.0, K * math.sqrt(dt), n)       # Wiener 过程，强度 K
    return white + np.cumsum(steps)


def _brute_allan(x: np.ndarray, fs: float, tau: float) -> float:
    """独立实现（非重叠分块），用来交叉验证库函数。"""
    m = int(round(tau * fs))
    n = (len(x) // m) * m
    blocks = x[:n].reshape(-1, m).mean(axis=1)
    d = np.diff(blocks)
    return float(math.sqrt(0.5 * np.mean(d * d)))


def run_selfcheck(log=print) -> bool:
    log("=" * 74)
    log("  合成自检 —— 用已知 σ/K 造数据，看能不能反解回来")
    log("=" * 74)
    ok = True
    fs = 200.0
    dt = 1.0 / fs

    # --- 检查 1：allan_deviation 的实现本身对不对（对暴力实现） ---
    r = np.random.default_rng(7)
    x = r.normal(0.0, 1.0, 200_000)
    taus, adev = allan_deviation(x, dt, pts_per_decade=PTS_PER_DECADE)
    worst = 0.0
    for tau_target in (0.05, 0.5, 2.0):
        j = int(np.argmin(np.abs(taus - tau_target)))
        b = _brute_allan(x, fs, float(taus[j]))
        rel = abs(adev[j] - b) / b
        worst = max(worst, rel)
    log(f"  [{'PASS' if worst < 0.05 else 'FAIL'}] 库函数 vs 独立实现，最大相对差 {worst:.2%}  (容差 5%)")
    ok &= worst < 0.05

    # --- 检查 2：纯白噪声 → σ 应被反解，K 应被【正确拒绝】 ---
    sigma = 1.0e-4
    x = synth(sigma, 0.0, int(0.5 * 3600 * fs), fs, seed=11)
    res = extract(x, fs, "自检-纯白噪声")
    good = res["sigma"] is not None and abs(res["sigma"] - sigma) / sigma < 0.10
    log(f"  [{'PASS' if good else 'FAIL'}] 纯白噪声 σ：got={_fmt(res['sigma'])}  want={sigma:.3e}  (容差 10%)")
    ok &= good
    rw_ok = res["K"] is None
    log(f"  [{'PASS' if rw_ok else 'FAIL'}] 纯白噪声 K 应判为【测不出】：got={_fmt(res['K'])}")
    ok &= rw_ok

    # --- 检查 3：白噪声 + 随机游走 → 两个都要反解回来 ---
    sigma, K = 1.0e-4, 1.0e-6
    x = synth(sigma, K, int(4 * 3600 * fs), fs, seed=12)
    res = extract(x, fs, "自检-白噪声+随机游走")
    g1 = res["sigma"] is not None and abs(res["sigma"] - sigma) / sigma < 0.15
    g2 = res["K"] is not None and abs(res["K"] - K) / K < 0.50
    log(f"  [{'PASS' if g1 else 'FAIL'}] σ：got={_fmt(res['sigma'])}  want={sigma:.3e}  (容差 15%)")
    log(f"  [{'PASS' if g2 else 'FAIL'}] K：got={_fmt(res['K'])}  want={K:.3e}  (容差 50%)")
    ok &= g1 and g2

    # --- 检查 4：纯随机游走 → K 反解，σ 应判为测不出 ---
    K = 1.0e-5
    x = synth(1e-12, K, int(2 * 3600 * fs), fs, seed=13)
    res = extract(x, fs, "自检-纯随机游走")
    g = res["K"] is not None and abs(res["K"] - K) / K < 0.50
    log(f"  [{'PASS' if g else 'FAIL'}] 纯随机游走 K：got={_fmt(res['K'])}  want={K:.3e}  (容差 50%)")
    ok &= g

    log("")
    log(f"  自检结果：{'✅ 全部通过' if ok else '❌ 有失败项 —— 拒绝分析真实数据'}")
    log("")
    return ok


def _fmt(v):
    return "测不出" if v is None else f"{v:.4e}"


# ==========================================================================
#  2. 提取 σ 和 K（核心）
# ==========================================================================

def _local_slopes(taus: np.ndarray, adev: np.ndarray, half: int = 2) -> np.ndarray:
    """逐点局部斜率 d(log AD)/d(log τ)。"""
    lt, la = np.log(taus), np.log(adev)
    n = len(lt)
    slopes = np.full(n, np.nan)
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        if hi - lo < 3:
            continue
        a = np.vstack([np.ones(hi - lo), lt[lo:hi]]).T
        c, s = np.linalg.lstsq(a, la[lo:hi], rcond=None)[0]
        slopes[i] = s
    return slopes


def _rw_start_index(taus, adev, sigma, ratio=1.3, need=MIN_RUN_POINTS):
    """随机游走只有在大 τ 端【真的把曲线抬起来】时才可测。

    判据：AD(τ) > ratio × σ/√τ（白噪声预言）。
    ratio=1.3 意味着随机游走在总功率里占到 1-1/1.3² ≈ 41%。
    否则 k² 会被拟合成 0 —— 那是在拿白噪声区冒充随机游走区。
    返回第一段满足条件的连续区段的起始下标；没有则返回 None。
    """
    if sigma is None:
        return 0
    mask = adev > ratio * sigma / np.sqrt(taus)
    run = 0
    for i, m in enumerate(mask):
        run = run + 1 if m else 0
        if run >= need:
            return i - need + 1
    return None


def _grow_window(taus, adev, kind, min_pts=MIN_RUN_POINTS, min_dec=MIN_RUN_DECADES,
                 tol=0.15, sigma_fixed=None, i_min=0, i_max=None,
                 prefer_low=False, spread_max=None, trend_max=None):
    """在 τ 轴上找一段【能用给定模型描述】的窗口。

    为什么不按"局部斜率是否等于 ±1/2"来切段：实测曲线在交叉点附近是弯的，
    严格连续且斜率命中的区段会被切成 2~3 个碎片（自检里就踩到了）。

    模型（物理上正确的两项之和）:
        AD²(τ) = σ²/τ + K²·τ/3
      kind="white": 固定 K=0，只解 σ   → AD_pred = σ/√τ
      kind="rw"   : 固定 σ=sigma_fixed，只解 K → AD_pred = √(σ²/τ + K²τ/3)

    判据用【相对偏差的 90 分位】而不是最大值，容忍个别离群点。

    prefer_low=True 时优先取【起始 τ 最小】的窗口 —— 这一条很关键：
        白噪声在物理上一定主导最短的积分时间；而漂移区（斜率也能碰巧≈-1/2）
        在长 τ 端。只在长 τ 端找，会把漂移当成白噪声，报出荒谬的大值。
        （2026-10-16 实测 log_8：不加这条会在 τ=2~213 s 的漂移区报出 1.5e-2。）
    """
    n = len(taus)
    best = None
    for i in range(i_min, min(n, i_max) if i_max is not None else n):
        for j in range(i + min_pts, n + 1):
            t, a = taus[i:j], adev[i:j]
            if math.log10(t[-1] / t[0]) < min_dec:
                continue
            if kind == "white":
                vals = a * np.sqrt(t)                     # σ(τ) = AD(τ)·√τ
                if trend_max is not None and len(t) >= 3:
                    # σ(τ) 对 τ 的双对数斜率：平台≈0，漂移区明显为正
                    llt, llv = np.log(t), np.log(vals)
                    trend = float(np.linalg.lstsq(
                        np.vstack([np.ones(len(llt)), llt]).T, llv, rcond=None)[0][1])
                    if trend > trend_max:
                        break                             # 系统性上升 → 这不是白噪声区
                spread = float(vals.max() / vals.min())
                if spread_max and spread > spread_max:
                    break
                scale = float(np.median(vals))
                pred = scale / np.sqrt(t)
                param = scale
            else:
                s2 = (sigma_fixed ** 2) if sigma_fixed else 0.0
                k2 = np.mean(3.0 * np.maximum(a ** 2 - s2 / t, 0.0) / t)
                if k2 <= 0:
                    continue
                param = math.sqrt(k2)
                pred = np.sqrt(s2 / t + k2 * t / 3.0)
            dev = float(np.percentile(np.abs(a - pred) / pred, 90))
            if dev > tol:
                break                                  # 再长就不成立了
            dec = math.log10(t[-1] / t[0])
            key = ((-i,) if prefer_low else ()) + (round(dec, 3), j - i)
            if best is None or key > best[0]:
                best = (key, i, j, param, dev)
    return best


def _peak_trough(adev: np.ndarray, first_frac: float = 0.5) -> tuple[int, int]:
    """找 AD 曲线的【第一个峰】和它后面的【最低点】。

    飞控做了低通时，Allan 曲线的标准形状是：

        上升（滤波滚降） → 峰(τ≈τ_c) → 下降(-1/2 白噪声) → 谷 → 上升(+1/2 漂移/随机游走)
                                          └── 白噪声区 ──┘   └─ 随机游走区 ─┘

    所以：
      * 白噪声区只能落在【峰 到 谷】之间 —— 这就是 _white_start_limit 的物理依据。
        不加这条会怎样（实测）：log_8 的漂移区在 τ=2~213 s 斜率碰巧是 -0.50、
        拟合偏差 0.13、R²=0.99，看着完美，报出来却是 5e-3，比真值大两个量级。
      * 随机游走区必须在【谷之后】，否则会把滤波滚降段（斜率也是 +0.5 附近）
        当成随机游走。实测 6 分钟那份数据就踩到了：gx/gz 在 τ=0.005~0.04 s
        报出 4.6e-3 / 7.6e-3 的"随机游走"，比 MEMS 典型值大 500 倍 ——
        那一段其实是低通滤波器的滚降，不是随机游走。
    """
    half = max(4, int(len(adev) * first_frac))
    i_peak = int(np.argmax(adev[:half]))
    i_trough = i_peak + int(np.argmin(adev[i_peak:]))
    return i_peak, i_trough


def extract(x: np.ndarray, fs: float, tag: str = "", plausible_max: float | None = None) -> dict:
    """从一条时间序列里提取白噪声密度 σ 和随机游走强度 K。

    返回 dict：sigma / K / 各自的拟合窗口、点数、偏差、理由。
    测不出来就返回 None，并给出理由 —— 【绝不硬凑一个数】。
    """
    x = np.asarray(x, float)
    x = x - x.mean()
    dt = 1.0 / fs
    taus, adev = allan_deviation(x, dt, pts_per_decade=PTS_PER_DECADE, max_ratio=MAX_RATIO)
    good = np.isfinite(adev) & (adev > 0)
    taus, adev = taus[good], adev[good]
    out = dict(tag=tag, n=len(x), fs=fs, duration_s=len(x) * dt,
               tau=taus.tolist(), adev=adev.tolist(),
               sigma=None, K=None, sigma_info={}, K_info={})
    if len(taus) < 8:
        out["sigma_info"] = dict(reason="可用 τ 点太少")
        out["K_info"] = dict(reason="可用 τ 点太少")
        return out

    slopes = _local_slopes(taus, adev)

    # ---- 1) 白噪声密度 σ ----
    #   prefer_low=True：锚在最短 τ 端（白噪声物理上主导最短积分时间）
    #   spread_max：AD·√τ 在窗口内必须是"平台"，不能是缓慢下滑
    i_peak, i_trough = _peak_trough(adev)
    # 容差 0.25：实测 6 分钟那份数据里，真平台的 σ(τ) 本身就有 ±15% 的统计散布，
    # 用 0.15 会把真平台切碎（gx 因此被漏掉）。防漂移主要由下面的 RATIO_MAX 把关。
    w = _grow_window(taus, adev, "white", tol=0.25, i_max=i_trough + 2,
                     prefer_low=True, spread_max=1.60, trend_max=TREND_MAX)
    # 免模型交叉验证的锚点：τ = 一个采样间隔时，漂移的贡献≈0，
    # 所以 AD(τ_min) 直接就是【单样本 std】，σ = AD(τ_min)/√fs。
    # 若飞控做了低通，这个是【下界】。
    sigma_short = float(adev[0] / math.sqrt(fs))

    sigma = None
    if w:
        _, i, j, sig_win, dev = w
        seg_t = taus[i:j]
        lt, la = np.log(seg_t), np.log(adev[i:j])
        c = float(np.mean(la + 0.5 * lt))
        resid = la - (c - 0.5 * lt)
        ss_tot = float(np.sum((la - la.mean()) ** 2))
        r2 = 1.0 - float(np.sum(resid ** 2)) / ss_tot if ss_tot > 0 else None
        ratio = float(sig_win / sigma_short) if sigma_short > 0 else None
        base = dict(
            tau_lo=float(seg_t[0]), tau_hi=float(seg_t[-1]), n_points=j - i,
            decades=math.log10(seg_t[-1] / seg_t[0]), r2=r2, max_rel_dev=dev,
            slope_median=float(np.nanmedian(slopes[i:j])),
            spread=float((adev[i:j] * np.sqrt(seg_t)).max() /
                         (adev[i:j] * np.sqrt(seg_t)).min()),
            sigma_short_tau_min=sigma_short, short_over_window=ratio)
        if ratio is not None and ratio > RATIO_MAX:
            # 曲线在短 τ 端是【上升】的（不是平台、也不是 -1/2 下降），
            # 说明白噪声在最短积分时间处并不主导 —— 那是漂移/量化的形状。
            # 此时任何"看着像 -1/2"的长 τ 区段都是漂移区，报出来会大两个量级。
            out["sigma_info"] = dict(
                reason=(f"曲线在短 τ 端是上升的：最短 τ 处 σ={sigma_short:.3e}，"
                        f"而拟合窗口给出 {sig_win:.3e}，相差 {ratio:.0f}×"
                        f"（合理上限 {RATIO_MAX:.0f}×）。白噪声不主导，不给数。"),
                rejected_value=float(sig_win), **base)
        else:
            out["sigma"] = float(sig_win)
            out["sigma_info"] = dict(
                **base, method="AD²=σ²/τ 约束拟合，σ 取 AD(τ)·√τ 中位")
    else:
        out["sigma_info"] = dict(
            reason="没找到能用 AD=σ/√τ 描述的区段（白噪声主导区）",
            sigma_short_tau_min=sigma_short,
            slope_range=[float(np.nanmin(slopes)), float(np.nanmax(slopes))])

    # ---- 1b) 物理合理性上界 ----
    #  必须放在这里：下面的随机游走段有两处提前 return，放最后会被绕过（实测踩到）。
    if plausible_max and out["sigma"] and out["sigma"] > plausible_max:
        out["sigma_info"] = dict(
            reason=(f"测出 {out['sigma']:.3e}，超过手册 {PLAUSIBLE_MAX_RATIO:g} 倍上限"
                    f"（{plausible_max:.3e}）—— 这是宽带振动/运动，不是传感器噪声。"
                    "Allan 图分不开这两者，只能靠量级否决。"),
            rejected_value=float(out["sigma"]),
            plausible_max=float(plausible_max), **(out["sigma_info"] or {}))
        out["sigma"] = None
        sigma = None                        # 随机游走那段的 σ 固定也要跟着作废

    # ---- 2) 随机游走 K ----
    #   用两项模型 AD² = σ²/τ + K²τ/3，σ 用上面测到的值固定住，
    #   这样交叉点附近"白噪声还在贡献"就不会污染 K。
    #   并且只在【曲线真的超出白噪声预言】的 τ 段里找。
    rw_gate = _rw_start_index(taus, adev, sigma)
    if rw_gate is None:
        out["K_info"] = dict(
            reason="大 τ 端曲线没有超出白噪声预言（AD ≤ 1.3×σ/√τ）——"
                   "这份数据里看不到随机游走，不能报一个数（要 4~8 小时才看得到）")
        return out
    # 双闸门：① 必须在曲线【谷之后】（否则会把低通滚降段当随机游走）
    #         ② 若 σ 已知，还必须真的超出白噪声预言
    i_min = max(rw_gate, i_trough)
    if i_min >= len(taus) - MIN_RUN_POINTS:
        out["K_info"] = dict(
            reason="曲线谷之后剩下的 τ 点太少，随机游走测不出来（要 4~8 小时）")
        return out
    k = _grow_window(taus, adev, "rw", tol=0.20, sigma_fixed=sigma, i_min=i_min)
    if k:
        _, i, j, K, dev = k
        seg_t = taus[i:j]
        out["K"] = float(K)
        out["K_info"] = dict(
            tau_lo=float(seg_t[0]), tau_hi=float(seg_t[-1]), n_points=j - i,
            decades=math.log10(seg_t[-1] / seg_t[0]), max_rel_dev=dev,
            slope_median=float(np.nanmedian(slopes[i:j])),
            sigma_used=sigma,
            method="AD²=σ²/τ+K²τ/3 约束拟合（σ 固定为上面测到的值）")
    else:
        out["K_info"] = dict(
            reason="找到的超白噪声区太短（<0.4 个十倍程）—— 录制时长不够，"
                   "随机游走测不出来（要 4~8 小时）")
    return out


# ==========================================================================
#  3. 读数据
# ==========================================================================

def load_csv(path: Path, log=print):
    """流式读 CSV —— 8 小时有 576 万行。

    不能写成 `rows = [row for row in r]` 再一次性转 numpy：
    576 万个 Python 小列表 ≈ 2 GB，加上最终的 ndarray 会顶到内存上限（实测踩到）。
    按 20 万行一块转，峰值内存 = 最终数组 + 一块的临时量（约 40 MB）。
    """
    with open(path, newline="") as fh:
        r = csv.reader(fh)
        hdr = next(r)
        chunks, buf, n = [], [], 0
        for row in r:
            if not row:
                continue
            buf.append(row)
            n += 1
            if len(buf) >= 200_000:
                chunks.append(np.array(buf, dtype=float))
                buf = []
                log(f"\r    读入 {n:,} 行 ……", end="", flush=True)
        if buf:
            chunks.append(np.array(buf, dtype=float))
    log(f"\r    读入 {n:,} 行 × {len(hdr)} 列                    ")
    a = np.concatenate(chunks) if chunks else np.empty((0, len(hdr)))
    if "fc_time_us" in hdr:
        t_fc = a[:, hdr.index("fc_time_us")]
        t_host = a[:, hdr.index("host_ns")] / 1e9
    else:                                              # 兼容旧的 imu.csv
        t_fc = np.zeros(len(a))
        t_host = a[:, hdr.index("timestamp")] / 1e9
    def col(name):
        return a[:, hdr.index(name)]
    data = dict(ax=col("ax"), ay=col("ay"), az=col("az"),
                gx=col("gx"), gy=col("gy"), gz=col("gz"))
    # 时间基准：优先飞控时钟（均匀、无 USB 抖动）
    if np.count_nonzero(np.diff(t_fc) > 0) > 0.9 * (len(t_fc) - 1):
        t, base = t_fc / 1e6, "fc_time_us（飞控时钟）"
    else:
        t, base = t_host, "host_ns（宿主时钟）"
    return t - t[0], data, dict(source=str(path), time_base=base, kind="csv", header=hdr)


def load_bin(path: Path, instance: int = 0):
    """读 ArduPilot DataFlash 日志里的 GYR/ACC（交叉验证用，不是主路径）。"""
    from logparse import read_log
    gyr, acc = [], []
    for name, f in read_log(str(path)):
        if name == "GYR" and f.get("I") == instance:
            gyr.append((f["SampleUS"] / 1e6, f["GyrX"], f["GyrY"], f["GyrZ"]))
        elif name == "ACC" and f.get("I") == instance:
            acc.append((f["SampleUS"] / 1e6, f["AccX"], f["AccY"], f["AccZ"]))
    if not gyr and not acc:
        raise SystemExit(f"{path} 里没有 IMU 实例 {instance} 的 GYR/ACC 数据")
    src = gyr if gyr else acc
    t = np.array([r[0] for r in src], float)
    data = {}
    if gyr:
        g = np.array(gyr, float)
        data.update(gx=g[:, 1], gy=g[:, 2], gz=g[:, 3])
    if acc:
        a = np.array(acc, float)
        data.update(ax=a[:, 1], ay=a[:, 2], az=a[:, 3])
    return t - t[0], data, dict(source=str(path), time_base="SampleUS（采样计数）",
                                kind="bin", n_gyr=len(gyr), n_acc=len(acc))


def timebase_report(t, log=print):
    """检查时间基准，切出最长无缝隙段。"""
    d = np.diff(t)
    d = d[d > 0]
    if len(d) == 0:
        raise SystemExit("时间戳没有严格递增的部分，无法分析")
    med = float(np.median(d))
    fs = 1.0 / med
    jitter = float(np.std(d) / med)
    log(f"  时间基准    {len(t):,} 样本，跨度 {t[-1]:.1f} s")
    log(f"  采样间隔    中位 {med * 1e3:.4f} ms → {fs:.2f} Hz；抖动 {jitter:.2%}")
    dt_all = np.diff(t)
    gaps = np.where(dt_all > GAP_FACTOR * med)[0]
    segs, bounds = [], [0] + [int(g) + 1 for g in gaps] + [len(t)]
    for i in range(len(bounds) - 1):
        a, b = bounds[i], bounds[i + 1]
        if b - a > 1:
            segs.append((a, b, t[b - 1] - t[a]))
    segs.sort(key=lambda s: -s[2])
    if gaps.size:
        log(f"  缝隙        {gaps.size} 处（>{GAP_FACTOR:g}× 中位间隔），按缝隙切成 {len(segs)} 段")
    log(f"  用最长段    {segs[0][2]:.1f} s / {segs[0][1] - segs[0][0]:,} 样本"
        + (f"（占总时长 {100 * segs[0][2] / (t[-1] - t[0]):.1f}%）" if gaps.size else ""))
    return fs, med, jitter, segs


# ==========================================================================
#  4. 输出
# ==========================================================================

def plot_allan(gyro_res, accel_res, outdir: Path, log=print):
    if not PLOT:
        return []
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpl")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
    except ImportError:
        log("  ⚠️ matplotlib 不可用，跳过绘图")
        return []
    # 中文字体：不设的话标题/轴标签会画成方块（实测过）
    for cand in ("Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei",
                 "Source Han Sans SC", "SimHei"):
        try:
            if any(f.name == cand for f in font_manager.fontManager.ttflist):
                matplotlib.rcParams["font.sans-serif"] = [cand]
                break
        except Exception:                                              # noqa: BLE001
            pass
    matplotlib.rcParams["axes.unicode_minus"] = False
    # 这个 matplotlib 版本在 savefig 时会刷一堆 MatplotlibDeprecationWarning
    # （facecolor/edgecolor/orientation 之类，不是我们传的）。图本身没问题，压掉。
    import warnings
    warnings.filterwarnings("ignore", category=matplotlib.MatplotlibDeprecationWarning)
    made = []
    for label, res, keys in (("gyro", gyro_res, ("gx", "gy", "gz")),
                             ("accel", accel_res, ("ax", "ay", "az"))):
        fig, ax = plt.subplots(figsize=(10, 6.5))
        colors = ("#d62728", "#2ca02c", "#1f77b4")
        for k, c in zip(keys, colors):
            r = res.get(k)
            if not r or not r.get("tau"):
                continue
            taus, adev = np.array(r["tau"]), np.array(r["adev"])
            lbl = f"{k}"
            if r["sigma"]:
                lbl += f"  σ={r['sigma']:.3e}"
            ax.loglog(taus, adev, color=c, lw=1.6, label=lbl)
            # 拟合参考线
            if r["sigma"]:
                i = r["sigma_info"]
                tt = np.array([i["tau_lo"], i["tau_hi"]])
                ax.loglog(tt, r["sigma"] / np.sqrt(tt), "--", color=c, alpha=0.5, lw=1.0)
            if r["K"]:
                i = r["K_info"]
                tt = np.array([i["tau_lo"], i["tau_hi"]])
                ax.loglog(tt, r["K"] * np.sqrt(tt / 3.0), ":", color=c, alpha=0.6, lw=1.2)
        ax.set_xlabel("簇时间 τ (s)")
        ax.set_ylabel("Allan 偏差")
        ax.set_title(f"Allan 偏差 — {label}   (σ: 白噪声密度, 虚线=σ/√τ;  K: 随机游走, 点线=K√(τ/3))")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend(fontsize=8, loc="lower left")
        p = outdir / f"allan_{label}.png"
        fig.tight_layout()
        fig.savefig(p, dpi=140)
        plt.close(fig)
        made.append(str(p))
        log(f"  ✅ {p}")
    return made


def write_yaml(path: Path, values: dict, meta: dict, spreads: dict | None = None, log=print):
    g = values
    spreads = spreads or {}

    def val(k, old):
        v = g.get(k)
        if v is None:
            return f"{old:.4e}   # ⚠️ 本次没测出，沿用旧值（见 noise_results.json 的理由）"
        sp = spreads.get(k)
        if sp and sp > 3.0:
            # 三轴差这么多，说明这个量本身没测稳（多半是被漂移污染了）。
            # 仍写出来（比旧值靠谱），但必须标清楚，不能标成"实测"。
            return (f"{v:.4e}   # ⚠️ 三轴离散度 {sp:.1f}×，可信度低 —— "
                    "拿它当量级参考，别当精测值")
        if sp:
            return f"{v:.4e}   # ✅ 本次实测（三轴离散度 {sp:.2f}×）"
        return f"{v:.4e}   # ✅ 本次实测"
    txt = f"""%YAML:1.0

# =====================================================================
# IMU 噪声参数 —— 实测
# 生成: {datetime.now():%Y-%m-%d %H:%M}    工具: analyze_static_noise.py
#
# 数据源: {meta.get('source')}
# 时长:   {meta.get('duration_s', 0):.0f} s ({meta.get('duration_s', 0) / 3600:.2f} h)
# 速率:   {meta.get('fs', 0):.2f} Hz
# 时间基准: {meta.get('time_base')}
#
# 方法: 重叠式 Allan 方差；白噪声密度在 -1/2 斜率区取 AD(τ)·√τ 的中位；
#       随机游走强度在 +1/2 斜率区取 AD(τ)·√(3/τ) 的中位。
#       约定与 Kalibr/OpenVINS 一致: 离散噪声 std = noise_density·√fs
#       本工具在分析前先过合成数据反解自检。
# =====================================================================

imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]

  gyroscope_noise_density:     {val('gyroscope_noise_density', CURRENT['gyroscope_noise_density'])}
  accelerometer_noise_density: {val('accelerometer_noise_density', CURRENT['accelerometer_noise_density'])}
  gyroscope_random_walk:       {val('gyroscope_random_walk', CURRENT['gyroscope_random_walk'])}
  accelerometer_random_walk:   {val('accelerometer_random_walk', CURRENT['accelerometer_random_walk'])}

  rostopic: /imu0
  time_offset: 0.0
  update_rate: {meta.get('fs', 200.0):.1f}
  model: "kalibr"
  Tw:          [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
  R_IMUtoGYRO: [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
  Ta:          [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
  R_IMUtoACC:  [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
  Tg:          [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]
"""
    path.write_text(txt)
    log(f"  ✅ {path}")


# ==========================================================================
#  5. 主流程
# ==========================================================================

def find_latest_csv() -> Path | None:
    cands = sorted((ROOT / "data").glob("static_*.csv"))
    return cands[-1] if cands else None


def combine(results: dict, kind: str):
    """把三轴的结果合成一个代表性数值（取中位），并检查一致性。"""
    vals = [results[k][kind] for k in results if results[k].get(kind)]
    if not vals:
        return None, None, None
    v = float(np.median(vals))
    spread = float(max(vals) / min(vals))
    return v, spread, vals


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    global SELFCHECK, PLOT
    if "--skip-selfcheck" in argv:
        SELFCHECK = False
        argv = [a for a in argv if a != "--skip-selfcheck"]
    if "--no-plot" in argv:
        PLOT = False
        argv = [a for a in argv if a != "--no-plot"]

    print("=" * 74)
    print("  IMU 噪声分析（Allan 方差）")
    print("=" * 74)
    print()

    if SELFCHECK and not run_selfcheck():
        print("  自检失败 → 分析器本身有问题，拒绝产出数字。")
        print("  （这是故意的：宁可不出数，也不要出一个看起来合理但是错的数）")
        return 1

    # ---- 输入 ----
    if argv:
        src = Path(argv[0]).expanduser()
    else:
        src = INPUT or find_latest_csv()
        if src is None:
            print(f"  ✗ {ROOT / 'data'} 里没有 static_*.csv")
            print("    → 先跑 record_static_imu.py 录一段（第一次可以只录 0.1 小时）")
            print("    → 或者直接把 CSV 路径当参数传进来：python analyze_static_noise.py <csv>")
            return 2
    if not src.exists():
        print(f"  ✗ 找不到文件 {src}")
        return 2

    print(f"  读取 {src}")
    if src.suffix == ".bin":
        t, data, meta = load_bin(src)
    else:
        t, data, meta = load_csv(src)
    print()

    # ---- 时间基准 ----
    fs, med, jitter, segs = timebase_report(t)
    if jitter > 0.10:
        # 实测：飞控 MAVLink 调度器会把发送时刻量化到几毫秒，87% 的间隔严格 = 5.000 ms，
        # 剩下 13% 落在 2~8 ms。这是"读取时刻"的抖动，不是采样值的抖动。
        # 对 τ ≥ 50 ms 的白噪声区（我们真正读数的区间）影响 < 6%，可以忽略；
        # 只有最短那几个 τ 点会失真，而那几点本来也不用。
        print(f"  ⚠️ 采样间隔抖动 {jitter:.1%}（飞控发送时刻的量化）—— "
              f"只影响 τ ≲ 20 ms 的短端，白噪声区（τ ≥ 50 ms）不受影响")
    a, b, dur = segs[0]
    t = t[a:b]
    data = {k: v[a:b] for k, v in data.items()}
    print()

    # ---- 分析 ----
    outdir = ROOT / "results" / datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir.mkdir(parents=True, exist_ok=True)
    results = {}
    for key in ("gx", "gy", "gz", "ax", "ay", "az"):
        if key not in data:
            continue
        print(f"  分析 {key} ……", end="", flush=True)
        ds = DATASHEET["gyroscope_noise_density" if key[0] == "g"
                       else "accelerometer_noise_density"]
        r = extract(data[key], fs, tag=key, plausible_max=ds * PLAUSIBLE_MAX_RATIO)
        results[key] = r
        print(f"  σ={_fmt(r['sigma'])}  K={_fmt(r['K'])}")
    print()

    gyro = {k: results[k] for k in ("gx", "gy", "gz") if k in results}
    accel = {k: results[k] for k in ("ax", "ay", "az") if k in results}

    # ---- 汇总 ----
    g_nd, g_spread, g_vals = combine(gyro, "sigma")
    a_nd, a_spread, a_vals = combine(accel, "sigma")
    g_rw, g_rw_spread, _ = combine(gyro, "K")
    a_rw, a_rw_spread, _ = combine(accel, "K")

    print("=" * 74)
    print("  结果")
    print("=" * 74)
    for label, vals in (("陀螺噪声密度", g_vals), ("加计噪声密度", a_vals)):
        if vals:
            print(f"  {label}: " + "  ".join(f"{v:.4e}" for v in vals))
    print()

    def line(name, got, spread, old, ds=None):
        if got is None:
            print(f"  {name:28s} ❌ 本次测不出")
            return
        s = f"  {name:28s} {got:.4e}"
        if spread:
            s += f"   三轴离散度 {spread:.2f}×"
        if old:
            s += f"   旧值 {old:.4e}（相差 {old / got:.1f}×）"
        if ds:
            s += f"   手册 {ds:.4e}（相差 {got / ds:.2f}×）"
        print(s)

    line("gyroscope_noise_density", g_nd, g_spread,
         CURRENT["gyroscope_noise_density"], DATASHEET["gyroscope_noise_density"])
    line("accelerometer_noise_density", a_nd, a_spread,
         CURRENT["accelerometer_noise_density"], DATASHEET["accelerometer_noise_density"])
    line("gyroscope_random_walk", g_rw, g_rw_spread, CURRENT["gyroscope_random_walk"])
    line("accelerometer_random_walk", a_rw, a_rw_spread, CURRENT["accelerometer_random_walk"])
    print()

    meas = dict(
        gyroscope_noise_density=g_nd, accelerometer_noise_density=a_nd,
        gyroscope_random_walk=g_rw, accelerometer_random_walk=a_rw)

    # ---- 判据 ----
    notes = []
    if g_nd is None:
        notes.append("陀螺噪声密度没测出：Allan 曲线里没有 -1/2 区。"
                     "常见原因是数据太短，或飞控低通把白噪声压掉了。")
    if a_nd is None:
        notes.append("加计噪声密度没测出：同上。")
    if g_rw is None or a_rw is None:
        notes.append("随机游走没测出：+1/2 区要 τ 到百秒量级，录制时长不够。"
                     "要测它得录 4~8 小时。")
    if g_spread and g_spread > 1.5:
        notes.append(f"陀螺三轴离散度 {g_spread:.2f}× 偏大，检查安装/轴对齐/温度。")
    if a_spread and a_spread > 1.5:
        notes.append(f"加计三轴离散度 {a_spread:.2f}× 偏大。")
    for label, vals in (("陀螺", g_vals), ("加计", a_vals)):
        if vals and len(vals) == 1:
            notes.append(f"{label}只有 1 个轴测出、另两轴测不出 —— 这种情况三轴一致性判据失效，"
                         "这个轴的值要小心看（多半是运动/振动把它抬起来了）。")
    for k, r in results.items():
        # 已经被 RATIO_MAX 拦下的轴不再重复报（它的 sigma 是 None，理由已在 sigma_info 里）
        ratio = (r.get("sigma_info") or {}).get("short_over_window")
        # 注意：飞控对 HIGHRES_IMU 做了低通（陀螺 20 Hz / 加计 10 Hz），
        # 短 τ 处的读数是被压低过的，所以比值天然 > 1：
        #     加计（10 Hz 低通）实测 12~13，陀螺（20 Hz）实测 5~6，都是正常的。
        #     真正说明"窗口选错"的是更大的比值（log_8 的 53~185、EuRoC 的 19~102）。
        # 所以这里只在远超合理范围时提示，判据主要靠三轴一致性和手册对照。
        if r.get("sigma") and ratio and (ratio > 20.0 or ratio < 0.33):
            notes.append(
                f"{k}: 拟合窗口给的 σ 是最短 τ 处直接读数的 {ratio:.1f} 倍 —— "
                "远超低通能解释的范围，选中的窗口可能不在白噪声区，这个 σ 要打折看。")
    if notes:
        print("  ⚠️ 需要注意")
        for n in notes:
            print(f"     · {n}")
        print()

    # ---- 输出 ----
    meta.update(duration_s=float(dur), fs=float(fs), jitter=float(jitter),
                n_samples=int(len(t)), generated=datetime.now().isoformat(timespec="seconds"))
    plots = plot_allan(gyro, accel, outdir)
    write_yaml(outdir / "kalibr_imu_chain.yaml", meas, meta, spreads={
        "gyroscope_noise_density": g_spread,
        "accelerometer_noise_density": a_spread,
        "gyroscope_random_walk": g_rw_spread,
        "accelerometer_random_walk": a_rw_spread,
    })
    payload = dict(meta=meta, measurements=meas, current=CURRENT, datasheet=DATASHEET,
                   axes=results, plots=plots, notes=notes)
    (outdir / "noise_results.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False))
    print(f"  ✅ {outdir / 'noise_results.json'}")
    print()

    print("=" * 74)
    print("  下一步")
    print("=" * 74)
    print(f"  1) 把 {outdir / 'kalibr_imu_chain.yaml'} 覆盖到：")
    print("       ~/vio_ws/final_config/kalibr_imu_chain.yaml")
    print("       ~/calib/ov_config_session05/kalibr_imu_chain.yaml")
    print("  2) 重跑 session05 的 OpenVINS，和新旧参数对比")
    print(f"  3) 曲线图看一眼：{outdir}/allan_gyro.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
