"""飞控 `time_usec` → 宿主 REALTIME 的时间基准拟合（`imu_recorder.py` 用）。

物理模型
--------
样本真实时刻 τ（飞控自己记的 `HIGHRES_IMU.time_usec`，开机微秒）；
宿主收到消息的时刻 H = τ + L，L ≥ 0 是"飞控输出 + 线缆 + OS 缓冲 + 读线程积压"
的总延迟。L 只会让时间戳偏晚，所以 **min over 全段 (H − τ)** 才是"最不拥堵那一次"
的延迟 —— 它才可能跨录制复现。

为什么不再用"每批 read 时间"当锚点
----------------------------------
旧方案给每批样本打本次 read 的宿主时间，批内用 `time_usec` 反推。批内间隔是准的，
但每批的锚点带着"这一批在 OS 缓冲里躺了多久"。2026-10-18 实测六次录制，同一台设备
估出的相机-IMU 时移在 −27 … +9 ms 之间跳，而相机侧到包延迟恒定 28.1~28.8 ms ——
变的就是这条链。标定只能估一个**常数**时移：常数偏差会被吸收，**变化**的偏差会直接
污染外参/时移。

做法
----
1. 批内仍用飞控 `time_usec` 定相对时间（不变）；
2. 全段录完后，用 (H − τ) 的**下包络**拟合一条直线 host = a + (1+b)·τ，
   抗住"开头积压""偶尔卡顿"这类成片的坏点；下包络的截距就是最小延迟；
3. 所有样本时间戳改成 τ + 该映射。剩下的固定延迟（线缆 + 飞控输出）是个常数，
   由标定的 offset 搜索吸收 —— 这正是我们想要的。

本模块不碰硬件、不读文件，纯运算，便于单测。
"""
from __future__ import annotations

from itertools import combinations

# 下包络截距用 0.1% 分位代替严格 min：一两帧被污染的时延不该把整段拉走。
_FLOOR_QUANTILE = 0.001
# 飞控/宿主晶振频率差再大也到不了 500 ppm；超了说明数据有问题，放弃锚定。
_MAX_DRIFT_PPM = 500.0
# 锚定前后时间戳差超过这个量说明拟合跑飞了（正常只会被回拨几百 ms 的积压）。
_MAX_SHIFT_NS = 5_000_000_000


def _median(values):
    s = sorted(values)
    n = len(s)
    if n == 0:
        return 0.0
    return float(s[n//2]) if n % 2 else 0.5*(s[n//2-1]+s[n//2])


def _quantile(sorted_values, q):
    if not sorted_values:
        return 0.0
    k = int(round(q*(len(sorted_values)-1)))
    return sorted_values[min(len(sorted_values)-1, max(0, k))]


def fit_timebase(fc_usec, host_ns, *, min_samples=50, max_bins=64):
    """对 (飞控时间, 宿主收到时间) 做下包络直线拟合。

    返回 dict：ok / reason / offset_ns / drift / f0_usec / drift_ppm / span_s /
    bins / delay_median_ms / delay_p95_ms。ok=False 时调用方保留旧时间戳。
    """
    pairs = [(int(f), int(h)) for f, h in zip(fc_usec, host_ns) if f > 0 and h > 0]
    if len(pairs) < min_samples:
        return {"ok": False, "reason": f"有飞控时间的样本只有 {len(pairs)} 个"}
    f0 = pairs[0][0]
    last = None
    for f, _h in pairs:
        if last is not None and f <= last:
            return {"ok": False, "reason": "飞控 time_usec 不严格递增（重启/回绕？）"}
        last = f
    span_ns = (pairs[-1][0]-f0)*1000
    if span_ns < 1_000_000_000:
        return {"ok": False, "reason": f"跨度只有 {span_ns/1e9:.2f} s，不足以拟合"}
    offsets = [h-f*1000 for f, h in pairs]          # 每帧的总延迟，ns
    # 等样本数分箱，取每箱最小值 → 去掉"某一批恰好卡住"这样的局部坏点。
    k = max(4, min(max_bins, len(pairs)//50))
    binned = []
    for i in range(k):
        a, b = i*len(pairs)//k, (i+1)*len(pairs)//k
        if b <= a:
            continue
        j = min(range(a, b), key=offsets.__getitem__)
        binned.append(((pairs[j][0]-f0)*1000, offsets[j]))   # x: 相对飞控时间 ns
    if len(binned) < 4:
        return {"ok": False, "reason": "下包络样本不足"}
    # Theil-Sen 斜率：抗"开头整段积压"这类成片坏点（普通最小二乘会被它带偏）。
    slopes = [(y2-y1)/(x2-x1) for (x1, y1), (x2, y2) in combinations(binned, 2) if x2 > x1]
    drift = _median(slopes) if slopes else 0.0
    drift_ppm = drift*1e6
    if abs(drift_ppm) > _MAX_DRIFT_PPM:
        return {"ok": False, "reason": f"飞控/宿主时钟频率差 {drift_ppm:.0f} ppm 异常"}
    corrected = sorted((h-f*1000)-drift*(f-f0)*1000 for f, h in pairs)
    floor_ns = _quantile(corrected, _FLOOR_QUANTILE)
    delays = sorted(d-floor_ns for d in corrected)     # 相对最小延迟的分布
    return {
        "ok": True, "reason": "",
        "offset_ns": floor_ns, "drift": drift, "f0_usec": f0,
        "drift_ppm": drift_ppm, "span_s": span_ns/1e9, "bins": len(binned),
        "delay_median_ms": _quantile(delays, 0.5)/1e6,
        "delay_p95_ms": _quantile(delays, 0.95)/1e6,
    }


def anchor_timestamps(fc_usec, host_ns, provisional_ns, *, min_samples=50):
    """把样本时间戳改成"飞控时间 + 下包络映射"。

    返回 (timestamps, info)。拟合不成立时原样返回 provisional_ns 并给 ok=False，
    调用方据此在报告里注明"回退到读时间"。
    info 里 shift_*_ms = 旧（读时间）时间戳被回拨了多少 —— 直接就是读线程积压的量。
    """
    provisional = [int(t) for t in provisional_ns]
    fit = fit_timebase(fc_usec, host_ns, min_samples=min_samples)
    if not fit["ok"]:
        return provisional, {"ok": False, "reason": fit["reason"],
                             "fallback_samples": len(provisional), "shift_median_ms": None}
    offset, drift, f0 = fit["offset_ns"], fit["drift"], fit["f0_usec"]
    out, last, clamped, fallback = [], None, 0, 0
    for f, prov in zip(fc_usec, provisional):
        f = int(f)
        if f > 0:
            x = (f-f0)*1000
            t = int(round(f*1000 + offset + drift*x))
        else:                                   # 这一帧没有飞控时间，只能留旧值
            t = prov
            fallback += 1
        if last is not None and t <= last:      # 标定程序要求严格递增
            t = last + 1
            clamped += 1
        last = t
        out.append(t)
    shifts = sorted(p-t for p, t in zip(provisional, out))
    info = {**fit,
            "shift_median_ms": _quantile(shifts, 0.5)/1e6,
            "shift_p95_ms": _quantile(shifts, 0.95)/1e6,
            "shift_max_ms": shifts[-1]/1e6,
            "clamped": clamped, "fallback_samples": fallback}
    if max(abs(s) for s in shifts) > _MAX_SHIFT_NS:
        return provisional, {"ok": False, "reason": "锚定前后时间戳差超过 5 s，疑似拟合异常",
                             "fallback_samples": len(provisional), "shift_median_ms": None}
    return out, info
