"""连续窗口选择与检出诊断。

为什么单独一个模块：
    `Problem`/`check_visual` 只该回答"这份数据能不能拟合"。而"这段录像里哪一段
    能标、为什么其余段落不能用"是采集质量问题，必须在求解之前用数据说清楚。
    2026-10-18 之前，整段录像里只要有一处 >max_camera_gap_s 的检出空洞，
    整次标定就在跑完几分钟检测之后被一句"大空洞"打回，报告里连空洞在哪、
    多长、有几个都没有 —— 于是每个 session 都要人肉挑窗口再来一遍。

这里只做纯时间序列运算，不读图像、不依赖 OpenCV，方便单元测试。
"""
from __future__ import annotations

import numpy as np

# 白格亮度（p90）低于这个数就是"太暗"，和 capture/image_health.py 用同一个判据：
# 得先加光 —— 不是降曝光、也不是运动模糊。
DARK_P90 = 45.0


def continuous_runs(times, max_gap_s):
    """把时间序列按相邻间隔 <= max_gap_s 切成连续段，返回 [(起索引, 止索引)]（含两端）。"""
    times = np.asarray(times, dtype=float)
    if times.size == 0:
        return []
    cuts = np.flatnonzero(np.diff(times) > max_gap_s)
    starts = np.r_[0, cuts+1]
    stops = np.r_[cuts, times.size-1]
    return list(zip(starts.tolist(), stops.tolist()))


def gaps_over(times, max_gap_s):
    """超过门槛的空洞 [(起点秒, 长度秒)]，按长度降序。"""
    times = np.asarray(times, dtype=float)
    if times.size < 2:
        return []
    idx = np.flatnonzero(np.diff(times) > max_gap_s)
    out = [(float(times[i]), float(times[i+1]-times[i])) for i in idx]
    return sorted(out, key=lambda g: g[1], reverse=True)


def run_info(times, a, b):
    """把 (起索引, 止索引) 变成人能读的窗口描述。"""
    times = np.asarray(times, dtype=float)
    return {"start_s": float(times[a]), "stop_s": float(times[b]),
            "duration_s": float(times[b]-times[a]), "frames": int(b-a+1),
            "start_index": int(a), "stop_index": int(b)}


def qualifying_runs(times, *, max_gap_s, min_frames, min_duration_s):
    """所有满足帧数/时长下限的连续段，最长的在前（同长取帧数多的）。"""
    runs = [run_info(times, a, b) for a, b in continuous_runs(times, max_gap_s)]
    ok = [r for r in runs if r["frames"] >= min_frames and r["duration_s"] >= min_duration_s]
    return sorted(ok, key=lambda r: (r["duration_s"], r["frames"]), reverse=True)


def used_mask(extraction):
    """哪些检测帧真正进入了求解。

    load_dataset 现在写 "kept"（检出 → PnP 合格 → 通过 camera_rate_hz 抽稀）；
    老 extraction.json 只有 accepted/detected，按能拿到的最高精度退化。
    """
    if all("kept" in e for e in extraction):
        return np.array([bool(e["kept"]) for e in extraction], dtype=bool)
    if any("accepted" in e for e in extraction):
        return np.array([bool(e.get("accepted", e.get("detected"))) for e in extraction], dtype=bool)
    return np.array([bool(e.get("detected")) for e in extraction], dtype=bool)


def visual_diagnostics(times, extraction, *, max_gap_s, min_frames, min_duration_s):
    """汇总"为什么这批帧能用/不能用"，可直接写进 report.json。

    times        进入求解的帧的时间，秒，相对时间原点
    extraction   load_dataset 逐个检测帧写下的记录
                 （timestamp_ns/detected/accepted/kept/pnp_rms_px/sharpness）
    """
    times = np.asarray(times, dtype=float)
    detected = np.array([bool(e.get("detected")) for e in extraction], dtype=bool) if extraction else np.zeros(0, bool)
    # "accepted" = 检出且 PnP 达标（抽稀前）；"kept" = 真正进入求解（抽稀后）。
    # 两个数不一样不是矛盾：camera_rate_hz 就是要把帧抽稀。
    pnp_passed = np.array([bool(e.get("accepted", e.get("detected"))) for e in extraction],
                          dtype=bool) if extraction else np.zeros(0, bool)
    sharp = np.array([np.nan if e.get("sharpness") is None else float(e["sharpness"])
                      for e in extraction], dtype=float) if extraction else np.zeros(0)
    bright = np.array([np.nan if e.get("p90") is None else float(e["p90"])
                       for e in extraction], dtype=float) if extraction else np.zeros(0)

    runs = [run_info(times, a, b) for a, b in continuous_runs(times, max_gap_s)] if times.size else []
    runs.sort(key=lambda r: (r["duration_s"], r["frames"]), reverse=True)
    gaps = gaps_over(times, max_gap_s)
    best = qualifying_runs(times, max_gap_s=max_gap_s, min_frames=min_frames,
                           min_duration_s=min_duration_s)
    # 清晰度对比的是"检出+PnP 合格"与"被拒"（没检出或 PnP 超门槛）。
    # 不要拿抽稀掉的帧当"被拒"：它们只是被 camera_rate_hz 降采样，清晰度跟合格帧一样，
    # 混进去会把模糊信号稀释掉（实测 64% 会被稀释成 85%，结论直接消失）。
    accepted_sharp = float(np.median(sharp[pnp_passed])) if np.isfinite(sharp[pnp_passed]).any() else None
    rejected_sharp = float(np.median(sharp[~pnp_passed])) if np.isfinite(sharp[~pnp_passed]).any() else None
    ratio = (rejected_sharp/accepted_sharp) if accepted_sharp and rejected_sharp else None
    # 亮度与"糊"必须分开判：板子没光照到时 Laplacian 方差同样会掉，
    # 只报"运动模糊"会把人引到错误的方向（2026-10-18 房间关灯就是这种情况）。
    # 一帧都没通过时 accepted 组是空的，必须退到"所有检测帧"的亮度，
    # 否则最该报"太暗"的那种数据反而什么诊断都没有。
    accepted_p90 = float(np.median(bright[pnp_passed])) if np.isfinite(bright[pnp_passed]).any() else None
    rejected_p90 = float(np.median(bright[~pnp_passed])) if np.isfinite(bright[~pnp_passed]).any() else None
    all_p90 = float(np.median(bright[np.isfinite(bright)])) if np.isfinite(bright).any() else None
    reference_p90 = accepted_p90 if accepted_p90 is not None else all_p90
    too_dark = bool(reference_p90 is not None and reference_p90 < DARK_P90)
    diag = {
        "detection_samples": int(len(extraction)),
        "detected": int(detected.sum()),
        "detected_rate": float(detected.mean()) if detected.size else 0.0,
        "pnp_passed": int(pnp_passed.sum()),
        "fitted_frames": int(times.size),
        "fitted_rate": float(times.size/len(extraction)) if extraction else 0.0,
        "fitted_span_s": float(times[-1]-times[0]) if times.size else 0.0,
        "sample_spacing_median_s": float(np.median(np.diff(times))) if times.size > 1 else None,
        "max_gap_s": float(gaps[0][1]) if gaps else 0.0,
        "gaps_over_limit_s": [{"start_s": t, "gap_s": g} for t, g in gaps],
        "runs": runs[:5],
        "best_run": best[0] if best else None,
        "sharpness": {"accepted_median": accepted_sharp, "rejected_median": rejected_sharp,
                      "rejected_to_accepted_ratio": ratio},
        "brightness": {"accepted_p90": accepted_p90, "rejected_p90": rejected_p90,
                       "all_p90": all_p90},
        "too_dark": too_dark,
        "motion_blur_suspected": bool(not too_dark and ratio is not None and ratio < 0.7),
    }
    return diag


def diagnostics_from_extraction(extraction, *, max_gap_s, min_frames, min_duration_s):
    """load_dataset 抛错时（例如有效帧数不够）也要能出诊断：从 extraction.json 重建。"""
    if not extraction:
        return None
    stamps = np.array([e["timestamp_ns"] for e in extraction], dtype=np.int64)
    used = used_mask(extraction)
    epoch = stamps[used][0] if used.any() else stamps[0]
    times = (stamps[used]-epoch).astype(float)*1e-9
    return visual_diagnostics(times, extraction, max_gap_s=max_gap_s,
                              min_frames=min_frames, min_duration_s=min_duration_s)


def describe_problem(diag, settings):
    """把诊断翻译成"下一步该动什么"，一行一条，给 stderr 和失败报告用。"""
    lines = []
    if diag["gaps_over_limit_s"]:
        first = diag["gaps_over_limit_s"][:6]
        shown = "  ".join(f"{g['start_s']:.2f}s处{g['gap_s']:.2f}s" for g in first)
        lines.append(f"检出空洞 {len(diag['gaps_over_limit_s'])} 处（门槛 {settings.max_camera_gap_s:g}s），"
                     f"最大 {diag['max_gap_s']:.2f}s：{shown}")
    best = diag["best_run"]
    if best:
        lines.append(f"最长可用连续段 {best['start_s']:.2f}s→{best['stop_s']:.2f}s"
                     f"（{best['duration_s']:.2f}s，{best['frames']} 帧）")
    else:
        runs = diag["runs"]
        if runs:
            longest = runs[0]
            lines.append(f"最长连续段只有 {longest['duration_s']:.2f}s / {longest['frames']} 帧，"
                         f"低于下限 {settings.min_duration_s:g}s / {settings.min_frames} 帧；"
                         "整段没有任何一段能标")
    brightness = diag.get("brightness") or {}
    # 一帧都没通过时，accepted 组是空的，退到"所有检测帧"的亮度再判一次。
    p90 = brightness.get("accepted_p90")
    if p90 is None:
        p90 = brightness.get("all_p90")
    if diag["detection_samples"] and diag["detected"] == 0:
        lines.append(f"{diag['detection_samples']} 帧里一个 tag 都没解出：先确认板子在画面里、"
                     "tag 边长 ≥30 px、板型/行列/方向与 configs 下的目标 YAML 一致")
    if p90 is not None and p90 < DARK_P90:
        # 先判亮度再判糊：同一份"清晰度掉了"，暗和糊的处理方式完全相反。
        lines.append(f"整段太暗：白格亮度 p90 中位 {p90:.0f}/255（判据 <{DARK_P90:.0f}）→ "
                     "检出掉到 0 是因为没光，不是运动模糊")
        lines.append("  先开灯/加一盏补光灯正对板子，用 capture/framing_check.py 把白格 p90 做到 60~150；"
                     "房间暗的时候降曝光只会更黑")
        lines.append("  实在不能加光：EXPOSURE 200→250~300（30fps 下约 330 是上限）、GAIN 10→30，"
                     "但曝光变长会更容易糊，得配套放慢转动")
    elif diag["motion_blur_suspected"]:
        ratio = diag["sharpness"]["rejected_to_accepted_ratio"]
        lines.append(f"被拒帧清晰度只有合格帧的 {100*ratio:.0f}%（{diag['sharpness']['rejected_median']:.0f}"
                     f" vs {diag['sharpness']['accepted_median']:.0f}）→ 运动模糊：转太快，或曝光时间太长")
        lines.append("  先确认光够（白格 p90 60~150）再放慢转动：标定只要约 3°/s 的多轴激励，"
                     "慢下来完全够；光是够的就别降曝光")
        lines.append("  整块板子要全程留在画面里，凑出连续 15~20 秒的可检出段")
    return lines
