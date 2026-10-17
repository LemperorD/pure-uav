"""看一眼采集到的图：亮度够不够、板子认不认得出、是不是出画了。

被 `framing_check.py`（取景体检）和 `record_session.py`（采完自动体检）共用。

为什么需要它：预览窗为了帧率被关掉了，于是采集变成"盲拍"。这个模块就是那双眼睛 ——
采完立刻把最关键的三件事说出来，不用等标定跑几分钟才发现板子根本没进画面。
"""
from __future__ import annotations

import cv2
import numpy as np


def _pick(paths, n):
    paths = list(paths)
    if len(paths) <= n:
        return paths
    idx = np.linspace(0, len(paths) - 1, n).astype(int)
    return [paths[i] for i in idx]


def _gray(path):
    return cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)


# --------------------------------------------------------------------------
# 亮度
# --------------------------------------------------------------------------

def brightness_stats(paths, sample=8):
    picks = _pick(paths, sample)
    rows = []
    for p in picks:
        g = _gray(p)
        if g is None:
            continue
        q = np.percentile(g, [50, 90, 99])
        rows.append((q[0], q[1], q[2], g.mean()))
    if not rows:
        return None
    a = np.asarray(rows)
    return {"n": len(rows), "p50": float(np.median(a[:, 0])),
            "p90": float(np.median(a[:, 1])), "p99": float(np.median(a[:, 2])),
            "mean": float(np.median(a[:, 3]))}


def brightness_verdict(b):
    """参考值来自你自己的历史采集（实测）：
         内参成功那次 1080p：p50=32 p90=68
         iPad 屏那次 720p  ：p50=20 p90=100
         打印板那次 720p   ：p50=16 p90=30（很暗但也标成了）
       所以判据不看 p50，看 **p90** —— 它代表"白格有多亮"。"""
    if b is None:
        return "unknown", "读不到图"
    if b["p90"] < 45:
        return "dark", f"p90={b['p90']:.0f}，太暗（白格应该到 60~150）"
    if b["p90"] > 235:
        return "bright", f"p90={b['p90']:.0f}，可能过曝（白格糊成一片）"
    return "ok", f"p90={b['p90']:.0f}，亮度可用"


# --------------------------------------------------------------------------
# 检出
# --------------------------------------------------------------------------

def _detect(gray, cols, rows):
    """和标定程序 calib/data.py 的 Target.detect 同一套策略：SB 优先，经典兜底。"""
    try:
        ok, pts, _ = cv2.findChessboardCornersSBWithMeta(
            gray, (cols, rows), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    except Exception:                                        # noqa: BLE001
        ok, pts = False, None
    kind = "SB"
    if not ok:
        ok, pts = cv2.findChessboardCorners(gray, (cols, rows))
        kind = "经典"
    return (pts.reshape(-1, 2).astype(float) if ok else None), (kind if ok else None)


def detect_stats(paths, cols, rows, sample=12, probe_subgrids=True):
    """返回检出率；顺带探一下"小一号"的网格能不能检出。

    这条探针很有用：整块板子出画时，小网格仍可能检出。
    所以『(cols,rows) 全灭但 (cols-1,rows-1) 能检出』= 板子被画面切掉了，
    而不是板子不对或太暗。
    """
    picks = _pick(paths, sample)
    tried = hit = 0
    kinds, margins, spans, spans_v, board_white = set(), [], [], [], []
    shape = None
    sharp_hit, sharp_miss = [], []
    for p in picks:
        g = _gray(p)
        if g is None:
            continue
        shape = g.shape
        tried += 1
        pts, kind = _detect(g, cols, rows)
        sharp = float(cv2.Laplacian(g, cv2.CV_64F).var())
        if pts is not None:
            hit += 1
            kinds.add(kind)
            margins.append(_edge_margin(pts, g.shape))
            sharp_hit.append(sharp)
            spans.append(float(pts[:, 0].max() - pts[:, 0].min()))
            spans_v.append(float(pts[:, 1].max() - pts[:, 1].min()))
            # 板子区域内的"白格"亮度 —— 用来分清【屏幕太暗】和【房间太暗】
            x0, x1 = int(pts[:, 0].min()), int(pts[:, 0].max())
            y0, y1 = int(pts[:, 1].min()), int(pts[:, 1].max())
            roi = g[max(0, y0):y1, max(0, x0):x1]
            if roi.size > 500:
                board_white.append(float(np.percentile(roi, 90)))
        else:
            sharp_miss.append(sharp)
    rate = hit / tried if tried else 0.0
    med = lambda a: float(np.median(a)) if a else None            # noqa: E731
    out = {"tried": tried, "detected": hit, "rate": rate,
           "kinds": sorted(kinds),
           "min_edge_margin_px": min(margins) if margins else None,
           "board_width_px": med(spans), "board_height_px": med(spans_v),
           # 面积法，与板子在画面里转了 90° 无关：(cols+1)x(rows+1) 个格子铺满 bbox
           "square_px": (float(np.sqrt(med(spans) * med(spans_v) / ((cols + 1) * (rows + 1))))
                         if (med(spans) and med(spans_v)) else None),
           "board_white": med(board_white),
           "shape": shape, "subgrid": {}, "cut_off_frames": 0,
           "sharp_median": med(sharp_hit + sharp_miss),
           "sharp_hit": med(sharp_hit), "sharp_miss": med(sharp_miss)}

    # 【切边探针】对【没检出】的那些帧试小一号的网格。
    # 小网格能检出、完整网格不能 ⟹ 只有一部分板子在画面里 —— 板子被画面切掉了。
    # 这是 2026-10-15 那次失败的真凶：转动时棋盘漂到画面边缘，
    # 连丢 15 帧 = 0.69 s 盲区，直接触发 max_camera_gap_s=0.35。
    # 注意它和延迟/转速都无关，把 PnP 门槛完全放开也照样盲。
    if probe_subgrids and tried and hit < tried:
        misses = [p for p in picks if _detect(_gray(p), cols, rows)[0] is None
                  and _gray(p) is not None]
        for dc, dr in ((1, 1), (2, 2)):
            c2, r2 = cols - dc, rows - dr
            if min(c2, r2) < 4:
                continue
            sub = 0
            for p in misses[:8]:
                g = _gray(p)
                if g is None:
                    continue
                if _detect(g, c2, r2)[0] is not None:
                    sub += 1
            out["subgrid"][f"{c2}x{r2}"] = sub
        if any(v > 0 for v in out["subgrid"].values()):
            out["cut_off_frames"] = max(out["subgrid"].values())
    return out


def _edge_margin(pts, shape):
    """板子离画面边缘最近的像素距离。贴边 = 随时可能被切掉。"""
    h, w = shape[:2]
    return float(min(pts[:, 0].min(), pts[:, 1].min(),
                     w - 1 - pts[:, 0].max(), h - 1 - pts[:, 1].max()))


# --------------------------------------------------------------------------
# 汇总结论
# --------------------------------------------------------------------------

def full_timeline(paths, camera_yaml, target_yaml, max_pnp_rms=2.0, stride=1,
                  gap_s=0.35, min_frames=35, min_duration_s=5.0, log=None):
    """对【每一帧】跑标定程序自己的检测 + PnP + 那道 PnP 门槛，
    算出它真正会接受哪些帧、以及由此产生的**空洞结构**。

    这是唯一能提前预测 `max_camera_gap_s` 会不会触发的方法 ——
    采样 12 张的"检出率"看不出来：86% 检出率完全可能被 4 处 0.5~1.9 秒的
    盲区毁掉（2026-10-15 就是这样白跑了三次标定，每次 5 分钟）。
    """
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parent.parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from calib.data import Camera, Target, make_observation, read_yaml

    cam = Camera.load(camera_yaml)
    tgt = Target(read_yaml(target_yaml))
    checker = tgt.kind in ("checkerboard", "chessboard")
    paths = list(paths)[::max(1, stride)]

    stamps, accepted = [], []
    detected = 0
    prev = None
    for i, p in enumerate(paths):
        g = _gray(p)
        if g is None:
            continue
        if log and (i % 25 == 0):
            log(f"    逐帧检测 {i}/{len(paths)} …")
        # 文件名就是宿主纳秒时间戳
        try:
            stamp = int(Path(p).stem)
        except ValueError:
            stamp = i
        res = tgt.detect(g)
        if res is None:
            continue
        detected += 1
        obs = make_observation(stamp, res[0], res[1], cam, prev,
                               checker=checker, max_points=100)
        if obs is None or obs.pnp_rms > max_pnp_rms:
            continue
        prev = obs
        stamps.append(stamp)
        accepted.append(obs.pnp_rms)

    out = {"frames": len(paths), "detected": detected, "accepted": len(stamps),
           "gaps_ms": [], "max_gap_ms": 0.0, "gaps_over": [], "windows": []}
    if len(stamps) < 2:
        return out
    s = np.asarray(stamps, dtype=np.int64)
    t = (s - s[0]) / 1e9
    g = np.diff(t)
    out["gaps_ms"] = (g * 1e3).tolist()
    out["max_gap_ms"] = float(g.max() * 1e3)
    out["span_s"] = float(t[-1])
    over = np.flatnonzero(g > gap_s)
    out["gaps_over"] = [(float(t[i]), float(g[i] * 1e3)) for i in over]
    bounds = [0] + list(over + 1) + [len(t)]
    for a, b in zip(bounds[:-1], bounds[1:]):
        span = t[b - 1] - t[a]
        n = b - a
        if span >= min_duration_s and n >= min_frames:
            out["windows"].append((float(t[a]), float(span), int(n)))
    out["best"] = max(out["windows"], key=lambda w: w[1]) if out["windows"] else None
    out["pnp_median"] = float(np.median(accepted))
    return out


def report_timeline(tl, log=print):
    """把 full_timeline 的结果打印成'这份数据能不能过标定'。返回 (ok, 说明)。"""
    if tl["accepted"] < 2:
        log(f"  逐帧复算        只有 {tl['accepted']} 帧被接受 —— 完全不够")
        return False, "接受帧太少"
    log(f"  逐帧复算        {tl['frames']} 帧里：检出 {tl['detected']}，"
        f"标定程序会接受 {tl['accepted']}（PnP 中位 {tl['pnp_median']:.2f} px）")
    log(f"  最大盲区        {tl['max_gap_ms']:.0f} ms（门槛 350 ms）")
    if tl["gaps_over"]:
        detail = "  ".join(f"{a:.2f}s起{g:.0f}ms" for a, g in tl["gaps_over"][:5])
        log(f"  超标盲区        {len(tl['gaps_over'])} 处：{detail}")
    if tl["best"]:
        a, span, n = tl["best"]
        log(f"  最长可用窗口    {a:.2f}s 起，{span:.2f}s，{n} 帧  ← 可以只跑这个窗口")
        return True, f"有可用窗口：start_s={a:.2f}, max_duration_s={span:.2f}"
    log("  最长可用窗口    无 —— 需要一段【连续】≥5 秒 且 ≥35 帧")
    return False, "没有连续可用窗口"


def analyze(paths, cols, rows, sample=12):
    paths = list(paths)
    b = brightness_stats(paths, min(8, sample))
    d = detect_stats(paths, cols, rows, sample=sample)
    return {"images": len(paths), "brightness": b, "detection": d,
            "board": f"{cols}x{rows}", "quality": None}


def calib_quality_stats(paths, camera_yaml, target_yaml, window=24):
    """用标定程序【自己的】Camera / Target / make_observation 跑一段连续帧，
    报出它真正会接受多少帧。

    为什么非要这一步：**检出 ≠ 可用**。2026-10-15 那次采集，检测器检出 80/97（82%），
    但其中 17 帧的 PnP 残差是 3~4 px（门槛 2.0），被标定程序剔掉 ——
    最终只有 63 帧，还留下了 5 处 >0.35 s 的空洞，整份数据作废。
    静态的检出率、亮度全都看不出来，**只有 PnP 残差看得出**（那些帧糊了）。
    """
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parent.parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from calib.data import Camera, Target, make_observation, read_yaml

    cam = Camera.load(camera_yaml)
    tgt = Target(read_yaml(target_yaml))
    checker = tgt.kind in ("checkerboard", "chessboard")

    paths = list(paths)
    if len(paths) > window:                      # 取中间一段连续帧
        start = (len(paths) - window) // 2
        paths = paths[start:start + window]

    n_det = n_ok = n_bad = 0
    rms_bad, sharp_ok, sharp_bad = [], [], []
    prev = None
    for p in paths:
        g = _gray(p)
        if g is None:
            continue
        res = tgt.detect(g)
        if res is None:
            continue
        n_det += 1
        obs = make_observation(0, res[0], res[1], cam, prev,
                               checker=checker, max_points=100)
        sharp = float(cv2.Laplacian(g, cv2.CV_64F).var())
        if obs is None or obs.pnp_rms > 2.0:
            n_bad += 1
            if obs is not None:
                rms_bad.append(obs.pnp_rms)
            sharp_bad.append(sharp)
            continue
        n_ok += 1
        sharp_ok.append(sharp)
        prev = obs
    med = lambda a: float(np.median(a)) if a else None            # noqa: E731
    return {"window": len(paths), "detected": n_det, "accepted": n_ok, "rejected": n_bad,
            "effective_rate": n_ok / len(paths) if paths else 0.0,
            "pnp_bad_median": med(rms_bad),
            "sharp_ok": med(sharp_ok), "sharp_bad": med(sharp_bad)}


def report(res, log=print):
    """把分析结果打印成人话 + 可执行的动作。返回 (level, [动作])。"""
    b, d = res["brightness"], res["detection"]
    b_level, b_text = brightness_verdict(b)
    actions = []

    log(f"  图片张数        {res['images']}")
    if b:
        log(f"  亮度            p50={b['p50']:.0f}  p90={b['p90']:.0f}  p99={b['p99']:.0f}"
            f"  均值={b['mean']:.1f}")
        log(f"                  {b_text}")
    log(f"  检出 ({res['board']})     {d['detected']}/{d['tried']}"
        f"  ({100*d['rate']:.0f}%)" + (f"  [{','.join(d['kinds'])}]" if d["kinds"] else ""))
    if d["min_edge_margin_px"] is not None:
        log(f"  板子离画面边缘   {d['min_edge_margin_px']:.0f} px")
    if d.get("board_white") is not None:
        log(f"  标定板白格亮度   {d['board_white']:.0f}/255"
            + ("   ← 板子本身太暗" if d["board_white"] < 100 else ""))
    if d.get("square_px"):
        log(f"  板子在画面里     {d['board_width_px']:.0f} × {d['board_height_px']:.0f} px，"
            f"每个格子约 {d['square_px']:.0f} px"
            "  ← 太小会让姿态解退化（目标 ~90 px）")
    if d["sharp_median"] is not None:
        extra = ""
        if d["sharp_hit"] and d["sharp_miss"]:
            extra = (f"（检出的 {d['sharp_hit']:.0f} vs 没检出的 {d['sharp_miss']:.0f}）")
        log(f"  画面清晰度       {d['sharp_median']:.0f}{extra}  ← Laplacian 方差，越大越锐")
    if d["subgrid"]:
        log(f"  小网格探针       " + "  ".join(f"({k}) {v}/6" for k, v in d["subgrid"].items()))
    if d.get("cut_off_frames"):
        log(f"  ⚠️ 其中 {d['cut_off_frames']} 帧是「完整网格检不出、小网格能检出」")

    # 完整网格检不出、小网格能检出 —— 说明画面里还有棋盘结构，但不够整块。
    # ⚠️ 这条【不能断言是切边】：2026-10-15 我提亮原图看过，有的帧棋盘完整可见，
    # 只是视角很斜 + 卷帘畸变，同样会让完整网格失败而小网格幸存。
    # 所以这里只报现象，把三种可能都列出来，不要误导人去改错的地方。
    if d.get("cut_off_frames"):
        actions.append(f"★ 有 {d['cut_off_frames']} 帧完整网格检不出、小网格能检出"
                       "（说明画面里还有棋盘结构，但不够整块）。三种可能，按可能性排：\n"
                       "     a) 转动时棋盘漂到画面边缘被切掉 —— 往后退，让棋盘只占画面 1/4~1/3\n"
                       "     b) 视角太斜（格子被压扁）—— 让板子尽量正对镜头\n"
                       "     c) 那一瞬间糊了 —— 转慢一点\n"
                       "   三种都指向同一件事：慢一点、退远一点、别斜着瞟")

    # 【板子太小 → 姿态解退化】2026-10-15 真机踩到的根本原因：
    # 板子只有 263 px 宽、每格 33 px，而内参成功那次是 742 px / 93 px。
    # 结果：画面运动明明和陀螺一致（5.2 vs 5.0 px/帧），PnP 姿态却每帧跳 1.3° ——
    # 平面 PnP 的经典退化（绕面内轴转一点 + 沿光轴平移一点，投影几乎不变）。
    # 表观 38°/s vs 陀螺 6.5°/s，工具无从调和，直接崩。
    if d.get("square_px") and d["square_px"] < 55:
        actions.append(f"★ 板子在画面里太小了（每格只有 {d['square_px']:.0f} px）。"
                       "这会让 PnP 姿态解退化：画面几乎没动，姿态却能跳 1° 以上，"
                       "于是'相机轨迹推出来的角速度'和陀螺差好几倍，标定必然失败。"
                       f"【靠近】到每格约 90 px（参考：内参成功那次板宽 742 px / 每格 93 px）")

    q = res.get("quality")
    if q is not None:
        log(f"  标定程序实际会用   {q['accepted']}/{q['window']} 帧"
            f"（检出 {q['detected']}，检出但 PnP>{2.0}px 被剔 {q['rejected']}）")
        if q["rejected"] and q["pnp_bad_median"]:
            log(f"  被剔帧的 PnP 残差  中位 {q['pnp_bad_median']:.2f} px")
        if q["sharp_ok"] and q["sharp_bad"]:
            log(f"  清晰度对比       合格的 {q['sharp_ok']:.0f}  vs 被剔的 {q['sharp_bad']:.0f}")

    # 糊：被 PnP 剔掉的帧明显比合格帧锐度低 → 运动模糊。
    # 这是**唯一**能预测"标定会不会过"的指标 —— 静态的检出率和亮度都看不出运动模糊。
    blur = bool(q and q["rejected"] and q["sharp_ok"] and q["sharp_bad"]
                and q["sharp_bad"] < 0.7 * q["sharp_ok"])
    if blur:
        actions.append(f"★ 画面糊了：合格的帧清晰度 {q['sharp_ok']:.0f}，"
                       f"被剔的只有 {q['sharp_bad']:.0f}"
                       f"（{100*q['sharp_bad']/q['sharp_ok']:.0f}%）。这是运动模糊 —— 转太快了。"
                       "标定只需要 min_rotation_std_rad_s=0.05 rad/s ≈ 3°/s 的多轴激励，"
                       "慢下来完全够；糊掉的帧 PnP 残差会爆到 3~4 px 被剔，"
                       "在窗口里留出 >0.35 s 的空洞，整份数据作废")
    elif q is not None and q["effective_rate"] < 0.6:
        actions.append(f"★ 标定程序只会用到 {100*q['effective_rate']:.0f}% 的帧"
                       "（门槛是连续的 35 帧以上）。先解决检出率，再谈标定")

    # 分清【屏幕暗】和【房间暗】—— 2026-10-16 踩到的：工具一直喊"把 iPad 亮度拉满"，
    # 而用户早就拉满了，真正的原因是 iOS 自动亮度在暗环境下把屏幕压暗了 5.9 倍。
    if b_level == "dark" and d.get("board_white") is not None and d["board_white"] < 100:
        actions.append(f"★ 标定板太暗：白格只有 {d['board_white']:.0f}/255（要 150 以上）。"
                       "先分清是哪种板子：\n"
                       "     【打印板】它是反光的，比自发光屏幕暗 5~8 倍，"
                       "和 iPad 用同一套曝光设置必然全黑。按顺序做：\n"
                       "       ① 开房间灯，最好再加一盏台灯/补光灯【正对板子】照"
                       "（这是最干净的办法，不要靠加 gain）\n"
                       "       ② capture_config.py 的 EXPOSURE 从 100 往上调："
                       "200 → 250 → 300（30fps 下 330 是上限）\n"
                       "       ③ 还不够才加 GAIN：0 → 20 → 40（加 gain 会引入噪声）\n"
                       "       ④ 或者把 FPS 降到 20，曝光预算从 33ms 涨到 50ms\n"
                       "     【iPad 屏】检查自动亮度/自动锁定/低电量/原彩/贴膜")
    elif b_level == "dark":
        actions.append("开房间灯。还不行就把 capture_config.py 的 EXPOSURE 从 100 往上调"
                       "（150 → 200 → 250），GAIN 也可以从 0 加到 20~50")

    cut = bool(d.get("cut_off_frames"))
    small = bool(d.get("square_px") and d["square_px"] < 55)
    if d["rate"] >= 0.6 and not blur and not cut and not small:
        # 检出率才是主判据。亮度只是"为什么检出率低"的一种解释 ——
        # 检出率够高就说明亮度没挡路，不要因为它把人拦下来。
        # （2026-10-15 实际踩到：p90=43 被判 dark，但检出 10/12=83%，其实完全可用。）
        if b_level not in ("ok", "unknown"):
            actions.append(f"亮度备注：{b_text} —— 但检出率已经过关，不挡路。"
                           "真机标定时要转动、角度更斜，光更足会更稳；顺手开个灯没坏处")
        return "ok", actions

    if (d["rate"] >= 0.6 and not cut) and blur:
        # 站着不动时清晰度没问题，一转动就糊 —— 静态取景体检看不出来，
        # 必须在真机采集（record_session.py 会跑这段）时才能发现。所以给 warn。
        return "warn", actions

    if small or cut:
        if small or d["rate"] >= 0.6:
            # 板子太小会让姿态解退化 → 相机角速度和陀螺差几倍 → 标定必崩。
            # 切边则会打断"连续 35 帧"。两条都不能判 ok。
            return "warn", actions

    if d["detected"] == 0:
        sub_hit = any(v > 0 for v in d["subgrid"].values())
        if sub_hit:
            actions.append("★ 板子被画面切掉了（小网格能检出、完整网格不能）。"
                           "把相机往后退 / 抬高，让整块板子连同四周留白都进画面 —— "
                           "板子占画面 1/3 ~ 1/2 就够，占到 2/3 以上必然切边")
        else:
            actions.append("完整网格和小网格都检不出。按可能性排：")
            actions.append("   a) 板子根本没对着镜头（先确认画面里能看到棋盘）")
            actions.append("   b) 标定板行列数不对 —— 数一下实际图案是几个格子，"
                           "内角点 = 格子数 − 1")
            actions.append("   c) 太暗 / 运动模糊 / 角度太斜")
        return "fail", actions

    actions.append(f"检出率只有 {100*d['rate']:.0f}%，要 >60% 才能跑标定。"
                   "多半是板子贴边、光不均、或者转动太快糊了")
    return "fail", actions
