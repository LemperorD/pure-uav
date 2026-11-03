"""看一眼采集到的图：亮度够不够、板子认不认得出、是不是出画了。

被 `framing_check.py`（取景体检）和 `record_session.py`（采完自动体检）共用。

为什么需要它：预览窗为了帧率被关掉了，于是采集变成"盲拍"。这个模块就是那双眼睛 ——
采完立刻把最关键的三件事说出来，不用等标定跑几分钟才发现板子根本没进画面。
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent


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

def _load_target(target_yaml, min_tags=None):
    """用标定程序自己的 Target —— 体检和标定走同一套检测，不会各说各话。

    min_tags 不传时去 extrinsics_config 读，保证"体检说能过"和"标定真会用"
    的下限是同一个数（AprilGrid 默认 8，别在这里写死 4）。
    """
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    if min_tags is None:
        try:
            import extrinsics_config as ec
            min_tags = int(getattr(ec.SETTINGS, "min_tags", 4))
        except Exception:                                    # noqa: BLE001
            min_tags = 4
    from calib.data import Target, read_yaml
    return Target(read_yaml(target_yaml), min_tags=min_tags)


def _detect(gray, target):
    """检测一帧。返回 (uv, 标签, meta)；uv 为 None 表示这帧不能用。

    棋盘格和 AprilGrid 都走 calib/data.py 的 Target.detect：
    棋盘是 SB/经典角点，AprilGrid 是 aruco 36h11。meta 额外带 tag 个数和 tag 边长。
    """
    meta = {"n_tags": None, "tag_px": None, "error": None}
    try:
        result = target.detect(gray)
    except Exception as exc:                                 # noqa: BLE001
        # 典型情况：画面里混进了别的 AprilTag → 重复 ID，标定程序也会直接抛异常
        meta["error"] = str(exc)
        return None, None, meta
    if result is None:
        return None, None, meta
    uv = np.asarray(result[1], dtype=float).reshape(-1, 2)
    if target.kind in ("checkerboard", "chessboard"):
        return uv, "Target", meta
    quads = uv.reshape(-1, 4, 2)                             # 每个 tag 固定 4 个角点
    edges = np.linalg.norm(quads[:, 1] - quads[:, 0], axis=1)
    meta["n_tags"] = int(len(quads))
    meta["tag_px"] = float(np.median(edges))
    return uv, "aprilgrid", meta


def _probe_chess(gray, cols, rows):
    """只给棋盘格的【切边探针】用：换一个小一号的内角点网格再试一次。"""
    try:
        ok = cv2.findChessboardCornersSBWithMeta(
            gray, (cols, rows), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)[0]
    except Exception:                                        # noqa: BLE001
        ok = False
    if not ok:
        ok = cv2.findChessboardCorners(gray, (cols, rows))[0]
    return bool(ok)


def detect_stats(paths, target_yaml, sample=12, probe_subgrids=True):
    """返回检出率 + 板子在画面里"有多大"。

    棋盘格和 AprilGrid 共用同一个 Target.detect，差别只在"大小"怎么算：
      · 棋盘：每格边长（面积法，与板子转了 90° 无关），太小会让平面 PnP 退化
      · AprilGrid：单个 tag 的边长，太小 aruco 解不出 36h11 的编码
    AprilGrid 只看到一部分 tag 也能用，所以【不做】切边探针 ——
    那条探针只对"整块板子必须全在画面里"的棋盘格有意义。
    """
    target = _load_target(target_yaml)
    checker = target.kind in ("checkerboard", "chessboard")
    frames = [(p, _gray(p)) for p in _pick(paths, sample)]
    frames = [(p, g) for p, g in frames if g is not None]

    tried = hit = 0
    kinds, margins, spans, spans_v, board_white = set(), [], [], [], []
    shape = None
    sharp_hit, sharp_miss = [], []
    tag_px_all, n_tags_all, errors = [], [], []
    misses = []
    for p, g in frames:
        shape = g.shape
        tried += 1
        uv, label, meta = _detect(g, target)
        if meta["error"]:
            errors.append(meta["error"])
        sharp = float(cv2.Laplacian(g, cv2.CV_64F).var())
        if uv is None or len(uv) < 4:
            sharp_miss.append(sharp)
            misses.append(g)
            continue
        hit += 1
        kinds.add(label)
        margins.append(_edge_margin(uv, g.shape))
        sharp_hit.append(sharp)
        spans.append(float(uv[:, 0].max() - uv[:, 0].min()))
        spans_v.append(float(uv[:, 1].max() - uv[:, 1].min()))
        if meta["tag_px"]:
            tag_px_all.append(meta["tag_px"])
            n_tags_all.append(meta["n_tags"])
        # 板子区域内的"白底"亮度 —— 用来分清【板子太暗】和【房间太暗】
        x0, x1 = int(uv[:, 0].min()), int(uv[:, 0].max())
        y0, y1 = int(uv[:, 1].min()), int(uv[:, 1].max())
        roi = g[max(0, y0):y1, max(0, x0):x1]
        if roi.size > 500:
            board_white.append(float(np.percentile(roi, 90)))

    rate = hit / tried if tried else 0.0
    med = lambda a: float(np.median(a)) if a else None            # noqa: E731
    out = {"tried": tried, "detected": hit, "rate": rate,
           "kind": target.kind, "checker": checker,
           "cols": int(target.cols), "rows": int(target.rows),
           "min_tags": None if checker else int(target.min_tags),
           "kinds": sorted(kinds),
           "min_edge_margin_px": min(margins) if margins else None,
           "board_width_px": med(spans), "board_height_px": med(spans_v),
           "n_tags": med(n_tags_all), "tag_px": med(tag_px_all),
           "board_white": med(board_white),
           "shape": shape, "subgrid": {}, "cut_off_frames": 0,
           "sharp_median": med(sharp_hit + sharp_miss),
           "sharp_hit": med(sharp_hit), "sharp_miss": med(sharp_miss),
           "errors": errors[:3]}

    # "最小可辨识单元"在画面里有多大 —— 判定该不该靠近就看这个
    if checker:
        # 面积法：(cols+1)x(rows+1) 个格子铺满 bbox，与板子在画面里转了 90° 无关
        out["cell_px"] = (float(np.sqrt(med(spans) * med(spans_v)
                                       / ((target.cols + 1) * (target.rows + 1))))
                          if (med(spans) and med(spans_v)) else None)
        out["cell_name"] = "每个格子"
        out["cell_target_px"] = 90
    else:
        out["cell_px"] = med(tag_px_all)
        out["cell_name"] = "每个 tag 边长"
        out["cell_target_px"] = 45

    # 【切边探针】只对棋盘格有意义：整块必须全在画面里，缺一个角点就整帧丢。
    # 小网格能检出、完整网格不能 ⟹ 只有一部分板子在画面里 —— 板子被画面切掉了。
    # 这是 2026-10-15 那次失败的真凶：转动时棋盘漂到画面边缘，
    # 连丢 15 帧 = 0.69 s 盲区，直接触发 max_camera_gap_s=0.35。
    if checker and probe_subgrids and misses:
        for dc, dr in ((1, 1), (2, 2)):
            c2, r2 = target.cols - dc, target.rows - dr
            if min(c2, r2) < 4:
                continue
            sub = sum(1 for g in misses[:8] if _probe_chess(g, c2, r2))
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
                  gap_s=0.35, min_frames=35, min_duration_s=5.0, min_tags=4, log=None):
    """对【每一帧】跑标定程序自己的检测 + PnP + 那道 PnP 门槛，
    算出它真正会接受哪些帧、以及由此产生的**空洞结构**。

    这是唯一能提前预测 `max_camera_gap_s` 会不会触发的方法 ——
    采样 12 张的"检出率"看不出来：86% 检出率完全可能被 4 处 0.5~1.9 秒的
    盲区毁掉（2026-10-15 就是这样白跑了三次标定，每次 5 分钟）。
    """
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    from calib.data import Camera, Target, make_observation, read_yaml

    cam = Camera.load(camera_yaml)
    tgt = Target(read_yaml(target_yaml), min_tags=min_tags)
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


def analyze(paths, target_yaml, sample=12):
    paths = list(paths)
    b = brightness_stats(paths, min(8, sample))
    d = detect_stats(paths, target_yaml, sample=sample)
    unit = "内角点" if d["checker"] else "tag"
    return {"images": len(paths), "brightness": b, "detection": d,
            "board": f"{d['cols']}x{d['rows']} {unit}", "quality": None}


def calib_quality_stats(paths, camera_yaml, target_yaml, window=24,
                        min_tags=4, max_pnp_rms=2.0):
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
    tgt = Target(read_yaml(target_yaml), min_tags=min_tags)
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
        if obs is None or obs.pnp_rms > max_pnp_rms:
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
            "pnp_bad_median": med(rms_bad), "pnp_gate": float(max_pnp_rms),
            "sharp_ok": med(sharp_ok), "sharp_bad": med(sharp_bad)}


def report(res, log=print):
    """把分析结果打印成人话 + 可执行的动作。返回 (level, [动作])。"""
    b, d = res["brightness"], res["detection"]
    b_level, b_text = brightness_verdict(b)
    checker = d.get("checker", True)
    actions = []

    log(f"  图片张数        {res['images']}")
    if b:
        log(f"  亮度            p50={b['p50']:.0f}  p90={b['p90']:.0f}  p99={b['p99']:.0f}"
            f"  均值={b['mean']:.1f}")
        log(f"                  {b_text}")
    log(f"  检出 ({res['board']})     {d['detected']}/{d['tried']}"
        f"  ({100*d['rate']:.0f}%)" + (f"  [{','.join(d['kinds'])}]" if d["kinds"] else ""))
    if d.get("errors"):
        log(f"  检测报错        {d['errors'][0]}")
    if d["min_edge_margin_px"] is not None:
        log(f"  板子离画面边缘   {d['min_edge_margin_px']:.0f} px")
    if d.get("board_white") is not None:
        log(f"  标定板白底亮度   {d['board_white']:.0f}/255"
            + ("   ← 板子本身太暗" if d["board_white"] < 100 else ""))
    if d.get("n_tags") is not None:
        log(f"  每帧看到 tag     {d['n_tags']:.0f} 个（标定下限 {d['min_tags']} 个）")
    if d.get("cell_px"):
        tip = ("太小会让姿态解退化，目标 ~90 px" if checker
               else "目标 ≥45 px（36h11 的码元才解得出来）")
        log(f"  板子在画面里     {d['board_width_px']:.0f} × {d['board_height_px']:.0f} px，"
            f"{d['cell_name']} {d['cell_px']:.0f} px  ← {tip}")
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
    # AprilGrid 换成看 tag 边长：36h11 的 tag 有 6x6 个码元，tag 30 px 时每码元才 4 px。
    small_limit = 55 if checker else 40
    if d.get("cell_px") and d["cell_px"] < small_limit:
        if checker:
            actions.append(f"★ 板子在画面里太小了（每格只有 {d['cell_px']:.0f} px）。"
                           "这会让 PnP 姿态解退化：画面几乎没动，姿态却能跳 1° 以上，"
                           "于是'相机轨迹推出来的角速度'和陀螺差好几倍，标定必然失败。"
                           "【靠近】到每格约 90 px（参考：内参成功那次板宽 742 px / 每格 93 px）")
        else:
            actions.append(f"★ 每个 tag 在画面里只有 {d['cell_px']:.0f} px。36h11 要解出 ID，"
                           "tag 边长至少 ~30 px、舒服要 45 px 以上；太小就等于这帧白给。"
                           "【靠近】板子，或者换 tagSize 更大的板（tagSize 减半，距离就得减半）")

    q = res.get("quality")
    if q is not None:
        log(f"  标定程序实际会用   {q['accepted']}/{q['window']} 帧"
            f"（检出 {q['detected']}，检出但 PnP>{q.get('pnp_gate', 2.0):g}px 被剔 {q['rejected']}）")
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
    small = bool(d.get("cell_px") and d["cell_px"] < small_limit)
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
        if d.get("errors"):
            actions.append("★ 检测器直接报错 —— 多半是画面里混进了别的 AprilTag（重复 ID），"
                           "标定程序遇到这个会直接崩。把桌面上其他 AprilTag / 二维码收走再跑。")
        elif checker:
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
        else:
            actions.append(f"一个 tag 都没解出来（下限 {d['min_tags']} 个）。按可能性排：")
            actions.append("   a) 板子不在画面里 —— 先确认相机确实对着 AprilGrid")
            actions.append("   b) tag 太小 —— 靠近板子，或换 tagSize 更大的板。"
                           "36h11 一个 tag 是 8x8 个码元，tag 只有 30 px 时每码元才 3.8 px，解不出")
            actions.append("   c) 糊了 / 太暗 —— 打印板比屏幕暗 5~8 倍，先开灯、再提 EXPOSURE")
            actions.append("   d) 板上 tag 的黑框外沿被裁掉了 —— AprilGrid 判角用的是黑框外沿，"
                           "打印/裁剪时别把外沿切掉")
            actions.append("   e) tag 族不是 36h11，或者某张 tag 贴反/转了 90°")
            actions.append("   f) configs/aprilgrid.yaml 的 tagRows/tagCols 是【tag 个数】，"
                           "不是内角点；填错一样 0 检出")
        return "fail", actions

    if checker:
        actions.append(f"检出率只有 {100*d['rate']:.0f}%，要 >60% 才能跑标定。"
                       "多半是板子贴边、光不均、或者转动太快糊了")
    else:
        actions.append(f"检出率只有 {100*d['rate']:.0f}%，要 >60% 才能跑标定。"
                       "多半是 tag 太小/太远、光不均、或者转动太快糊了")
    return "fail", actions
