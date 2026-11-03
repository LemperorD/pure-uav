#!/usr/bin/env python3
"""按"初始化段"评估一段 VIO 轨迹的精度（需要真值约束，不需要额外设备）。

为什么必须按段：LOST 之后重建会**重锚定**，整条轨迹是若干段独立滤波结果拼起来的。
把整条轨迹的"终点减起点"当精度，会把重锚定和丢输出混进去 —— 那个数没有意义。

真值从哪来（二选一，成本都是零）：
  --loop                 原地闭环：从地面标记出发，绕一圈走回**同一个标记**。
                         真实起终点位移 = 0，所以每段的 |终点 − 起点| 就是该段的漂移。
  --scale-dist 5.0       已知直线距离（米）：量出 5 米走过去，
                         段内路径长 / 5.0 = 尺度比（理想 1.0）。

用法：
  python3 scripts/eval_accuracy.py --log out/rec1/console.log \\
      --tum out/rec1/traj_safe.tum --loop
  python3 scripts/eval_accuracy.py --log ... --tum ... --scale-dist 5.0

  --tum 建议用 traj_safe.tum（只含健康门限通过的位姿），和实际输出一致。
  --log 用来切段（解析 "[VIO] [segment N] 初始化完成：init_time=X s"）。
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np

SEG_RE = re.compile(r"\[segment (\d+)\] 初始化完成：init_time=([\d.]+) s")


def read_tum(path: Path):
    rows = []
    for line in path.read_text(errors="ignore").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        v = line.split()
        if len(v) < 4:
            continue
        try:
            rows.append([float(x) for x in v[:4]])
        except ValueError:
            continue
    if not rows:
        return np.zeros((0, 4))
    return np.asarray(rows)


def parse_segments(log: Path):
    """返回 [(segment_id, init_time_s)]，按时间排序。"""
    segs = []
    if log.exists():
        for line in log.read_text(errors="ignore").splitlines():
            m = SEG_RE.search(line)
            if m:
                segs.append((int(m.group(1)), float(m.group(2))))
    segs.sort(key=lambda r: r[1])
    return segs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", type=Path, help="实时/离线控制台日志（切段用）")
    ap.add_argument("--tum", required=True, type=Path)
    ap.add_argument("--loop", action="store_true", help="这是闭环行走（真实起终点距离=0）")
    ap.add_argument("--scale-dist", type=float, default=0.0, help="已知直线距离（米）")
    ap.add_argument("--max-gap", type=float, default=0.5, help="段内允许的最大帧间隔（秒）")
    args = ap.parse_args()

    xyz = read_tum(args.tum)
    if len(xyz) < 10:
        print(f"[错误] {args.tum} 里只有 {len(xyz)} 个位姿，没法评估")
        return 2

    segs = parse_segments(args.log) if args.log else []
    t_all = xyz[:, 0]

    # 没有日志就退化成"按时间大跳变切段"
    if not segs:
        bounds = [t_all[0]]
        for i in range(1, len(t_all)):
            if t_all[i] - t_all[i - 1] > args.max_gap:
                bounds.append(t_all[i])
        bounds.append(t_all[-1] + 1e-9)
        segs = [(i, bounds[i]) for i in range(len(bounds) - 1)]
        segs = [(i, t) for i, t in segs if t < t_all[-1]]
        print("[提示] 没有 --log，退化按时间跳变切段，结果仅供粗看")

    print(f"轨迹文件 : {args.tum}（{len(xyz)} 个位姿，跨度 {t_all[-1]-t_all[0]:.1f} s）")
    print(f"初始化段 : {len(segs)} 段")
    print()

    hdr = f"{'段':>3} {'段长':>8} {'点数':>6} {'路径长':>8} {'|终-起|':>8}"
    if args.loop:
        hdr += f" {'闭环误差':>9}"
    if args.scale_dist > 0:
        hdr += f" {'尺度比':>8}"
    print(hdr)
    print("-" * len(hdr))

    rows = []
    for i, (sid, t0) in enumerate(segs):
        t1 = segs[i + 1][1] if i + 1 < len(segs) else t_all[-1] + 1e-9
        m = (t_all >= t0) & (t_all < t1)
        p = xyz[m][:, 1:4]
        tt = xyz[m][:, 0]
        if len(p) < 5:
            continue
        dt = np.diff(tt)
        path = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
        disp = p[-1] - p[0]
        line = (f"{i:>3} {tt[-1]-tt[0]:>7.1f}s {len(p):>6} {path:>8.2f} "
                f"{np.linalg.norm(disp):>8.2f}")
        if args.loop:
            line += f" {np.linalg.norm(disp):>9.2f}"
        if args.scale_dist > 0:
            # 尺度比用"净位移"而不是路径长（路径长含抖动/来回）
            line += f" {np.linalg.norm(disp)/args.scale_dist:>8.2f}"
        print(line)
        rows.append(dict(sid=sid, dur=tt[-1] - tt[0], n=len(p), path=path,
                         disp=float(np.linalg.norm(disp))))

    if not rows:
        print("[错误] 没有一段点够多，无法评估")
        return 2

    print()
    dur = np.array([r["dur"] for r in rows])
    dis = np.array([r["disp"] for r in rows])
    longest = int(np.argmax(dur))
    print(f"总可用时长 : {dur.sum():.1f} s")
    print(f"最长段     : 第 {rows[longest]['sid']} 段，{dur[longest]:.1f} s")

    # 【2026-10-22】死也要检查"路径是不是合理"：
    # 实测存在"位置状态冻结"的退化 —— 姿态在变、更新也在跑，但位置恒为常数，
    # 这时闭环误差 = 0.00 m，看起来完美，实际一步没走。
    allp = xyz[:, 1:4]
    total_path = float(np.sum(np.linalg.norm(np.diff(allp, axis=0), axis=1)))
    print(f"全轨迹路径 : {total_path:.2f} m")
    if total_path < 0.3:
        print("  ⛔ 路径几乎为 0 → **滤波器位置冻结**，闭环误差没有意义（别当成绩）")
    elif total_path > 50.0:
        print("  ⚠️ 路径异常长（手持行走 1 分钟通常 10–30 m）→ 疑似尺度爆炸")

    if args.loop:
        # 闭环误差只看"够长的段"，短段的起终点本来就可能不重合（还没走回去）
        good = dur >= 10.0
        if good.any():
            print(f"闭环误差   : 中位 {np.median(dis[good]):.2f} m，最长段 {dis[longest]:.2f} m")
            print(f"             （{good.sum()} 段 ≥10 s；这一项越小越好，理想 0）")
            print(f"每段漂移率 : 中位 {100*np.median(dis[good]/dur[good]):.2f} % 的段长")
        else:
            print("闭环误差   : 没有 ≥10 s 的段，无法评估（这次跑得太碎）")
    if args.scale_dist > 0:
        good = dur >= 3.0
        if good.any():
            print(f"尺度       : 中位 {np.median(dis[good])/args.scale_dist:.3f}"
                  f"（理想 1.000，偏差 {(np.median(dis[good])/args.scale_dist-1)*100:+.1f}%）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
