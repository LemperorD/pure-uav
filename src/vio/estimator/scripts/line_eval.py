#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直线段评估：走一条直线，量"会不会偏"、偏多少、偏在哪。

为什么单独做这个脚本（2026-10-25）：
  闭环数据只能给出"回到起点差多少"（dev2 3.8%、long4 1.9%），但它分不清三件事：
      ① 尺度错（走 8 m 记成 8.3 m）
      ② 航向漂（方向越走越偏 → 轨迹变成弧线）
      ③ 随机抖动（抖动不累积，闭环误差却一样难看）
  这三件对 RoboMaster 的意义完全不同：**②才是"开炮打不中"的直接原因**
  （航向差 1° 在 5 m 处 = 8.7 cm）。所以需要一条已知长度、已知是直线的轨迹。

判据（不需要真值以外的任何东西）：
  · 尺度    = 轨迹路径长度 / 卷尺量的真实长度     （目标 |误差| < 2%）
  · 横偏    = 各点到 PCA 主轴的垂直距离 RMS       （目标 < 1.5% 里程）
  · 航向漂  = 速度方向相对主轴的累计偏离（°/m）   （目标 < 0.15°/m）
  · 姿态一致性 = 相机 yaw 的变化量 vs 轨迹航向变化量（两者物理上必须相等）

用法：
  python3 scripts/line_eval.py --tum out/line1/traj_safe.tum --length 8.0
  python3 scripts/line_eval.py --tum out/line1/traj_safe.tum --length 8.0 --json
"""
import argparse
import json
import math
import sys

import numpy as np


def load_tum(path):
    """TUM: timestamp tx ty tz qx qy qz qw（也接受带表头的 pose.csv 风格由调用方转换）"""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.replace(',', ' ').split()
            if len(p) < 8:
                continue
            try:
                rows.append([float(x) for x in p[:8]])
            except ValueError:
                continue  # 表头
    d = np.array(rows)
    if len(d) < 10:
        sys.exit("轨迹点太少（%d），检查 --tum 路径" % len(d))
    return d


def yaw_deg(qx, qy, qz, qw):
    return np.degrees(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))


def unwrap_deg(a):
    return np.degrees(np.unwrap(np.radians(a)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tum', required=True)
    ap.add_argument('--length', type=float, default=0.0, help='卷尺量的真实直线距离（m），给了才算尺度')
    ap.add_argument('--min-speed', type=float, default=0.08, help='判定"在走"的速度门限 m/s')
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--pauses', action='store_true',
                    help='两点停顿法：起点站住 1 s、走到终点再站住 1 s，用两次停顿的位移算尺度（不受摇摆/绕路影响）')
    ap.add_argument('--still-speed', type=float, default=0.15, help='判"站着"的速度门限 m/s（手持实测 0.02~0.10）')
    ap.add_argument('--still-time', type=float, default=0.5, help='判"站着"的最短持续时间 s')
    a = ap.parse_args()

    d = load_tum(a.tum)
    t = d[:, 0]
    t = t - t[0]
    xyz = d[:, 1:4]
    yaw = unwrap_deg(yaw_deg(d[:, 4], d[:, 5], d[:, 6], d[:, 7]))

    # 水平面：LOCAL_FRD 的 x-y 是水平面，z 向下（高度单独看）
    xy = xyz[:, :2]
    dt = np.diff(t)
    dt[dt <= 0] = 1e-3
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    speed = step / dt
    # 【2026-10-25】逐帧差分的速度噪声很大：人站着不动时逐帧速度也会跳到 0.1~0.2 m/s，
    # 用它判"站着"会漏。改成 0.2 s 窗口的位移/时间，静止时能稳定落到 0.05 以下。
    sp_win = []
    for i in range(len(t)):
        j = int(np.searchsorted(t, t[i] + 0.2))
        j = min(j, len(t) - 1)
        sp_win.append(np.linalg.norm(xy[j] - xy[i]) / max(t[j] - t[i], 1e-3))
    speed = np.array(sp_win)

    # 只取"在走"的部分，静止段会把主轴拟合带偏
    moving = speed > a.min_speed
    if moving.sum() < 10:
        sys.exit("几乎没检测到运动（最大速度 %.2f m/s）—— 门限 --min-speed 调小试试" % speed.max())

    #=====================================================================
    # 【2026-10-25】两点停顿法
    # 为什么比"路径长度"可靠：手在走的过程里会左右摆、会轻微绕，路径长度一定 > 真实位移；
    # 而**两次停顿之间的位移**就是真实走过的那段直线距离，与摇摆、绕行、航向漂都无关
    # （前提是这 10 m 之内航向漂小到可以忽略，实测 57 m 才 0.15°，10 m 可以无视）。
    #=====================================================================
    if a.pauses:
        still = speed < a.still_speed
        # 找连续的静止段
        runs = []
        i = 0
        n = len(still)
        while i < n:
            if still[i]:
                j = i
                while j < n and still[j]:
                    j += 1
                if t[j - 1] - t[i] >= a.still_time:
                    runs.append((i, j - 1))
                i = j
            else:
                i += 1
        # 【2026-10-25】常见情况：起点站着不动的那 1~2 秒 **VIO 还没初始化**，
        # 所以轨迹里根本没有这一段（轨迹从 init 完成那一刻才开始）。
        # 此时只要第一个位姿本身是"慢"的，就把它当成 A 点，给出警示。
        start_slow = speed[0] < a.still_speed
        if not runs:
            sys.exit("找不到静止段（起点速度 %.2f m/s）。请在起点多站 3~5 秒，"
                     "**等屏幕上有位姿再开始走**，到终点再站 3 秒。" % speed[0])
        if not runs[0][0] == 0:
            if not start_slow:
                sys.exit("起点速度 %.2f m/s = 一开录就在走。请在起点多站 3~5 秒，"
                         "**等屏幕上有位姿再开始走**。" % speed[0])
            runs = [(0, 0)] + runs
            print("[提示] 起点没有静止段 —— 站着的那 1~2 秒还没初始化、不在轨迹里。"
                  "改用第一个位姿当 A 点（A 点未必正好在标记上，结论会偏大）。\n")
        # 取"最长的一动"两侧的静止段：即运动量最大的那一对
        best = None
        for u in range(len(runs) - 1):
            a0 = runs[u][1]
            a1 = runs[u + 1][0]
            if a1 - a0 < 3:
                continue
            moved = np.linalg.norm(xy[a1] - xy[a0])
            if best is None or moved > best[0]:
                best = (moved, a0, a1, u)
        if best is None:
            sys.exit("静止段之间没有连续运动")
        _, ia, ib, u = best
        p0 = xyz[runs[u][0]:runs[u][1] + 1].mean(0)
        p1 = xyz[runs[u + 1][0]:runs[u + 1][1] + 1].mean(0)
        dxyz = p1 - p0
        d_h = math.hypot(dxyz[0], dxyz[1])
        d_3d = float(np.linalg.norm(dxyz))
        seg_len = float(step[ia:ib].sum())
        out = {
            'pause_a_n': runs[u][0], 'pause_b_n': runs[u + 1][0],
            'pause_a_s': round(float(t[runs[u][0]]), 2),
            'pause_b_s': round(float(t[runs[u + 1][0]]), 2),
            'displacement_h_m': round(d_h, 4),
            'displacement_3d_m': round(d_3d, 4),
            'path_between_m': round(seg_len, 4),
            'sway_ratio': round(seg_len / max(d_h, 1e-9), 4),
            'dz_m': round(float(dxyz[2]), 4),
            'moving_time_s': round(float(t[ib] - t[ia]), 2),
            'v_mean': round(float(seg_len / max(t[ib] - t[ia], 1e-6)), 3),
        }
        if a.length > 0:
            out['scale_error_pct'] = round(100.0 * (d_h / a.length - 1.0), 2)
            out['scale_error_path_pct'] = round(100.0 * (seg_len / a.length - 1.0), 2)
        if a.json:
            print(json.dumps(out, ensure_ascii=False, indent=2))
            return
        print("停顿 A: t=%.2fs（第 %d 帧）   停顿 B: t=%.2fs（第 %d 帧）  中间走了 %.2f s，平均 %.2f m/s" %
              (out['pause_a_s'], out['pause_a_n'], out['pause_b_s'], out['pause_b_n'],
               out['moving_time_s'], out['v_mean']))
        print()
        print("  两次停顿的位移   %.3f m（水平） / %.3f m（三维），高度差 %+.3f m" %
              (out['displacement_h_m'], out['displacement_3d_m'], out['dz_m']))
        print("  中间路径长度     %.3f m   摇摆比 %.3f（1.000 = 完全直着走）" %
              (out['path_between_m'], out['sway_ratio']))
        if a.length > 0:
            print()
            print("  真实距离         %.3f m" % a.length)
            print("  >>> 尺度误差   %+.2f%%   （用位移算，推荐）" % out['scale_error_pct'])
            print("      参考        %+.2f%%   （用路径长度算，会偏高）" % out['scale_error_path_pct'])
        return

    path_len = step.sum()
    idx = np.where(moving)[0]
    seg = xy[idx[0]:idx[-1] + 1]

    # PCA 主轴 = 最拟合直线的方向
    c = seg - seg.mean(0)
    w, V = np.linalg.eigh(c.T @ c)
    axis = V[:, -1]
    perp = np.array([-axis[1], axis[0]])
    lat = c @ perp
    lon = c @ axis
    if lon[-1] < lon[0]:          # 统一成从负到正
        lat = -lat
        lon = -lon

    # 【2026-10-25 修正】航向漂移不能用"逐步方向"累加：中间出现几步反向（转身/回身）
    # 会让角度翻 180°，unwrap 之后累加成 176° 这种假数字（实测 line2：人只转了 0.73°）。
    # 正确做法：把轨迹投影到主轴上，看"横向偏移 vs 沿轴里程"的斜率 —— 斜率就是系统航向漂。
    ds = np.linalg.norm(np.diff(seg, axis=0), axis=1)
    s_cum = np.cumsum(ds)
    L = lon - lon[0]                      # 沿轴里程（m）
    Y = lat - lat[0]                      # 横向偏移（m）
    # 取中间 5%~95% 区段做线性拟合，避开起步/收尾的抖动
    k0, k1 = int(0.05 * len(L)), int(0.95 * len(L))
    if k1 - k0 > 10:
        A = np.vstack([L[k0:k1], np.ones(k1 - k0)]).T
        slope, _ = np.linalg.lstsq(A, Y[k0:k1], rcond=None)[0]
    else:
        slope = 0.0
    heading_rel = np.array([np.degrees(math.atan(slope))])   # 系统航向漂（全程，度）
    travel = max(L[k1 - 1] - L[k0], 1e-6)
    yaw_rate = heading_rel[0] / travel

    # 姿态一致性：相机 yaw 变化量（同样按运动段取）应该等于轨迹航向变化量
    yaw_move = yaw[idx[-1]] - yaw[idx[0]]
    path_heading_change = heading_rel[0]

    out = {
        'frames': int(len(t)),
        'path_len_m': round(float(path_len), 3),
        'moving_len_m': round(float(s_cum[-1]), 3),
        'straight_dist_m': round(float(np.linalg.norm(seg[-1] - seg[0])), 3),
        'straightness': round(float(np.linalg.norm(seg[-1] - seg[0]) / max(path_len, 1e-9)), 4),
        'lateral_rms_m': round(float(np.sqrt((lat ** 2).mean())), 4),
        'lateral_max_m': round(float(np.abs(lat).max()), 4),
        'lateral_rms_pct': round(float(100.0 * np.sqrt((lat ** 2).mean()) / max(s_cum[-1], 1e-9)), 3),
        'heading_drift_deg': round(float(heading_rel[0]), 3),
        'heading_drift_deg_per_m': round(float(yaw_rate), 4),
        'yaw_change_cam_deg': round(float(yaw_move), 3),
        'yaw_change_path_deg': round(float(path_heading_change), 3),
        'yaw_consistency_deg': round(float(yaw_move - path_heading_change), 3),
        'alt_change_m': round(float(xyz[idx[-1], 2] - xyz[idx[0], 2]), 3),
        'v_mean': round(float(speed[moving].mean()), 3),
        'v_max': round(float(speed.max()), 3),
    }
    if a.length > 0:
        out['scale_error_pct'] = round(100.0 * (s_cum[-1] / a.length - 1.0), 3)

    if a.json:
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return

    print("轨迹 %d 帧，运动段 %.2f m（全程 %.2f m），平均速度 %.2f m/s" %
          (out['frames'], out['moving_len_m'], out['path_len_m'], out['v_mean']))
    print()
    print("  直线度        %.3f      （1.000 = 完全直；<0.99 说明明显拐了）" % out['straightness'])
    print("  横偏 RMS      %.3f m    = 里程的 %.2f%%   （目标 <1.5%%）" %
          (out['lateral_rms_m'], out['lateral_rms_pct']))
    print("  横偏 最大     %.3f m" % out['lateral_max_m'])
    print("  航向漂移      %+.2f°  = %+.3f°/m        （目标 <0.15°/m）" %
          (out['heading_drift_deg'], out['heading_drift_deg_per_m']))
    print("  姿态一致性    相机 yaw %+.2f° vs 轨迹航向 %+.2f° → 差 %+.2f°" %
          (out['yaw_change_cam_deg'], out['yaw_change_path_deg'], out['yaw_consistency_deg']))
    print("  高度变化      %+.3f m" % out['alt_change_m'])
    if 'scale_error_pct' in out:
        print("  尺度误差      %+.2f%%   （轨迹 %.2f m / 真值 %.2f m）" %
              (out['scale_error_pct'], out['moving_len_m'], a.length))
    print()
    # 一句话结论
    bad = []
    if out['lateral_rms_pct'] > 1.5:
        bad.append("横偏 %.2f%% 偏大" % out['lateral_rms_pct'])
    if abs(out['heading_drift_deg_per_m']) > 0.15 and abs(out['heading_drift_deg']) > 0.5:
        bad.append("航向漂 %+.2f°（%.3f°/m）" % (out['heading_drift_deg'], out['heading_drift_deg_per_m']))
    if 'scale_error_pct' in out and abs(out['scale_error_pct']) > 2.0:
        bad.append("尺度 %+.1f%%" % out['scale_error_pct'])
    if abs(out['yaw_consistency_deg']) > 3.0:
        bad.append("姿态与航向不自洽 %.1f°" % out['yaw_consistency_deg'])
    print("结论：" + ("、".join(bad) if bad else "直线段正常（横偏/航向/尺度都在门限内）"))


if __name__ == '__main__':
    main()
