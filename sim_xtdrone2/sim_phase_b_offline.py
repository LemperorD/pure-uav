#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""制导闭环离线仿真 —— 无人机 / 弹丸 拦截交会（不需要 ROS / Gazebo / PX4）

三种模式：
  · cruise（阶段 B 用）：无人机从悬停起步 → 朝固定方向预加速 → 到速度后发射弹丸 → 交班 PPN
  · hot                ：弹丸先出现，无人机朝弹丸助推（**最坏起步**，用于对比）
  · pilot ★本阶段用    ：**无人机已经在以巡航速度飞行**（速度/方向给定）
                         → 弹丸出现 → **立即开始制导**
                           （模拟"决策层已判定：就在这个时刻开始拦截"）

模型：
  · 无人机：质点 + **一阶加速度响应**（时间常数 tau，模拟 PX4 姿态环滞后）
  · 弹丸  ：仓库的**真 TargetModel**（DART：重力 + 二次气动阻力），经 shim 调用
  · 制导  ：仓库的**真 guidance_step**（PPN/TPN/APN），经 shim 调用
  · 防御塔：`scene_model.Tower` —— 同时评估 **"若不拦截，弹丸会不会打到塔"**

对应《守备拦截系统规划.md》：
  决策层（T2/T3/T4/T5…）已挂起；本脚本只做**制导层 L3** 的闭环验证，
  并顺带把场景实体（T1）用起来。

用法：
  python3 sim_phase_b_offline.py                              # cruise 模式
  python3 sim_phase_b_offline.py --mode pilot                 # ★ 巡航态 + 立即拦截
  python3 sim_phase_b_offline.py --mode pilot --csv t.csv      # 导出轨迹
  python3 sim_phase_b_offline.py --mode pilot --law 2          # 用 APN
  python3 sim_phase_b_offline.py --scene scene.json            # 从场景文件加载
"""
import argparse
import ctypes as C
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scene_model import Scene, Tower, preset_pilot      # noqa: E402


# ==========================================================================
#  动态库加载（与 test_guidance_shim.py 相同的方式）
# ==========================================================================
def find_lib(explicit):
    if explicit:
        return explicit
    here = os.path.dirname(os.path.abspath(__file__))
    names = ("libpure_guidance.so", "pure_guidance.dll",
             "libpure_guidance.dll", "libpure_guidance.dylib")
    for d in (here, os.getcwd()):
        for n in names:
            p = os.path.join(d, n)
            if os.path.exists(p):
                return p
    raise SystemExit("找不到制导核动态库（libpure_guidance.so / pure_guidance.dll）。\n"
                     "先运行 build_guidance_shim.sh，或把库路径作为第一个参数传入。")


def load_lib(path):
    lib = C.CDLL(os.path.abspath(path))
    lib.pgu_version.restype = C.c_int
    lib.pgu_create.restype = C.c_void_p
    lib.pgu_destroy.argtypes = [C.c_void_p]
    lib.pgu_config_target.argtypes = [C.c_void_p, C.POINTER(C.c_double)]
    lib.pgu_config_guidance.argtypes = [C.c_void_p, C.POINTER(C.c_double)]
    lib.pgu_config_sensor.argtypes = [C.c_void_p, C.POINTER(C.c_double)]
    lib.pgu_target_flight_time.restype = C.c_double
    lib.pgu_target_flight_time.argtypes = [C.c_void_p]
    lib.pgu_target_hits_ground.restype = C.c_int
    lib.pgu_target_hits_ground.argtypes = [C.c_void_p]
    lib.pgu_target_state.argtypes = [C.c_void_p, C.c_double, C.POINTER(C.c_double)]
    lib.pgu_step.argtypes = [C.c_void_p, C.c_double, C.c_double,
                             C.POINTER(C.c_double), C.POINTER(C.c_double),
                             C.POINTER(C.c_double), C.POINTER(C.c_double),
                             C.POINTER(C.c_double)]
    return lib


def arr(vals):
    return (C.c_double * len(vals))(*vals)


def norm3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def sub3(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def unit3(a, fallback=(1.0, 0.0, 0.0)):
    n = norm3(a)
    if n < 1e-9:
        return fallback
    return (a[0] / n, a[1] / n, a[2] / n)


def parse_vec3(s):
    parts = [float(x) for x in s.replace(" ", "").split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"需要 3 个分量（x,y,z）：{s!r}")
    return tuple(parts)


def v3txt(v, nd=2):
    return f"({v[0]:.{nd}f},{v[1]:.{nd}f},{v[2]:.{nd}f})"


# ==========================================================================
#  配置数组（与 shim 文档一致）
# ==========================================================================
def target_cfg(sc):
    c = [0.0] * 29
    c[0] = 2                                  # DART
    c[1:4] = list(sc["p0"])
    c[7] = sc["speed"]
    c[8] = math.radians(sc["elev"])
    c[9] = math.radians(sc["azim"])
    c[10], c[11], c[12], c[13] = sc["mass"], sc["cd"], sc["area"], sc["rho"]
    c[14:17] = [0.0, 0.0, -9.81]
    c[26], c[27], c[28] = 0.0005, 10.0, 0.0   # dt_table / t_max / ground_z
    return c


def guidance_cfg(sc):
    return [float(sc["law"]), sc["N"], sc["a_max"], 0.0, sc["t_go_freeze"], 5.0, 0.0]


def sensor_cfg():
    return [0.0, 0.1, 0.0, 0.0, 0.05, 0.05, 12345.0,
            1200.0, 1200.0, 320.0, 240.0, 640.0, 480.0, 0.5, 1.0]


# ==========================================================================
#  塔威胁评估（"若不拦截，弹丸会不会打到塔"）
# ==========================================================================
def evaluate_tower(L, h, tower, dt=0.002):
    """采弹丸标称弹道 → 到塔的最近距离。

    返回 dict：{d_min, t_at, p_at, threatened}
      d_min   : 到**塔体表面**的最近距离（0 = 打进塔里）
      threatened : d_min < tower.safety_margin（安全垫）
    """
    if tower is None:
        return None
    t_span = L.pgu_target_flight_time(h)
    n = int(t_span / dt) + 2
    tgt = arr([0.0] * 9)
    pts, ts = [], []
    for i in range(n):
        tt = min(i * dt, t_span)
        L.pgu_target_state(h, tt, tgt)
        pts.append((tgt[0], tgt[1], tgt[2]))
        ts.append(tt)
    d_min, idx = tower.closest_approach(pts)
    return {"d_min": d_min, "t_at": ts[idx], "p_at": pts[idx],
            "threatened": d_min < tower.safety_margin,
            "t_span": t_span}


# ==========================================================================
#  仿真主循环
# ==========================================================================
def run(L, h, sc, csv_path=None):
    mode = sc.get("mode", "cruise")

    P = list(sc["drone0"])
    V = list(sc.get("drone_v0", (0.0, 0.0, 0.0))) if mode == "pilot" else [0.0, 0.0, 0.0]
    A = [0.0, 0.0, 0.0]                    # 实际加速度（带一阶滞后）
    A_CMD = [0.0, 0.0, 0.0]                # 期望加速度

    q_id = arr([0.0, 0.0, 0.0, 1.0])
    cmd = arr([0.0] * 19)
    dbg = arr([0.0] * 17)
    tgt = arr([0.0] * 9)

    dt_s, dt_g, tau = sc["dt_sim"], sc["dt_g"], sc["tau"]
    t_max = sc["t_max"]
    flight_time = L.pgu_target_flight_time(h)
    hits_ground = bool(L.pgu_target_hits_ground(h))

    boost_dir = unit3(sc.get("boost_dir", (-1.0, 0.0, 0.0)))
    t_launch = float(sc.get("t_launch", 0.0))

    # ---- 模式初始化 ----
    if mode == "pilot":
        phase = "CRUISE"                   # 巡航段（弹丸未出现，保持给定速度）
        launched = False
    elif mode == "hot":
        phase = "BOOST"
        launched = True
    else:                                   # cruise
        phase = "BOOST"
        launched = False
    t_target0 = 0.0
    handover = None                         # (t, |v|, R, p) —— 交班/发射时刻的记录

    next_g = 0.0
    t = 0.0

    hit = False
    passed = False                          # 是否已越过真正最近点
    min_R, t_min = None, 0.0
    p_M_at_min = pT_at_min = None
    R_prev = None
    peak_a_cmd = 0.0
    peak_a_act = 0.0
    sat_steps = 0
    steps = 0
    modes = {}
    path_len = 0.0
    event_log = []

    f = None
    if csv_path:
        f = open(csv_path, "w", newline="")
        f.write("t,t_rel,phase,pMx,pMy,pMz,vMx,vMy,vMz,pTx,pTy,pTz,vTx,vTy,vTz,"
                "R_true,acx,acy,acz,a_norm,sat,mode,t_go,lam_dot,Vc,"
                "d_tower\n")

    # ★ 2026-10-17 审查：整段**没有 try/finally** ⇒ 中途抛异常时 CSV 句柄不关，
    #   Windows 上文件会一直被占（后续读/删都报错），内容也可能缺尾部。
    #   这里把主循环包起来，任何退出路径都关文件。
    try:
        while t <= t_max:
            # ---- pilot：到 t_launch 发射弹丸，**立即**开始制导 ----
            if mode == "pilot" and not launched and t + 1e-9 >= t_launch:
                launched = True
                t_target0 = t
                phase = "GUIDED"
                handover = (t, norm3(V), float("nan"), tuple(P))
                event_log.append(
                    f"t={t:.2f} **弹丸发射 → 立即开始制导**"
                    f"（无人机巡航 |v|={norm3(V):.2f} m/s @ {v3txt(P)}）")

            t_rel = (t - t_target0) if launched else 0.0
            if launched:
                L.pgu_target_state(h, t_rel, tgt)
                pT = (tgt[0], tgt[1], tgt[2])
                vT = (tgt[3], tgt[4], tgt[5])
                R_true = norm3(sub3(pT, P))
            else:
                pT = (float("nan"),) * 3
                vT = (float("nan"),) * 3
                R_true = float("inf")

            # ---------------- 制导（dt_g 零阶保持） ----------------
            if t + 1e-9 >= next_g:
                if phase == "GUIDED":
                    L.pgu_step(h, t_rel, 1.0, arr(P), arr(V), q_id, cmd, dbg)
                    A_CMD = [cmd[1], cmd[2], cmd[3]]
                    modes[int(cmd[8])] = modes.get(int(cmd[8]), 0) + 1
                    if cmd[5] > 0.5:
                        sat_steps += 1
                    if norm3(A_CMD) > peak_a_cmd:
                        peak_a_cmd = norm3(A_CMD)
                elif phase == "BOOST":
                    # hot：朝目标加速；cruise：朝固定方向预加速
                    d = unit3(sub3(pT, P)) if mode == "hot" else boost_dir
                    A_CMD = [sc["a_boost"] * d[k] for k in range(3)]
                    if norm3(V) >= sc["v_boost_min"]:
                        phase = "GUIDED"
                        if mode == "cruise":
                            launched = True
                            t_target0 = t
                            t_rel = 0.0
                            event_log.append(
                                f"t={t:.2f} 预加速到 |v|={norm3(V):.2f} m/s → "
                                f"**弹丸此刻发射**，交班 PPN（无人机已在 {v3txt(P)}）")
                        else:
                            event_log.append(
                                f"t={t:.2f} 交班 GUIDED（|v|={norm3(V):.2f} m/s, "
                                f"R={R_true:.2f} m）")
                        handover = (t, norm3(V), R_true, tuple(P))
                else:                            # CRUISE（pilot 的弹丸发射前）
                    A_CMD = [0.0, 0.0, 0.0]      # 保持巡航速度
                next_g += dt_g

            # ---------------- 命中 / 脱靶判定 ----------------
            # ⚠ 关键：**击中判定成立 ≠ 立刻停**。必须跑到"真正最近点"之后再停，
            #   否则统计到的"最近距离"只是"刚跌破 hit_radius 的那个采样点"，
            #   真实脱靶量会被截断掩盖（曾因此误以为精度卡在 0.5 m 上不去）。
            if launched:
                if min_R is None or R_true < min_R:
                    min_R, t_min = R_true, t
                    p_M_at_min, pT_at_min = tuple(P), tuple(pT)
                if R_true < sc["hit_radius"] and not hit:
                    hit = True
                    event_log.append(f"t={t:.3f} **进入命中球**（R={R_true:.3f} m < "
                                     f"{sc['hit_radius']} m，|v_M|={norm3(V):.2f} m/s）")
                # "已过最近点"：曾经接近到 5 m 内，且现在明显拉开
                if min_R is not None and min_R < 5.0 and R_true > min_R + 0.15:
                    passed = True
                R_prev = R_true

            # ---------------- 记录 ----------------
            d_tower = (math.nan if not launched or sc.get("tower") is None
                       else sc["tower"].distance_to_body(pT))
            if f:
                f.write(f"{t:.4f},{t_rel:.4f},{phase},"
                        f"{P[0]:.4f},{P[1]:.4f},{P[2]:.4f},"
                        f"{V[0]:.4f},{V[1]:.4f},{V[2]:.4f},"
                        f"{pT[0]:.4f},{pT[1]:.4f},{pT[2]:.4f},"
                        f"{vT[0]:.4f},{vT[1]:.4f},{vT[2]:.4f},"
                        f"{R_true:.4f},"
                        f"{A_CMD[0]:.4f},{A_CMD[1]:.4f},{A_CMD[2]:.4f},"
                        f"{norm3(A_CMD):.4f},{int(cmd[5])},{int(cmd[8])},"
                        f"{cmd[7]:.4f},{cmd[9]:.4f},{cmd[10]:.4f},"
                        f"{d_tower:.4f}\n")

            if passed or (launched and t_rel >= flight_time + 0.2):
                break

            # ---------------- 无人机动力学（一阶加速度响应 + 积分）----------------
            for k in range(3):
                if tau > 1e-9:
                    A[k] += (A_CMD[k] - A[k]) * (dt_s / tau)
                else:
                    A[k] = A_CMD[k]
                V[k] += A[k] * dt_s
                P[k] += V[k] * dt_s
            path_len += norm3(V) * dt_s
            if norm3(A) > peak_a_act:
                peak_a_act = norm3(A)

            t += dt_s
            steps += 1

    finally:
        if f:
            f.close()

    return {
        "mode": mode, "hit": hit, "missed": (passed and not hit),
        "min_R": min_R, "t_min": t_min,
        "p_M_at_min": p_M_at_min, "pT_at_min": pT_at_min,
        "peak_a_cmd": peak_a_cmd, "peak_a_act": peak_a_act,
        "sat_steps": sat_steps, "steps": steps, "modes": modes,
        "handover": handover, "flight_time": flight_time,
        "hits_ground": hits_ground, "events": event_log,
        "end_pM": tuple(P), "end_pT": tuple(pT) if launched else None,
        "t_end": t, "launched": launched, "path_len": path_len,
    }


# ==========================================================================
def build_parser():
    ap = argparse.ArgumentParser(
        description="制导闭环离线仿真（巡航态 → 弹丸 → 立即拦截）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("lib", nargs="?", help="动态库路径（默认：同目录自动查找）")
    ap.add_argument("--scene", help="从场景 JSON 加载（scene_model.Scene.save 产出）")
    # 场景
    ap.add_argument("--mode", choices=("pilot", "cruise", "hot"), default="pilot",
                    help="pilot=巡航态+立即拦截（★本阶段）；cruise/hot=阶段 B 的起步方式")
    ap.add_argument("--drone0", type=parse_vec3, default=(-4.0, 0.0, 8.0),
                    help="无人机初始位置 ENU")
    ap.add_argument("--drone-v0", type=parse_vec3, default=(-5.0, 0.0, 0.0),
                    help="无人机初始速度（**巡航速度**，pilot 模式用）")
    ap.add_argument("--boost-dir", type=parse_vec3, default=(-1.0, 0.0, 0.0),
                    help="cruise 模式的预加速方向")
    ap.add_argument("--t-launch", type=float, default=0.0,
                    help="弹丸发射时刻（pilot 模式；0=一开始就发射）")
    ap.add_argument("--p0", type=parse_vec3, default=(-14.0, 0.0, 10.0),
                    help="弹丸发射点 ENU")
    ap.add_argument("--speed", type=float, default=14.0, help="弹丸速率 m/s")
    ap.add_argument("--elev", type=float, default=-6.0,
                    help="弹丸仰角 deg（负=俯冲）")
    ap.add_argument("--azim", type=float, default=0.0, help="弹丸方位角 deg")
    ap.add_argument("--mass", type=float, default=0.40)
    ap.add_argument("--cd", type=float, default=0.50)
    ap.add_argument("--area", type=float, default=2.83e-3)
    ap.add_argument("--rho", type=float, default=1.225)
    # 防御塔
    ap.add_argument("--tower", type=parse_vec3, default=(0.0, 0.0, 0.0),
                    help="塔底面圆心 ENU")
    ap.add_argument("--tower-radius", type=float, default=1.2)
    ap.add_argument("--tower-height", type=float, default=3.0)
    ap.add_argument("--tower-margin", type=float, default=1.0,
                    help="安全垫 m（威胁判定的富裕度）")
    ap.add_argument("--no-tower", action="store_true", help="不建塔（不做威胁评估）")
    # 制导
    ap.add_argument("--law", type=int, default=0, help="0=PPN 1=TPN 2=APN")
    ap.add_argument("--N", type=float, default=6.0)
    ap.add_argument("--a-max", type=float, default=10.0)
    ap.add_argument("--t-go-freeze", type=float, default=0.10)
    # 助推 / 预加速（cruise / hot 模式用）
    ap.add_argument("--a-boost", type=float, default=6.0)
    ap.add_argument("--v-boost-min", type=float, default=5.0)
    # 数值
    ap.add_argument("--tau", type=float, default=0.20)
    ap.add_argument("--dt-g", type=float, default=0.05, help="制导周期（20 Hz）")
    ap.add_argument("--dt-sim", type=float, default=0.0025)
    ap.add_argument("--hit-radius", type=float, default=0.5)
    ap.add_argument("--t-max", type=float, default=10.0)
    ap.add_argument("--csv", help="导出轨迹 CSV")
    return ap


def main():
    ap = build_parser()
    sc = vars(ap.parse_args())

    # 从场景文件覆盖（CLI 显式给的优先）
    if sc["scene"]:
        s = Scene.load(sc["scene"])
        for k, v in s.to_sim_dict().items():
            sc[k] = v
        print(f"（已从场景文件加载：{sc['scene']}）")

    # 建塔对象
    tower = None
    if not sc["no_tower"]:
        tower = Tower(name="tower", center=tuple(sc["tower"]),
                      radius=sc["tower_radius"], height=sc["tower_height"],
                      safety_margin=sc["tower_margin"])
    sc["tower"] = tower

    L = load_lib(find_lib(sc["lib"]))
    h = L.pgu_create()
    L.pgu_config_target(h, arr(target_cfg(sc)))
    L.pgu_config_guidance(h, arr(guidance_cfg(sc)))
    L.pgu_config_sensor(h, arr(sensor_cfg()))

    flight_time = L.pgu_target_flight_time(h)
    hits_ground = bool(L.pgu_target_hits_ground(h))

    law_name = ["PPN", "TPN", "APN"][sc["law"]] if 0 <= sc["law"] <= 2 else "?"
    print("=" * 76)
    print("制导闭环离线仿真（无人机质点 + 一阶响应 / 真制导核 / 真 DART 弹丸）")
    print("=" * 76)
    if sc["mode"] == "pilot":
        print(f"  无人机: **已在巡航** @ {v3txt(sc['drone0'])}，"
              f"速度 {v3txt(sc['drone_v0'])} = {norm3(sc['drone_v0']):.2f} m/s")
        print(f"          弹丸于 t={sc['t_launch']:.2f}s 发射 → **立即开始制导**"
              f"（决策层已判定此刻开始）")
    elif sc["mode"] == "cruise":
        print(f"  无人机: 起点 {v3txt(sc['drone0'])} 悬停起步 → 朝 "
              f"{v3txt(sc['boost_dir'])} 预加速至 |v|≥{sc['v_boost_min']} m/s"
              f"（{sc['a_boost']} m/s²）→ 发射弹丸、交班制导")
    else:
        print(f"  无人机: 起点 {v3txt(sc['drone0'])} 悬停起步 → 朝弹丸助推至 "
              f"|v|≥{sc['v_boost_min']} m/s")
    print(f"  弹丸  : DART 从 {v3txt(sc['p0'])} 发射，{sc['speed']:.1f} m/s，"
          f"仰角 {sc['elev']:+.1f}°，方位 {sc['azim']:.1f}°"
          f"   [重力 + 二次阻力：m={sc['mass']}kg Cd={sc['cd']} "
          f"A={sc['area'] * 1e4:.1f}cm²]")
    print(f"          弹丸飞行时间 = {flight_time:.2f} s（hits_ground={hits_ground}）")
    print(f"  制导  : {law_name} N={sc['N']} a_max={sc['a_max']} "
          f"t_go_freeze={sc['t_go_freeze']}   指令 {1.0 / sc['dt_g']:.0f} Hz   "
          f"响应 tau={sc['tau']} s")
    if tower:
        print(f"  防御塔: {tower.describe()}")
    print("-" * 76)

    # ---- 塔威胁评估（不拦截的情况）----
    tev = evaluate_tower(L, h, tower) if tower else None
    if tev:
        mark = "⚠ 构成威胁" if tev["threatened"] else "✓ 不构成威胁"
        print(f"  [若不拦截] 弹丸最近距塔 **{tev['d_min']:.2f} m**"
              f"（t={tev['t_at']:.2f}s @ {v3txt(tev['p_at'])}，安全垫 "
              f"{tower.safety_margin:.2f} m）→ {mark}")
        print("-" * 76)

    r = run(L, h, sc, csv_path=sc["csv"])

    # ---- 结果 ----
    print()
    print("=" * 76)
    print("结果")
    print("=" * 76)
    if r["min_R"] is not None:
        verdict = ("✓ **命中**" if r["hit"]
                   else ("✗ 脱靶（已过最近点）" if r["missed"] else "— 未命中（时间到）"))
        print(f"  拦截结果  : {verdict}")
        print(f"  最近距离  : {r['min_R']:.3f} m  @ t={r['t_min']:.3f} s"
              f"（命中阈值 {sc['hit_radius']} m）")
        if r["p_M_at_min"]:
            print(f"  最近处位置: 无人机 {v3txt(r['p_M_at_min'])}  "
                  f"弹丸 {v3txt(r['pT_at_min'])}")
            if tower:
                print(f"             拦截点距塔 "
                      f"{math.hypot(r['pT_at_min'][0] - tower.center[0], r['pT_at_min'][1] - tower.center[1]):.2f} m"
                      f"（水平）/ {r['pT_at_min'][2]:.2f} m（高度）")
    if r["handover"]:
        th, vh, rh, ph = r["handover"]
        rt = f"{rh:.2f} m" if math.isfinite(rh) else "—"
        lbl = "弹丸发射" if sc["mode"] == "pilot" else "交班"
        print(f"  {lbl:8s}  : t={th:.2f} s（|v_M|={vh:.2f} m/s, R={rt}）")
    print(f"  峰值指令  : {r['peak_a_cmd']:.2f} m/s²"
          f"{'（限幅触发 ' + str(r['sat_steps']) + ' 步）' if r['sat_steps'] else '（未触发限幅）'}")
    print(f"  实际峰值  : {r['peak_a_act']:.2f} m/s²（含一阶滞后）")
    print(f"  制导模式  : {r['modes']}   (0=PN 1=APN 2=HOLD 3=SEARCH)")
    print(f"  无人机航程: {r['path_len']:.2f} m（t=0→{r['t_end']:.2f} s）")
    if r["end_pT"]:
        print(f"  末状态    : 无人机 {v3txt(r['end_pM'])}  弹丸 {v3txt(r['end_pT'])}")
    for e in r["events"]:
        print(f"  · {e}")
    if sc["csv"]:
        print(f"  轨迹 CSV  : {sc['csv']}")

    L.pgu_destroy(h)

    # ---- 结论 ----
    print("-" * 76)
    ok = True
    if not r["hit"]:
        print("  结论：**未命中** —— 调整场景（拉近/降速/提高 a_max/增大巡航速度）或制导参数。")
        ok = False
    else:
        print("  结论：**制导闭环成立** —— 巡航态起步 + 弹丸（含重力与气动）+ 立即拦截 → 命中。")
    if tev and tev["threatened"] and r["hit"]:
        print("        且该弹丸**确实威胁到塔**（不拦截会打到），本拦截是有意义的。")
    elif tev and not tev["threatened"]:
        print("        ⚠ 但该弹丸**本不威胁塔** —— 建议调整场景，让演示更有意义。")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
