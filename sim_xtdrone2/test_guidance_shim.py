#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ctypes ↔ C++ 制导核（pure_guidance_shim）冒烟 + 等价性测试。不需要 ROS。

分三部分：
  1) SEARCH 分支：目标未出现 → mode=SEARCH、零指令；
  2) 全闭环：**逐字复刻 pn_sim 的默认场景与主循环**
     （PPN N=6、DART 迎头拦截、RK4 dt=0.001、制导 0.002、IDEAL 传感器），
     断言命中、最近距离、饱和步数、mode 轨迹；
  3) 可选等价性对照：`--ref <pn_sim 生成的 CSV>` —— 逐行比对，
     证明 shim 链路与仓库离线仿真一致（容差见 compare_csv）。

用法：
  python3 test_guidance_shim.py                          # 自动找同目录 .so/.dll
  python3 test_guidance_shim.py /path/libpure_guidance.so
  python3 test_guidance_shim.py --out my.csv             # 导出复刻轨迹（列同 pn_sim）
  python3 test_guidance_shim.py --ref pn_sim_log.csv     # 与 pn_sim 轨迹对照
"""
import argparse
import ctypes as C
import math
import os
import sys

FAIL = []


def check(ok, msg):
    print(("  PASS  " if ok else "  FAIL  ") + msg)
    if not ok:
        FAIL.append(msg)


# ==========================================================================
#  动态库加载
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
    lib.pgu_sensor_stats.argtypes = [C.c_void_p, C.POINTER(C.c_double)]
    lib.pgu_step.argtypes = [C.c_void_p, C.c_double, C.c_double,
                             C.POINTER(C.c_double), C.POINTER(C.c_double),
                             C.POINTER(C.c_double), C.POINTER(C.c_double),
                             C.POINTER(C.c_double)]
    return lib


def arr(vals):
    return (C.c_double * len(vals))(*vals)


# ==========================================================================
#  场景常量 —— 与 pn_sim.cpp main() 的默认场景逐字一致
# ==========================================================================
SCEN = {
    "p_M0": (0.0, 0.0, 6.0),
    "v_M0": (18.0, 0.0, 0.0),
    "target_type": 2,               # DART
    "p0": (40.0, 0.0, 2.0),
    "launch_speed": 18.0,
    "launch_elev": math.radians(30.0),
    "launch_azim": math.pi,         # 朝 −x（正对无人机）
    "mass": 0.40, "drag_cd": 0.50, "drag_area": 2.83e-3, "rho": 1.225,
    "N": 6.0, "a_max": 30.0, "t_go_freeze": 0.15,
    "dt_sim": 0.001, "dt_g": 0.002, "t_end": 20.0, "hit_radius": 0.10,
}


def target_cfg(s):
    c = [0.0] * 29
    c[0] = s["target_type"]
    c[1:4] = list(s["p0"])
    c[4:7] = [0.0, 0.0, 0.0]                      # v0（用 launch_speed）
    c[7] = s["launch_speed"]; c[8] = s["launch_elev"]; c[9] = s["launch_azim"]
    c[10] = s["mass"]; c[11] = s["drag_cd"]; c[12] = s["drag_area"]; c[13] = s["rho"]
    c[14:17] = [0.0, 0.0, -9.81]                  # gravity
    c[17:20] = [0.0, 0.0, 0.0]                    # wind
    c[20:23] = [0.0, 0.0, 0.0]                    # a_maneuver
    c[23] = 0.6; c[24] = 5.0; c[25] = 3.0         # turn_rate / snake
    c[26] = 0.0005; c[27] = 10.0; c[28] = 0.0     # dt_table / t_max / ground_z
    return c


def guidance_cfg(s):
    return [0.0, s["N"], s["a_max"], 0.0, s["t_go_freeze"], 5.0, 0.0]
    #      PPN                    BY_TGO                    R_freeze  FROM_RELVEL


def sensor_cfg():
    return [0.0, 0.1, 0.0, 0.0, 0.05, 0.05, 12345.0,
            1200.0, 1200.0, 320.0, 240.0, 640.0, 480.0, 0.5, 1.0]
    #      IDEAL + 全默认值（本测试不消费这些）


# ==========================================================================
#  主循环 —— 逐字复刻 pn_sim.cpp 的 run_sim()
# ==========================================================================
def run_scenario(L, h, s, csv_path=None):
    P = list(s["p_M0"])
    V = list(s["v_M0"])
    q_id = arr([0.0, 0.0, 0.0, 1.0])
    cmd = arr([0.0] * 19)      # 初始为零指令（与 C++ 默认 GuidanceCmd 一致）
    dbg = arr([0.0] * 17)
    tgt = arr([0.0] * 9)

    dt, dt_g = s["dt_sim"], s["dt_g"]
    t_span = min(s["t_end"], L.pgu_target_flight_time(h) + 0.05)

    t, next_g = 0.0, 0.0
    min_R, t_min = None, 0.0
    passed = hit = False
    peak_a = peak_lam = 0.0
    sat_steps = steps = 0
    modes = {}
    n_rows = 0

    f = None
    if csv_path:
        f = open(csv_path, "w", newline="")
        f.write("t,pMx,pMy,pMz,vMx,vMy,vMz,pTx,pTy,pTz,vTx,vTy,vTz,"
                "R,R_est,R_upd,Vc,lam_dot,acx,acy,acz,a_norm,sat,mode,t_go\n")

    R_upd = 0.0
    while t <= t_span:
        L.pgu_target_state(h, t, tgt)
        R = math.sqrt((tgt[0] - P[0]) ** 2 + (tgt[1] - P[1]) ** 2 + (tgt[2] - P[2]) ** 2)

        if t + 1e-9 >= next_g:
            L.pgu_step(h, t, 1.0, arr(P), arr(V), q_id, cmd, dbg)
            R_upd = R
            next_g += dt_g

        if min_R is None or R < min_R:
            min_R, t_min = R, t
        if R < s["hit_radius"]:
            hit = True
        if R > min_R + 0.1:
            passed = True

        a_norm = cmd[4]
        if R > 1.0:
            if a_norm > peak_a:
                peak_a = a_norm
            if cmd[9] > peak_lam:
                peak_lam = cmd[9]
        if cmd[5] > 0.5:
            sat_steps += 1
        mode = int(cmd[8])
        modes[mode] = modes.get(mode, 0) + 1

        if f:
            f.write("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                    "%.4f,%.4f,%.4f,%.4f,%.6f,%.4f,%.4f,%.4f,%.4f,%d,%d,%.4f\n"
                    % (t, P[0], P[1], P[2], V[0], V[1], V[2],
                       tgt[0], tgt[1], tgt[2], tgt[3], tgt[4], tgt[5],
                       R, cmd[11], R_upd, cmd[10], cmd[9],
                       cmd[1], cmd[2], cmd[3], cmd[4],
                       int(cmd[5]), mode, cmd[7]))
            n_rows += 1

        if passed:
            break

        # 积分：常数加速度下与 pn_sim 的 rk4_step 逐字等价（抛物线）
        for k in range(3):
            ak, vk = cmd[1 + k], V[k]
            k1p = vk
            k2p = vk + ak * (dt * 0.5)
            k3p = k2p
            k4p = vk + ak * dt
            P[k] += (k1p + k2p * 2.0 + k3p * 2.0 + k4p) * (dt / 6.0)
            V[k] += (ak + ak * 2.0 + ak * 2.0 + ak) * (dt / 6.0)
        t += dt
        steps += 1

    if f:
        f.close()

    return {
        "hit": hit, "min_R": min_R, "t_min": t_min,
        "peak_a": peak_a, "peak_lam": peak_lam,
        "sat_steps": sat_steps, "steps": steps,
        "modes": modes, "rows": n_rows, "t_last": t,
    }


def compare_csv(mine_path, ref_path, tol=1e-3):
    import csv
    A = list(csv.reader(open(mine_path, newline="", encoding="utf-8")))
    B = list(csv.reader(open(ref_path, newline="", encoding="utf-8")))
    ok = True
    if len(A) != len(B):
        print(f"  FAIL  行数不同: shim={len(A)} vs pn_sim={len(B)}")
        ok = False
    if A[0] != B[0]:
        print(f"  FAIL  表头不同:\n    {A[0]}\n    {B[0]}")
        ok = False
    n = min(len(A), len(B))
    hdr = A[0]
    maxd = {k: 0.0 for k in hdr}
    for i in range(1, n):
        for j in range(len(hdr)):
            d = abs(float(A[i][j]) - float(B[i][j]))
            if d > maxd[hdr[j]]:
                maxd[hdr[j]] = d
    worst_col, worst_d = max(maxd.items(), key=lambda kv: kv[1])
    print(f"  逐行对比（{n - 1} 行 × {len(hdr)} 列）：最大偏差 = {worst_d:.3g}（列 {worst_col}）")
    bad = {k: v for k, v in maxd.items() if v > tol}
    if bad:
        print(f"  FAIL  以下列偏差超过 {tol:g}: " +
              ", ".join(f"{k}={v:.3g}" for k, v in sorted(bad.items())))
        ok = False
    else:
        print(f"  PASS  全部列偏差 ≤ {tol:g} → shim 链路与 pn_sim 一致 ✓")
    return ok


# ==========================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("lib", nargs="?", help="动态库路径（默认：同目录自动查找）")
    ap.add_argument("--out", help="导出复刻轨迹 CSV（列与 pn_sim 相同）")
    ap.add_argument("--ref", help="与 pn_sim 生成的 CSV 逐行对比")
    args = ap.parse_args()

    path = find_lib(args.lib)
    L = load_lib(path)
    print(f"已加载制导核: {path}  (ABI 版本 {L.pgu_version()})")

    h = L.pgu_create()
    L.pgu_config_target(h, arr(target_cfg(SCEN)))
    L.pgu_config_guidance(h, arr(guidance_cfg(SCEN)))
    L.pgu_config_sensor(h, arr(sensor_cfg()))
    print(f"目标模型: DART  flight_time={L.pgu_target_flight_time(h):.2f} s  "
          f"hits_ground={L.pgu_target_hits_ground(h)}")

    # ---- 1) SEARCH：目标未出现 ----
    print()
    print("=" * 70)
    print("用例 1：目标未出现 → SEARCH、零指令")
    print("=" * 70)
    cmd = arr([0.0] * 19)
    dbg = arr([0.0] * 17)
    L.pgu_step(h, -1.0, 0.0, arr([0.0, 0.0, 10.0]), arr([0.0, 0.0, 0.0]),
               arr([0.0, 0.0, 0.0, 1.0]), cmd, dbg)
    check(int(cmd[8]) == 3, f"mode == SEARCH(3)（实际 {int(cmd[8])}）")
    check(abs(cmd[4]) < 1e-12, f"|a_cmd| == 0（实际 {cmd[4]:.3g}）")
    check(dbg[15] < 0, f"dbg R_true == −1 表示无目标（实际 {dbg[15]:.3g}）")

    # ---- 2) 全闭环复刻 pn_sim 默认场景 ----
    print()
    print("=" * 70)
    print("用例 2：复刻 pn_sim 默认场景（PPN N=6 · DART 迎头拦截）")
    print("=" * 70)
    r = run_scenario(L, h, SCEN, csv_path=args.out)
    print(f"  命中        : {'是' if r['hit'] else '否'}")
    print(f"  最近距离    : {r['min_R']:.3f} m  (t={r['t_min']:.2f} s)")
    print(f"  峰值加速度  : {r['peak_a']:.2f} m/s²（R>1m）")
    print(f"  峰值 lam_dot: {r['peak_lam']:.4f} rad/s（R>1m）")
    print(f"  饱和步数    : {r['sat_steps']} / {r['steps']}")
    print(f"  mode 分布   : {r['modes']}   (0=PN 1=APN 2=HOLD 3=SEARCH)")
    if args.out:
        print(f"  轨迹 CSV    : {args.out}（{r['rows']} 行）")
    stats = arr([0.0, 0.0, 0.0])
    L.pgu_sensor_stats(h, stats)
    print(f"  传感器统计  : frames={int(stats[0])} dropped={int(stats[1])} lost={int(stats[2])}")

    check(r["hit"], "命中（最近距离 < 0.10 m 命中球）")
    check(r["min_R"] is not None and r["min_R"] < 0.3, f"最近距离 < 0.3 m（实际 {r['min_R']:.3f}）")
    check(r["modes"].get(0, 0) > 0, "出现过 PN 模式")
    check(r["modes"].get(2, 0) > 0, "末端出现过 HOLD（t_go 冻结生效）")
    check(r["sat_steps"] < 0.2 * max(r["steps"], 1),
          f"饱和步数占比 < 20%（{r['sat_steps']}/{r['steps']}）")
    check(r["peak_a"] <= SCEN["a_max"] + 1e-9, "峰值指令 ≤ a_max（矢量缩放限幅生效）")

    # ---- 3) 可选：与 pn_sim 轨迹对照 ----
    if args.ref:
        print()
        print("=" * 70)
        print(f"用例 3：与 pn_sim 轨迹逐行对比（{args.ref}）")
        print("=" * 70)
        if not args.out:
            print("  （需要 --out 一起使用；先把本脚本的轨迹写盘）")
            check(False, "--ref 需与 --out 同时使用")
        else:
            check(compare_csv(args.out, args.ref), "轨迹一致")

    L.pgu_destroy(h)
    print()
    print("=" * 70)
    print("全部通过 ✅" if not FAIL else f"失败 {len(FAIL)} 项 ❌")
    print("=" * 70)
    return 0 if not FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
