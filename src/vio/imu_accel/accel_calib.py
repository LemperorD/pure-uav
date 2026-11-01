#!/usr/bin/env python3
"""accel_calib.py —— 加速度计「尺度因子 + 三轴不正交」标定（六姿态法）。

为什么需要它
------------
`kalibr_imu_chain.yaml` 里有两个完全不同的东西：

    accelerometer_noise_density / accelerometer_random_walk   ← 随机误差，Allan 方差测
    Ta / R_IMUtoACC                                           ← 确定性误差，本工具测

Allan 方差**在物理上测不出尺度误差**：它只看同一姿态下数据的抖动。
而 Kalibr 的 imu.yaml 格式虽然留了 Ta 字段，Kalibr 本身从不估计它，
所以那份配置里的 `Ta = 单位阵` 意思是"没填过"，不是"测出来是单位阵"。

后果：加速度计 1° 的轴间不正交，会让重力漏进水平轴
      9.81 × sin(1°) = 0.17 m/s²，而且这个泄漏随姿态变化、滤波器的
      3 轴零偏吸收不掉。积分 20 秒 = 34 m。这正是我们观测到的
      0.14~0.60 m/s² 速度斜坡。

模型（与 imucal / Ferraris / Tedaldi 一致）
------------------------------------------
    a_corrected = F · (a_raw − b),     ‖a_corrected‖ = g

F 为 3x3（含尺度与不正交），b 为 3x1 零偏。OpenVINS 里用的是：

    a_hat = R_ACCtoIMU · Da · (a_raw − bias_a)     其中 Da = Ta⁻¹
    （见 ov_msckf/src/state/Propagator.cpp）

所以要写进配置的是 **Ta = F⁻¹**、`R_IMUtoACC = I`（零偏交给滤波器自己估）。

怎么采数据
----------
用现成的 `imu/src/record_static_imu.py`（不需要相机）：

    python imu/src/record_static_imu.py 120        # 录 120 秒

录的时候把 IMU 依次摆 **12 个朝向，每个朝向静止 10~15 秒**（见下表）。
之所以要 12 个而不是 6 个：椭球（含中心）有 9 个参数，6 个点定不出来 ——
设计矩阵是 6x9，秩必然小于 9，拟合结果不唯一。实测条件数：
    6 个面        → 秩不足，不可用 ❌
    10 个         → 条件数 6.1  ✅
    12 个（下表） → 条件数 6.4  ✅
    6 面+8 角=14  → 条件数 3.1  ✅✅（最好，但摆角很难）

12 个姿态（"哪面朝下"用机体的鼻子/右侧/肚子描述，按你飞机的实际朝向改）：
    1  正放（肚子朝下）              6  左侧朝下
    2  倒放（背朝下）                7  头 + 右侧同时朝下（靠墙角）
    3  头朝下（鼻子朝地）            8  头 + 左侧同时朝下
    4  尾朝下                        9  尾 + 右侧同时朝下
    5  右侧朝下                     10  尾 + 左侧同时朝下
                                    11  肚子朝下、机头垫高约 45°
                                    12  右侧朝下、肚子垫高约 45°

⚠️ **只摆 6 个轴对齐朝向是不够的**：椭球有 9 个参数，6 个点定不出来，
   会拟合出一个"恰好穿过这 6 个点"的错矩阵。本工具自检的实测对比：
       6 姿态轴对齐  → 尺度误差 0.15%，不正交误差 2.9°   ❌
       12 姿态带倾斜 → 尺度误差 0.006%，不正交误差 0.027° ✅

然后：

    python tools/accel_calib.py --csv imu/data/static_20261021_101500.csv

自检（不需要任何硬件）：

    python tools/accel_calib.py --self-test
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

G_STD = 9.81  # 与 estimator_config.yaml 的 gravity_mag 一致


# --------------------------------------------------------------------------
# 1. 读数据
# --------------------------------------------------------------------------
def load_imu(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """返回 (acc Nx3, gyro Nx3, t 秒)。自动识别列名。"""
    with open(path) as fh:
        header = fh.readline().strip()
    cols = [c.strip().lower() for c in header.replace(",", " ").split()]
    # 常见两种：host_ns,fc_time_us,ax,ay,az,gx,gy,gz  和  timestamp_us,gx,gy,gz,ax,ay,az
    def pick(names):
        for n in names:
            if n in cols:
                return cols.index(n)
        return None

    ia = pick(["ax", "acc_x", "a_x"])
    ig = pick(["gx", "gyr_x", "g_x"])
    it = pick(["fc_time_us", "timestamp_us", "time_us", "host_ns"])
    if ia is None or ig is None or it is None:
        raise SystemExit(f"认不出列：{cols}\n需要 ax..az / gx..gz / 一个时间列")
    need = max(ia + 2, ig + 2, it) + 1
    if len(cols) < need:
        raise SystemExit(f"列不够：{cols}")

    raw = np.genfromtxt(path, delimiter=",", skip_header=1)
    if raw.ndim == 1:
        raw = raw.reshape(1, -1)
    t = raw[:, it].astype(np.float64)
    tname = cols[it]
    # 优先按列名判断单位（数值猜测在合成/截断数据上会错）
    if "ns" in tname:
        t = t * 1e-9
    elif "us" in tname or "usec" in tname:
        t = t * 1e-6
    elif "ms" in tname:
        t = t * 1e-3
    elif "sec" in tname:
        pass
    elif t[0] > 1e17:
        t = t * 1e-9
    elif t[0] > 1e14:
        t = t * 1e-6
    elif t[0] > 1e11:
        t = t * 1e-3
    t = t - t[0]
    acc = raw[:, ia:ia + 3]
    gyr = raw[:, ig:ig + 3]
    return acc, gyr, t


# --------------------------------------------------------------------------
# 2. 找静止段（每个姿态一段）
# --------------------------------------------------------------------------
def find_static_runs(acc: np.ndarray, gyr: np.ndarray, t: np.ndarray,
                     win_s: float = 0.5, max_gyro: float = 0.05,
                     max_std: float = 0.08, min_len_s: float = 4.0):
    """把连续静止的样本切成若干段，每段给一个平均加速度向量。"""
    if len(t) < 10:
        return []
    dt = float(np.median(np.diff(t)))
    k = max(5, int(win_s / max(dt, 1e-4)))
    na = np.linalg.norm(acc, axis=1)
    ng = np.linalg.norm(gyr, axis=1)

    # 滑窗稳定性
    pad = k // 2
    ker = np.ones(k) / k
    mu = np.convolve(na, ker, mode="same")
    var = np.convolve((na - mu) ** 2, ker, mode="same")
    sd = np.sqrt(np.maximum(var, 0))
    gs = np.convolve(ng, ker, mode="same")

    ok = (sd < max_std) & (gs < max_gyro)
    runs = []
    i = 0
    n = len(ok)
    while i < n:
        if not ok[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and ok[j + 1]:
            j += 1
        if t[j] - t[i] >= min_len_s:
            sel = slice(i, j + 1)
            runs.append({
                "t0": float(t[i]), "t1": float(t[j]),
                "acc": acc[sel].mean(axis=0),
                "std": float(np.linalg.norm(acc[sel], axis=1).std()),
                "n": int(j - i + 1),
            })
        i = j + 1
    return runs


def cluster_orientations(runs, min_sep_deg: float = 12.0):
    """按方向聚类：同一姿态可能被切成几段，合并掉。"""
    if not runs:
        return []
    groups = []
    for r in runs:
        u = r["acc"] / np.linalg.norm(r["acc"])
        placed = False
        for g in groups:
            if np.degrees(np.arccos(np.clip(u @ g["u"], -1, 1))) < min_sep_deg:
                g["runs"].append(r)
                g["u"] = (g["u"] + u) / 2
                g["u"] /= np.linalg.norm(g["u"])
                placed = True
                break
        if not placed:
            groups.append({"u": u, "runs": [r]})
    out = []
    for g in groups:
        A = np.array([r["acc"] for r in g["runs"]])
        out.append({"acc": A.mean(axis=0), "n": sum(r["n"] for r in g["runs"]),
                    "n_runs": len(g["runs"])})
    out.sort(key=lambda x: -x["n"])
    return out


def conditioning(groups):
    """姿态覆盖是否足以确定椭球的 9 个参数。

    椭球（含中心）有 9 个参数，所以**至少要有 10 个朝向**（9 个是理论下限，
    第 10 个提供冗余）。只摆 6 个轴对齐朝向时设计矩阵是 6x9，秩必然 <9，
    拟合结果不唯一 —— 会得到一个"恰好穿过这 6 个点"的错矩阵。

    返回 (条件数, 第9奇异值)；行数 <9 时条件数为 inf。
    """
    U = np.array([gp["acc"] for gp in groups])
    U = U / np.linalg.norm(U, axis=1, keepdims=True)
    x, y, z = U[:, 0], U[:, 1], U[:, 2]
    D = np.column_stack([x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z,
                         2 * x, 2 * y, 2 * z])
    n = D.shape[0]
    if n < 9:
        return float("inf"), 0.0
    sv = np.sort(np.linalg.svd(D, compute_uv=False))[::-1]
    return float(sv[0] / sv[8]), float(sv[8])


# --------------------------------------------------------------------------
# 3. 拟合 F, b
# --------------------------------------------------------------------------
def fit_algebraic(acc: np.ndarray):
    """椭球拟合（线性最小二乘）：(a−b)ᵀQ(a−b)=1。返回 b 和 Q。

    参数化：x^T P x + q^T x = 1，P 对称 6 参数 + q 3 参数。
    展开后对 P、q 线性。
    """
    x, y, z = acc[:, 0], acc[:, 1], acc[:, 2]
    D = np.column_stack([x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z,
                         2 * x, 2 * y, 2 * z])
    rhs = np.ones(len(acc))
    sol, *_ = np.linalg.lstsq(D, rhs, rcond=None)
    P = np.array([[sol[0], sol[3], sol[4]],
                  [sol[3], sol[1], sol[5]],
                  [sol[4], sol[5], sol[2]]])
    q = sol[6:9]
    b = -0.5 * np.linalg.solve(P, q)
    return b, P


def ellipsoid_to_F(P: np.ndarray, g: float) -> np.ndarray:
    """由 (a−b)ᵀQ(a−b)=1 得 F = g·chol(Q)（上三角，消除旋转自由度）。"""
    # 归一化：把 Q 缩放成使椭球半轴乘积合理
    ev = np.linalg.eigvalsh(P)
    if np.any(ev <= 0):
        raise ValueError("椭球矩阵非正定（姿态激励不足或数据有问题）")
    # Q 对应的二次型等于 1：aᵀQa=1 ⇒ 半轴 = 1/sqrt(ev)
    # 缩放使 aᵀQa = 1 对应 ‖Fa‖ = g  ⇒ Q = FᵀF/g²
    L = np.linalg.cholesky(P)      # P = L Lᵀ，L 下三角
    # F = g Lᵀ  ⇒ FᵀF = g² L Lᵀ = g² P  ⇒ (a-b)ᵀFᵀF(a-b)/g² = (a-b)ᵀP(a-b) = 1 ✓
    return g * L.T


def refine(F: np.ndarray, b: np.ndarray, acc: np.ndarray, g: float,
           iters: int = 40):
    """Gauss-Newton：min Σ (‖F(a−b)‖ − g)²"""

    def unpack(p):
        return p[:9].reshape(3, 3), p[9:]

    def resid(p):
        Fm, bm = unpack(p)
        r = np.linalg.norm((acc - bm) @ Fm.T, axis=1) - g
        return r

    p = np.concatenate([F.ravel(), b])
    lam = 1e-6
    for _ in range(iters):
        r = resid(p)
        J = np.zeros((len(r), 12))
        eps = 1e-7
        for k in range(12):
            pp = p.copy()
            pp[k] += eps
            J[:, k] = (resid(pp) - r) / eps
        A = J.T @ J + lam * np.eye(12)
        try:
            step = np.linalg.solve(A, J.T @ r)
        except np.linalg.LinAlgError:
            break
        p_new = p - step
        if np.linalg.norm(resid(p_new)) <= np.linalg.norm(r):
            p = p_new
            lam = max(lam * 0.5, 1e-9)
        else:
            lam *= 5
        if np.linalg.norm(step) < 1e-10:
            break
    return unpack(p)


# --------------------------------------------------------------------------
# 3b. Ferraris 六姿态法（只需要 6 个面）
# --------------------------------------------------------------------------
# 椭球法要 ≥10 个姿态，但六面法只要 6 个 —— 代价是它**假设**这 6 个面
# 正好垂直于传感器三个轴。推导：
#     a_corrected = F·(a_raw − b) = g·d     （d 是该姿态下重力在机体系的单位方向）
#   ⇒  a_raw = g·F⁻¹·d + b
#   对成对的 ±d：
#     b        = (a_+d + a_−d)/2
#     g·F⁻¹·d  = (a_+d − a_−d)/2
#   取 d = 三个机体轴，就得到 g·F⁻¹ 的三列 ⇒ F = g·inv(cols)
#
# 机体系是 FRD（x 前、y 右、z 下），所以：
#     肚子朝下 = +z,  背朝下 = −z
#     鼻子朝下 = +x,  尾巴朝下 = −x
#     右侧朝下 = +y,  左侧朝下 = −y

FACE_ALIASES = {
    "belly": ("z", +1), "down": ("z", +1), "bottom": ("z", +1),
    "back": ("z", -1), "top": ("z", -1), "upside_down": ("z", -1),
    "nose": ("x", +1), "head": ("x", +1), "front": ("x", +1),
    "tail": ("x", -1), "rear": ("x", -1),
    "right": ("y", +1),
    "left": ("y", -1),
}


def fit_ferraris(poses, g):
    """poses: {"belly": a_vec, "back": a_vec, ...} 六个面朝下时的平均加速度。

    返回 (F, b, 自检信息)
    """
    need = [("z", +1, "belly"), ("z", -1, "back"),
            ("x", +1, "nose"), ("x", -1, "tail"),
            ("y", +1, "right"), ("y", -1, "left")]
    base = {}
    for ax, sgn, key in need:
        base[(ax, sgn)] = poses[key]

    # 零偏：三对的中点求平均
    b = np.mean([0.5 * (base[(ax, +1)] + base[(ax, -1)]) for ax in "xyz"], axis=0)

    # 加速度计静止时测的是【比力】f = a − g，方向与重力相反：
    #     a_raw = −g·F⁻¹·d + b        （d = 该面朝下时重力在机体系的单位方向）
    # 所以 (a₊ − a₋)/2 = −g·F⁻¹·d，要取负号。
    cols = {}
    for ax in "xyz":
        cols[ax] = -0.5 * (base[(ax, +1)] - base[(ax, -1)])   # = g·F⁻¹·e_ax

    M = np.column_stack([cols["x"], cols["y"], cols["z"]])    # = g·F⁻¹

    # 符号消歧：每一列的 ± 在数学上都可以翻，但物理上 F 必须是「接近单位阵」的
    # 小修正（尺度≈1、不正交≈1°）。所以穷举 8 种符号组合，取最接近 +I 的那个。
    best = None
    for sx in (+1, -1):
        for sy in (+1, -1):
            for sz in (+1, -1):
                Ms = M * np.array([sx, sy, sz])
                if abs(np.linalg.det(Ms)) < 1e-9:
                    continue
                Fs = g * np.linalg.inv(Ms)
                cost = np.abs(Fs - np.eye(3)).max()
                if np.any(np.diag(Fs) <= 0):
                    continue
                if best is None or cost < best[0]:
                    best = (cost, Fs, Ms, (sx, sy, sz))
    if best is None:
        F = g * np.linalg.inv(M)
        sign = (1, 1, 1)
    else:
        _, F, M, sign = best
    # 正交性自检：三个向量本应互相垂直、且模长都等于 g
    norms = np.linalg.norm(M, axis=0)
    angs = {}
    for a1, a2 in (("x", "y"), ("x", "z"), ("y", "z")):
        u1, u2 = cols[a1] / norms["xyz".index(a1)], cols[a2] / norms["xyz".index(a2)]
        angs[f"{a1}{a2}"] = 90.0 - np.degrees(np.arccos(np.clip(u1 @ u2, -1, 1)))
    info = {"axis_norms": norms, "off90": angs,
            "sign": locals().get("sign", (1, 1, 1))}
    return F, b, info


# --------------------------------------------------------------------------
# 4. 报告
# --------------------------------------------------------------------------
def misalignment_deg(F: np.ndarray) -> np.ndarray:
    """F 去掉对角尺度后，非对角项对应的角度（度）。"""
    D = np.diag(np.diag(F))
    E = np.linalg.solve(D, F) - np.eye(3)   # 归一化后的不正交
    ang = np.degrees(np.arcsin(np.clip(E, -1, 1)))
    np.fill_diagonal(ang, 0.0)
    return ang


def report(F, b, acc_static, g, groups):
    raw_n = np.linalg.norm(acc_static, axis=1)
    cor_n = np.linalg.norm((acc_static - b) @ F.T, axis=1)
    print(f"\n{'='*66}")
    print(f"姿态数: {len(groups)}")
    for i, gp in enumerate(groups):
        u = gp["acc"] / np.linalg.norm(gp["acc"])
        print(f"  姿态 {i+1}: 采样 {gp['n']:6d} 个｜"
              f"方向 ({u[0]:+.2f},{u[1]:+.2f},{u[2]:+.2f})｜"
              f"|a| = {np.linalg.norm(gp['acc']):.4f} m/s²")

    print(f"\n标定前 |a| : 均值 {raw_n.mean():.4f}  std {raw_n.std():.4f}  "
          f"范围 [{raw_n.min():.4f}, {raw_n.max():.4f}]")
    print(f"标定后 |a| : 均值 {cor_n.mean():.4f}  std {cor_n.std():.4f}  "
          f"范围 [{cor_n.min():.4f}, {cor_n.max():.4f}]")
    print(f"            （真值 g = {g:.4f}；std 从 {raw_n.std():.4f} 降到 "
          f"{cor_n.std():.4f}，降低 {100*(1-cor_n.std()/max(raw_n.std(),1e-12)):.0f}%）")

    diag = np.diag(F)
    print(f"\nF 对角（尺度因子）/ 偏差: " +
          "  ".join(f"{d:.5f} ({(d-1)*100:+.3f}%)" for d in diag))
    print(f"三轴不正交（度）:\n{misalignment_deg(F)}")
    print(f"零偏 b (m/s²): {b[0]:+.4f} {b[1]:+.4f} {b[2]:+.4f}")

    # 泄漏到水平轴的重力加速度估计
    E = np.linalg.solve(np.diag(diag), F) - np.eye(3)
    leak = g * np.linalg.norm(E - np.diag(np.diag(E)))
    print(f"\n重力泄漏量级 ≈ {leak:.3f} m/s²"
          f"（{leak:.3f} m/s² × 20 s 积分 ≈ {0.5*leak*400:.1f} m 位置误差）")
    if cor_n.std() < 0.02 and raw_n.std() > 0.05:
        print("判定: ✅ 有明显的确定性误差，标定有效")
    elif raw_n.std() > 0.05:
        print("判定: ⚠️ 标定改善了分散度，但没到 0.02 以内，检查姿态覆盖")
    else:
        print("判定: ℹ️ 标定前 |a| 已经很集中（<0.05），这个 IMU 的确定性误差本来就小")


def emit_yaml(F, b):
    Ta = np.linalg.inv(F)
    print("\n" + "=" * 66)
    print("粘进 vio_online/config/gemini_1920x1080/kalibr_imu_chain.yaml（替换这几项）：\n")
    print("  R_IMUtoACC:")
    for r in np.eye(3):
        print(f"    - [{r[0]:.1f}, {r[1]:.1f}, {r[2]:.1f}]")
    print("  Ta:            # = inv(F)，即 a_corrected = F·(a_raw − b)")
    for r in Ta:
        print(f"    - [{r[0]:.10f}, {r[1]:.10f}, {r[2]:.10f}]")
    print("  Tg:")
    for _ in range(3):
        print("    - [0.0, 0.0, 0.0]")
    print("\n注意：零偏 b 不要写进配置 —— 滤波器的 bias_a 状态会自己估，")
    print("      写死反而会和它的估计打架。这里给出来只是让你知道量级。")


# --------------------------------------------------------------------------
# 5. 自检
# --------------------------------------------------------------------------
def self_test() -> int:
    print("=== 自检：合成已知的尺度/不正交，看能不能还原 ===")
    rng = np.random.default_rng(42)
    g = G_STD
    # 真值：1.8% 尺度差 + 1.5° 不正交
    th = np.radians(1.5)
    F_true = np.array([[1.018, np.tan(th), -np.tan(th) * 0.5],
                       [0.0, 0.985, np.tan(th)],
                       [0.0, 0.0, 1.012]])
    b_true = np.array([0.12, -0.08, 0.05])

    # 12 个朝向：±x/±y/±z + 6 个带明显倾斜的。
    # 注意：**6 个轴对齐的朝向对椭球拟合是不足的**（6 个点定不出 9 个参数），
    # 必须带倾斜、且数量 ≥10，否则拟合出来的 F 是"能穿过这些点"的任意一个。
    dirs = []
    for ax in range(3):
        for sgn in (+1, -1):
            v = np.zeros(3)
            v[ax] = sgn
            dirs.append(v)
    for _ in range(6):
        v = rng.normal(0, 1, 3)
        dirs.append(v / np.linalg.norm(v))
    acc = []
    for v in dirs:
        v = v + rng.normal(0, 0.12, 3)      # ~7° 的真实手持摆放倾斜
        v /= np.linalg.norm(v)
        a_true = g * v                       # 传感器系里的真实比力
        # 反解 raw：a_true = F(a_raw − b) ⇒ a_raw = F⁻¹ a_true + b
        a_raw = np.linalg.solve(F_true, a_true) + b_true
        for _ in range(400):                 # 每姿态 400 个样本（2 s @200Hz）
            acc.append(a_raw + rng.normal(0, 0.02, 3))
    acc = np.array(acc)

    b0, P = fit_algebraic(acc)
    F0 = ellipsoid_to_F(P, g)
    F, b = refine(F0, b0, acc, g)

    raw_n = np.linalg.norm(acc, axis=1)
    cor_n = np.linalg.norm((acc - b) @ F.T, axis=1)

    # 评价：F 带一个旋转自由度（F 和 R·F 给出同一个椭球），
    # 所以不能逐个元素比 F。可观测的只有 FᵀF，归一化掉整体尺度后比较。
    E = F.T @ F
    E_true = F_true.T @ F_true
    E = E / (np.linalg.det(E) ** (1 / 3))
    E_true = E_true / (np.linalg.det(E_true) ** (1 / 3))
    dE = E - E_true
    # 用"等效角度"表达：非对角项对应轴间不正交
    err_scale = np.abs(np.diag(dE)).max() * 100
    err_ang = np.degrees(np.abs(dE - np.diag(np.diag(dE))).max())

    # 残差只看"每个姿态的平均"，不看单样本噪声（单样本噪声本来就该保留）
    groups_mean = acc.reshape(len(dirs), -1, 3).mean(axis=1)
    g_raw = np.linalg.norm(groups_mean, axis=1)
    g_cor = np.linalg.norm((groups_mean - b) @ F.T, axis=1)

    print(f"  各姿态 |a| std: 标定前 {g_raw.std():.5f} → 标定后 {g_cor.std():.5f}"
          f"   （目标 < 0.01）")
    print(f"  尺度还原误差 = {err_scale:.3f}%   （目标 < 0.3%）")
    print(f"  不正交还原误差 = {err_ang:.3f}°      （目标 < 0.2°）")
    print(f"  零偏还原误差 = {np.abs(b - b_true).max():.4f} m/s²")

    # ---- Ferraris 六面法（只摆 6 个面时必须用它）----
    # 用「正好轴对齐」的 6 个面（无倾斜）合成数据
    faces = {}
    for ax, sgn, key in (("z", +1, "belly"), ("z", -1, "back"), ("x", +1, "nose"),
                         ("x", -1, "tail"), ("y", +1, "right"), ("y", -1, "left")):
        d = np.zeros(3)
        d["xyz".index(ax)] = sgn
        a_true = g * d
        a_raw = np.linalg.solve(F_true, a_true) + b_true
        faces[key] = a_raw + rng.normal(0, 0.002, 3)
    Ff, bf, info = fit_ferraris(faces, g)
    Ef = Ff.T @ Ff
    Ef = Ef / (np.linalg.det(Ef) ** (1 / 3))
    dEf = Ef - E_true
    f_scale = np.abs(np.diag(dEf)).max() * 100
    f_ang = np.degrees(np.abs(dEf - np.diag(np.diag(dEf))).max())
    f_cor = np.linalg.norm((np.array(list(faces.values())) - bf) @ Ff.T, axis=1)
    print("\n  --- Ferraris 六面法（只摆 6 个面）---")
    print(f"  尺度还原误差 = {f_scale:.3f}%")
    print(f"  不正交还原误差 = {f_ang:.3f}°")
    print(f"  零偏还原误差 = {np.abs(bf - b_true).max():.4f} m/s²")
    print(f"  六面正交性自检（偏离 90° 的量，理想摆放应 < 0.1°）: "
          + "  ".join(f"{k} {v:+.3f}°" for k, v in info["off90"].items()))
    ok_f = f_scale < 0.5 and f_ang < 0.3

    ok = g_cor.std() < 0.01 and err_scale < 0.3 and err_ang < 0.2 and ok_f
    print("自检", "通过" if ok else "失败")
    if not ok:
        print(f"  [调试] F=\n{F}\nF_true=\n{F_true}")
    return 0 if ok else 1


# --------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description="加速度计尺度/不正交标定（六姿态法）")
    ap.add_argument("--csv", help="imu CSV（record_static_imu.py 的输出）")
    ap.add_argument("--session", help="也可以直接给相机采集目录（读里面的 imu.csv）")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--g", type=float, default=G_STD, help=f"当地重力（默认 {G_STD}）")
    ap.add_argument("--max-gyro", type=float, default=0.05, help="判定静止的角速度上限 rad/s")
    ap.add_argument("--min-static-s", type=float, default=4.0, help="每个姿态最少静止几秒")
    ap.add_argument("--mode", choices=("auto", "ellipsoid", "ferraris"), default="auto",
                    help="ellipsoid=≥10 姿态且互信（默认优先）；ferraris=只摆 6 个面时用")
    ap.add_argument("--order", default=None,
                    help="ferraris 模式下 6 段静止的摆放顺序，逗号分隔，"
                         "可选 belly,back,nose,tail,right,left（默认就是它）")
    ap.add_argument("--save", help="把结果存成 npz")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    if args.session:
        cand = Path(args.session).expanduser() / "imu.csv"
        if not cand.exists():
            raise SystemExit(f"{cand} 不存在")
        path = cand
    elif args.csv:
        path = Path(args.csv).expanduser()
    else:
        ap.error("要么 --csv / --session，要么 --self-test")
    if not path.exists():
        raise SystemExit(f"找不到 {path}")

    acc, gyr, t = load_imu(path)
    print(f"读入 {path}: {len(t)} 样本, {t[-1]:.1f} s, "
          f"{len(t)/max(t[-1],1e-9):.0f} Hz")

    runs = find_static_runs(acc, gyr, t, max_gyro=args.max_gyro,
                            min_len_s=args.min_static_s)
    print(f"找到 {len(runs)} 段静止")
    groups = cluster_orientations(runs)
    print(f"合并成 {len(groups)} 个姿态")
    use_ferraris = args.mode == "ferraris" or (args.mode == "auto" and len(groups) < 10)
    if use_ferraris and args.mode != "ellipsoid":
        order = (args.order or "belly,back,nose,tail,right,left").split(",")
        order = [o.strip().lower() for o in order if o.strip()]
        if len(order) != 6:
            print(f"\n❌ --order 需要 6 个，现在 {len(order)} 个")
            return 2
        runs_sorted = sorted(runs, key=lambda r: r["t0"])
        if args.mode == "auto" and len(runs_sorted) == 6 and len(groups) != 6:
            pass
        if len(runs_sorted) != 6:
            print(f"\n❌ 六面法需要正好 6 段静止，检测到 {len(runs_sorted)} 段。")
            print("   请：每摆好一个面就静止 10~15 秒，中间不要停顿，共 6 段。")
            print("   相邻两个面之间要动起来（>1 秒），否则工具会把它们并成一段。")
            return 2
        poses, n_by_name = {}, {}
        for name, r in zip(order, runs_sorted):
            if name not in FACE_ALIASES:
                print(f"\n❌ 认不出朝向名 '{name}'，可选: {sorted(FACE_ALIASES)}")
                return 2
            poses[name] = r["acc"]
            n_by_name[name] = r["n"]
        try:
            F, b, info = fit_ferraris(poses, args.g)
        except np.linalg.LinAlgError:
            print("\n❌ 六面法矩阵奇异 —— 检查是不是有两个面摆反/重复了")
            return 2
        names = {"x": "鼻子/尾巴", "y": "右/左", "z": "肚子/背"}
        print(f"\n六面法：三段差分向量的模长（应都 ≈ g = {args.g:.3f}）")
        for i, nm in enumerate("xyz"):
            print(f"  {nm} 轴（{names[nm]}）: {info['axis_norms'][i]:.4f}")
        print(f"正交性（偏离 90°，理想摆放应 <0.5°）: "
              + "  ".join(f"{k} {v:+.3f}°" for k, v in info["off90"].items()))
        if max(abs(v) for v in info["off90"].values()) > 2.0:
            print("⚠️ 正交性偏差偏大：某几个面没摆正，结果可信度下降")
        groups_out = [{"acc": poses[k], "n": n_by_name[k], "n_runs": 1} for k in order]
        report(F, b, np.array(list(poses.values())), args.g, groups_out)
        emit_yaml(F, b)
        if args.save:
            np.savez(args.save, F=F, b=b, g=args.g, Ta=np.linalg.inv(F))
        return 0

    cond, s9 = conditioning(groups)
    print(f"姿态覆盖条件数 = {cond:.2f}（第 9 奇异值 {s9:.4f}）")
    if len(groups) < 10 or not np.isfinite(cond) or cond > 30.0:
        print(f"\n❌ 椭球法姿态覆盖不足（条件数 {cond:.2f}，要求 < 30）。")
        print("   椭球有 9 个参数，至少要 10 个不同朝向。")
        print("   如果只能摆 6 个面，改用： --mode ferraris --order "
              "belly,back,nose,tail,right,left")
        return 2

    if len(groups) < 8:
        print(f"\n❌ 姿态不够（{len(groups)} < 8）。")
        print("   注意：**6 个轴对齐的朝向对椭球拟合是不足的**——6 个点定不出椭球的")
        print("   9 个参数，会拟合出一个'恰好穿过这些点'的错误矩阵。本工具自检里")
        print("   6 姿态时尺度误差 0.15%、不正交误差 2.9°；12 姿态带倾斜时分别是")
        print("   0.006% 和 0.027°。请摆 10~12 个朝向，每个带点倾斜、各静止")
        print(f"   {args.min_static_s:.0f} s 以上重录。")
        return 2

    A = np.array([gp["acc"] for gp in groups])
    try:
        b0, P = fit_algebraic(A)
        F0 = ellipsoid_to_F(P, args.g)
    except Exception as exc:
        print(f"\n❌ 椭球拟合失败：{exc}")
        return 2
    F, b = refine(F0, b0, A, args.g)

    report(F, b, A, args.g, groups)
    emit_yaml(F, b)

    if args.save:
        np.savez(args.save, F=F, b=b, g=args.g, Ta=np.linalg.inv(F))
        print(f"\n已保存 {args.save}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
