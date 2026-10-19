#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
着陆段数值审计：检查「停螺旋桨之前」的运动状态是否合理。

【为什么要单独做】
  现有的 `test_phase_b_states.FakePX4` 在进入 landing_phase 的瞬间把水平速度
  **硬置零**（`self.v = [0.0, 0.0, -vz]`）—— 这会**掩盖**真实问题：
  实飞里无人机在脱靶时还有 ~5.8 m/s 的水平速度，PX4 需要**时间与距离**才能把它刹住。
  本脚本用一个更真实的模型（水平按有限加速度减速 + 垂直按指令下降），
  把着陆全程打出来，并检查停桨时刻的状态。

【检查项】
  A. 下降过程是否平滑（无掉高/冲击）
  B. **停桨时刻的离地高度**（>0.5 m 就是硬摔）
  C. **停桨时刻的水平速度**（>0.3 m/s 就是带速度落地，会侧翻）
  D. 水平漂移距离（决定"跑完停在哪儿"）
  E. 从 LAND 到 DISARM 的耗时是否合理

用法：python audit_landing.py [libpure_guidance.so|pure_guidance.dll]
"""
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
sys.path.insert(0, HERE)
sys.argv = ["audit"]

spec = importlib.util.spec_from_file_location("tpb", "test_phase_b_states.py")
TPB = importlib.util.module_from_spec(spec)
spec.loader.exec_module(TPB)          # 只建桩，不跑 main


# --------------------------------------------------------------------------
#  更真实的着陆模型
# --------------------------------------------------------------------------
class RealLandingFakePX4(TPB.FakePX4):
    """在 TPB.FakePX4 基础上，**着陆阶段不硬置零水平速度**。

    真实 PX4 在 AUTO_LAND 里会用位置控制器把水平速度刹到 0 —— 这是有限加速度的，
    所以会**边降边飘**一段距离。这里用一个可配的减速度近似：
      · 水平：以 `a_h_max` 减速（默认 2 m/s²，AUTO_LAND 的水平限幅通常不大）
      · 垂直：按节点下发的 `land_down_speed` 匀速下降
      · 触地（alt ≤ gear）后：水平继续摩擦减速，直到 |v_h| ≤ 0.1 才真正静止
    """

    def __init__(self, node, a_h_max=2.0, gear_h=0.20, **kw):
        super().__init__(node, **kw)
        self.a_h_max = a_h_max
        self.gear_h = gear_h
        self.land_log = []

    def advance(self):
        n = self.n
        if not getattr(n, "landing_phase", False):
            return super().advance()

        dt = self.dt
        # ---- 水平：有限减速 ----
        vh = [self.v[0], self.v[1]]
        mag = (vh[0] ** 2 + vh[1] ** 2) ** 0.5
        if mag > 1e-9:
            dv = min(self.a_h_max * dt, mag)
            vh[0] -= dv * vh[0] / mag
            vh[1] -= dv * vh[1] / mag
        self.v[0], self.v[1] = vh
        self.p[0] += vh[0] * dt
        self.p[1] += vh[1] * dt

        # ---- 垂直 ----
        if not self.on_ground:
            vd = n.args.land_down_speed          # m/s，向下
            self.v[2] = -vd
            self.p[2] -= vd * dt
            if self.p[2] <= self.gear_h:
                self.p[2] = self.gear_h
                self.on_ground = True
        else:
            self.p[2] = self.gear_h
            self.v[2] = 0.0

        self.a = [0.0, 0.0, 0.0]
        self.sync()
        self.land_log.append(dict(t=TPB.VT[0], alt=self.p[2],
                                  vx=self.v[0], vy=self.v[1], vz=self.v[2],
                                  on_ground=self.on_ground,
                                  armed=(n.arming_state == 2)))


def run_case(lib, law, name, p0=(0.0, 0.0, 10.0), a_h_max=2.0, **kw):
    print()
    print("=" * 78)
    print(f"审计用例：{name}（law={law}，着陆水平减速度 {a_h_max} m/s²）")
    print("=" * 78)
    n = TPB.new_node(lib, law=law, **kw)
    n.log = lambda s: None
    n.warn = lambda s: None
    fx = RealLandingFakePX4(n, a_h_max=a_h_max, p_enu=p0)

    # 跑到收尾为止
    disarm_snap = None
    land_start = None
    for _ in range(20000):
        prev_armed = (n.arming_state == 2)
        TPB.step(n, fx, 1)
        if land_start is None and n.landing_phase:
            land_start = dict(px=fx.p[0], py=fx.p[1], alt=fx.p[2],
                              vh=(fx.v[0] ** 2 + fx.v[1] ** 2) ** 0.5,
                              t=TPB.VT[0])
        # ⚠ 必须先记停桨再判 DONE —— tick() 会在同一帧里把状态推到 DONE
        if disarm_snap is None and prev_armed and n.arming_state == TPB.M.ARMING_STATE_DISARMED:
            disarm_snap = dict(alt=fx.p[2], vx=fx.v[0], vy=fx.v[1], vz=fx.v[2],
                               px=fx.p[0], py=fx.p[1], on_ground=fx.on_ground,
                               t=TPB.VT[0], state=n.state)
        if n.state == "DONE":
            break

    # ---- 打印着陆段抽样 ----
    log = fx.land_log
    if log:
        print(f"  着陆段共 {len(log)} 帧（{len(log) * 0.02:.1f} s @ 50 Hz），抽样：")
        step = max(1, len(log) // 12)
        print(f"    {'t(s)':>7} {'alt(m)':>8} {'vx':>7} {'vy':>7} {'vz':>7} "
              f"{'|v_h|':>7} {'触地':>5} {'armed':>6}")
        for i in range(0, len(log), step):
            r = log[i]
            print(f"    {r['t']:7.2f} {r['alt']:8.3f} {r['vx']:7.3f} {r['vy']:7.3f} "
                  f"{r['vz']:7.3f} {(r['vx']**2 + r['vy']**2)**0.5:7.3f} "
                  f"{'是' if r['on_ground'] else '否':>5} {'是' if r['armed'] else '否':>6}")
        r = log[-1]
        print(f"    {r['t']:7.2f} {r['alt']:8.3f} {r['vx']:7.3f} {r['vy']:7.3f} "
              f"{r['vz']:7.3f} {(r['vx']**2 + r['vy']**2)**0.5:7.3f} "
              f"{'是' if r['on_ground'] else '否':>5} {'是' if r['armed'] else '否':>6}"
              f"   ← 末帧")

    # ---- 判定 ----
    print()
    print("  ── 停桨时刻状态 ──")
    if disarm_snap is None:
        print("    ✗ 未观察到停桨")
        return None
    vh = (disarm_snap["vx"] ** 2 + disarm_snap["vy"] ** 2) ** 0.5
    print(f"    离地高度 alt = {disarm_snap['alt']:.3f} m")
    print(f"    水平速度 |v_h| = {vh:.3f} m/s")
    print(f"    垂直速度 vz = {disarm_snap['vz']:+.3f} m/s（NED，+ = 向下）")
    print(f"    已触地 = {disarm_snap['on_ground']}    触发时状态 = {disarm_snap['state']}")
    if land_start:
        drift = ((disarm_snap["px"] - land_start["px"]) ** 2
                 + (disarm_snap["py"] - land_start["py"]) ** 2) ** 0.5
        print(f"    着陆起点 ({land_start['px']:+.2f},{land_start['py']:+.2f}) "
              f"alt={land_start['alt']:.2f} |v_h|={land_start['vh']:.2f} m/s"
              f"  →  停桨 ({disarm_snap['px']:+.2f},{disarm_snap['py']:+.2f})")
        print(f"    水平漂移 {drift:.2f} m  耗时 {disarm_snap['t'] - land_start['t']:.1f} s")
    else:
        drift = None

    print()
    print("  ── 合理性判定 ──")
    ok_alt = disarm_snap["alt"] <= 0.35
    ok_vh = vh <= 0.30
    ok_vz = abs(disarm_snap["vz"]) <= 0.15
    print(f"    [{'✓' if ok_alt else '✗'}] 停桨高度 ≤0.35 m（起落架高度量级） "
          f"→ 实测 {disarm_snap['alt']:.3f} m")
    print(f"    [{'✓' if ok_vh else '✗'}] 停桨水平速度 ≤0.30 m/s（不会侧翻） "
          f"→ 实测 {vh:.3f} m/s")
    print(f"    [{'✓' if ok_vz else '✗'}] 停桨垂直速度 ≤0.15 m/s（不是砸下来的） "
          f"→ 实测 {abs(disarm_snap['vz']):.3f} m/s")
    n.close()
    return dict(alt=disarm_snap["alt"], vh=vh, vz=disarm_snap["vz"],
                on_ground=disarm_snap["on_ground"], n_land=len(log),
                drift=drift, start_alt=(land_start or {}).get("alt"))


def main():
    lib = TPB.find_lib(sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else None)
    print("=" * 78)
    print("着陆段数值审计（真 C++ 制导核 + 更真实的着陆模型）")
    print(f"制导核: {lib}")
    print("=" * 78)

    res = {}
    # pilot 守备场景：PPN 会脱靶，APN 会命中 —— 两种收尾都看
    pilot = dict(target_p0=(-14.0, 0.0, 10.0), target_speed=14.0,
                 target_elev=-6.0, target_height=8.0, v_boost_min=5.0,
                 cruise_x=-4.0)
    res["PPN(脱靶)"] = run_case(lib, 0, "pilot · PPN（预期脱靶）",
                                p0=(0.0, 0.0, 8.0), **pilot)
    res["APN(命中)"] = run_case(lib, 2, "pilot · APN（预期命中）",
                                p0=(0.0, 0.0, 8.0), **pilot)
    # 脱靶时水平速度最大 → 最考验着陆；把减速能力调小模拟"刹车弱"
    res["PPN(弱刹车)"] = run_case(lib, 0, "pilot · PPN（水平减速度仅 1.0 m/s²）",
                                  p0=(0.0, 0.0, 8.0), a_h_max=1.0, **pilot)

    print()
    print("=" * 78)
    print("汇总")
    print("=" * 78)
    print(f"  {'用例':<16} {'停桨高度':>10} {'|v_h|':>8} {'|vz|':>8} {'已触地':>7}  结论")
    for k, v in res.items():
        if v is None:
            print(f"  {k:<16} {'—':>10}")
            continue
        verdict = "合理" if (v["alt"] <= 0.35 and v["vh"] <= 0.30) else "⚠ 需关注"
        print(f"  {k:<16} {v['alt']:10.3f} {v['vh']:8.3f} {abs(v['vz']):8.3f} "
              f"{'是' if v['on_ground'] else '否':>7}  {verdict}")
    print("=" * 78)


if __name__ == "__main__":
    main()
