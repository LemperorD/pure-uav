#!/usr/bin/env python3
"""不插相机也能跑：验证"曝光步进测试"的判据对不对（用假相机）。

真机实测的两件事驱动了这个测试：
  1. 相机输出是 gamma 编码的，2 倍曝光在原始灰度域只有 1.33 倍
     → 判据必须换到 (v/255)**2.4 的线性域，否则会把成功的锁定判成失败；
  2. G_CTRL 读回只证明驱动缓存，不证明固件执行了
     → 只能看图像亮度是否随曝光变化。

三种假相机：
  manual —— 亮度 ∝ 曝光           应该通过
  ae     —— 亮度恒定（AE 还在接管） 应该不通过（重试 3 次后返回 False）
  dark   —— 太暗，比值不可信       应该按"跳过"通过，并注明原因

用法：python3 tools/selftest_exposure_verify.py
"""
from __future__ import annotations

import sys
import types
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "camera_imu_calibration" / "capture" / "vendor"))

import capture_calibration as cc  # noqa: E402


class FakeCap:
    """按当前曝光出图；曝光值由被 monkeypatch 的 ioctl 写进来。"""

    def __init__(self, mode, base_linear=0.02):
        self.mode = mode
        self.base = base_linear
        self.exposure = 100

    def read(self):
        if self.mode == "manual":
            linear = self.base * self.exposure / 100.0
        elif self.mode == "ae":            # 自动曝光：不管写成什么，画面都一个亮度
            linear = self.base
        elif self.mode == "blown":         # 一半画面极亮：加倍曝光会截断
            half = min(1.0, 0.9 * self.exposure / 100.0)
            gray = np.zeros((48, 64), np.uint8)
            gray[:, :32] = int(round((half ** (1 / cc.GAMMA)) * 255))
            gray[:, 32:] = int(round((20e-6 ** (1 / cc.GAMMA)) * 255))
            return np.dstack([gray, gray, gray]), {}
        else:                              # 太暗
            linear = 20e-6
        gray = np.clip((max(linear, 0.0) ** (1.0 / cc.GAMMA)) * 255.0, 0, 255).astype(np.uint8)
        return np.dstack([gray, gray, gray]), {}


def install_patches(cap):
    """把 ioctl / 读回 换成内存里的寄存器，不需要真相机。"""
    state = {"exposure": 100}
    real_ioctl = cc.fcntl.ioctl
    real_get = cc.get_ctrl
    real_checked = cc.set_ctrl_checked

    def fake_get(fd, cid):
        return state["exposure"] if cid == cc.CONTROLS["exposure"] else 0

    def fake_ioctl(fd, request, control):
        if request == cc.VIDIOC_S_CTRL and control.id == cc.CONTROLS["exposure"]:
            state["exposure"] = int(control.value)
            cap.exposure = int(control.value)
        return 0

    def fake_checked(fd, cid, value, name):
        if cid == cc.CONTROLS["exposure"]:
            state["exposure"] = int(value)
            cap.exposure = int(value)

    cc.get_ctrl = fake_get
    cc.fcntl = types.SimpleNamespace(ioctl=fake_ioctl)
    cc.set_ctrl_checked = fake_checked

    def restore():
        cc.get_ctrl = real_get
        cc.set_ctrl_checked = real_checked
        import importlib
        cc.fcntl = importlib.import_module("fcntl")
        cc.fcntl.ioctl = real_ioctl
    return restore


class FakeAutoCap:
    """均匀亮度假相机：mean = scale × 曝光 × 2**(增益/40)，会被 255 截断。"""

    def __init__(self, state, scale):
        self.state = state
        self.scale = scale

    def read(self):
        mean = min(255.0, self.scale * self.state["exposure"] * 2 ** (self.state["gain"] / 40.0))
        v = int(round(mean))
        img = np.full((32, 32, 3), v, np.uint8)
        return img, {}


def install_auto_patches(state):
    real_ioctl = cc.fcntl.ioctl
    real_get = cc.get_ctrl
    real_query = getattr(cc, "query_ctrl", None)
    real_checked = cc.set_ctrl_checked

    def fake_get(fd, cid):
        if cid == cc.CONTROLS["exposure"]:
            return state["exposure"]
        if cid == cc.CONTROLS["gain"]:
            return state["gain"]
        return 0

    def fake_ioctl(fd, request, ctrl):
        if request == cc.VIDIOC_S_CTRL:
            if ctrl.id == cc.CONTROLS["exposure"]:
                state["exposure"] = int(ctrl.value)
            elif ctrl.id == cc.CONTROLS["gain"]:
                state["gain"] = int(ctrl.value)
        return 0

    def fake_query(fd, cid):
        if cid == cc.CONTROLS["exposure"]:
            return {"name": "exposure", "min": 1, "max": 10000, "step": 1,
                    "default": 157, "flags": 0}
        if cid == cc.CONTROLS["gain"]:
            return {"name": "gain", "min": 0, "max": 255, "step": 1,
                    "default": 0, "flags": 0}
        raise OSError("no such control")

    cc.get_ctrl = fake_get
    cc.fcntl = types.SimpleNamespace(ioctl=fake_ioctl)
    cc.query_ctrl = fake_query
    cc.set_ctrl_checked = lambda *a, **k: None

    def restore():
        cc.get_ctrl = real_get
        cc.set_ctrl_checked = real_checked
        if real_query is not None:
            cc.query_ctrl = real_query
        import importlib
        cc.fcntl = importlib.import_module("fcntl")
        cc.fcntl.ioctl = real_ioctl
    return restore


def check_auto_expose() -> int:
    """自动测光：够亮要选中某个增益；太暗要拒绝；光足要收曝光。"""
    print("自动测光 (auto_expose)")
    args = types.SimpleNamespace(fps=30, auto_expose_max_exposure_ms=15.0,
                                 auto_expose_target=100.0, auto_expose_min_mean=45.0,
                                 allow_dark=False, exposure=100, gain=30)
    cases = [
        # (说明, scale, 期望可用?, 期望状态)
        ("正常室内光", 0.2, True, "ok"),
        ("很暗的房间", 0.00317, False, "too_dark"),
        ("光很足", 3.0, True, "ok"),
    ]
    failures = 0
    for label, scale, want_ok, want_status in cases:
        state = {"exposure": 100, "gain": 30}
        restore = install_auto_patches(state)
        try:
            ok, ev = cc.auto_expose(FakeAutoCap(state, scale), object(), args,
                                    log=lambda *_a: None)
        finally:
            restore()
        flag = "OK " if (ok == want_ok and ev.get("status") == want_status) else "✗  "
        if flag.startswith("✗"):
            failures += 1
        print(f"{flag}{label:8s} 可用={ok!s:5s}（期望 {want_ok}） 状态={ev.get('status')}"
              f"（期望 {want_status}）")
        print(f"      选定 曝光 {ev.get('exposure_ms')} ms / 增益 {ev.get('gain')}"
              f" → 灰度 {ev.get('achieved_mean')}（目标 {args.auto_expose_target}）")
        exp_ms = [round(r["exposure"] * 0.1, 1) for r in ev["ladder"]]
        print(f"      梯子 曝光 {exp_ms[:3]} ... 共 {len(ev['ladder'])} 步"
              f"，增益 {[r['gain'] for r in ev['ladder']][:6]} ...")
        print(f"      曝光不超过模糊预算 15 ms: {max(exp_ms) <= 15.0}")
        if max(exp_ms) > 15.0:
            failures += 1
        print()
    return failures


def main() -> int:
    print(f"gamma = {cc.GAMMA}；判据 = 线性域比值 ≥ 1 + 0.5×(曝光倍数-1)")
    print()
    args = types.SimpleNamespace(exposure=100, warmup_frames=10)
    # (假相机, 期望通过?, 期望状态)
    cases = [("manual", True, "passed"), ("ae", False, "failed"),
             ("dark", True, "too_dark"), ("blown", True, "inconclusive")]
    failures = 0
    for mode, want_pass, want_status in cases:
        cap = FakeCap(mode)
        restore = install_patches(cap)
        try:
            passed, evidence = cc.verify_manual_exposure(cap, object(), args)
        finally:
            restore()
        last = evidence["attempts"][-1]
        flag = "OK " if (passed == want_pass and evidence["status"] == want_status) else "✗  "
        if flag.startswith("✗"):
            failures += 1
        ratio = last["linear_ratio"] or 0.0
        raw_ratio = round(ratio ** (1 / cc.GAMMA), 3) if ratio else 0
        print(f"{flag}{mode:6s} 通过={passed!s:5s}（期望 {want_pass}） "
              f"状态={evidence['status']}（期望 {want_status}）")
        print(f"      {evidence['verdict']}")
        print(f"      曝光 {last['target_exposure']}→{last['high_exposure']} "
              f"线性 {last['linear_before']}→{last['linear_after']} "
              f"比值 {last['linear_ratio']}（原始域比值约 {raw_ratio}）")
        print(f"      重试次数 {len(evidence['attempts'])}")
        print()
    print()
    failures += check_auto_expose()
    if failures:
        print(f"❌ {failures} 个用例判错")
        return 1
    print("✅ 全对：曝光步进判据 + 自动测光（正常/太暗/光足）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
