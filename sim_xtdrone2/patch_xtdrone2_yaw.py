#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
补丁：修 XTDrone2 `cmd_accel_ned` / `cmd_accel_flu` 强制把无人机转到正北的 bug
================================================================================

【现象】
  一发 `cmd_accel_ned`，无人机就狂转航向，最后锁死在正北（heading → 0°）。
  实测：t=24.1 s heading=85.5° → t=26.1 s 8.2° → t=30.1 s 0.6°。

【根因】
  `xtd2_communication/multirotor_communication.py` 里：

      def cmd_accel_ned_callback(self, msg):
          ...
          cmd.acceleration = [msg.linear.x, msg.linear.y, msg.linear.z]
          #TODO: How about yaw?          ← 官方自己留的坑
          self.cmd = cmd

  只设了 position / velocity / acceleration，**没设 yaw**。
  ROS2 消息里 `float32 yaw` 的默认值是 **0.0**（不是 NaN），于是 PX4 收到
  "航向 = 0（正北）"的 setpoint，就把机头转过去了。

  对照：同文件的 `cmd_vel_ned_callback` 是显式写了 `cmd.yaw = math.nan` 的，
  只有两个 accel 回调漏了。

【修法】
  在这两个回调里补上：
      cmd.yaw = math.nan
      cmd.yawspeed = math.nan
  NaN 表示"不指定"，PX4 会保持当前航向。

【影响面】
  * 阶段 A / B 用世界系（NED）加速度，航向不影响加速度方向 → **不修也能跑**，
    但会有一个几秒的转向瞬态，且机头恒指正北。
  * 阶段 C（相机闭环）**必须修** —— 相机跟着机头走，机头恒指正北就没法看目标。

【用法】
      python3 patch_xtdrone2_yaw.py --dry-run   # 先看会改什么
      python3 patch_xtdrone2_yaw.py             # 真正写入（自动备份 .bak）

  会同时修**源码**和**已安装副本**（后者才是 `ros2 launch` 实际加载的）。
  已打过补丁的会被跳过（幂等）。
"""

import argparse
import os
import shutil
import sys

SRC = os.path.expanduser(
    "~/XTDrone2/src/xtd2_communication/xtd2_communication/multirotor_communication.py")
INST = os.path.expanduser(
    "~/XTDrone2/install/xtd2_communication/lib/python3.12/site-packages/"
    "xtd2_communication/multirotor_communication.py")

MARK = "PATCH(yaw)"
ACCEL_FUNCS = ("cmd_accel_ned_callback", "cmd_accel_flu_callback")


def patch_text(src):
    """在 cmd_accel_*_callback 的 `self.cmd = cmd` 前插入 yaw=NaN。返回 (新文本, 插入次数)。"""
    lines = src.splitlines(keepends=True)
    out, in_accel, inserted = [], False, 0
    for ln in lines:
        if ln.lstrip().startswith("def "):
            in_accel = any(f in ln for f in ACCEL_FUNCS)
        if in_accel and ln.strip() == "self.cmd = cmd":
            ind = ln[:len(ln) - len(ln.lstrip())]
            out.append(f"{ind}cmd.yaw = math.nan       # {MARK}: 不指定航向，保持当前航向\n")
            out.append(f"{ind}cmd.yawspeed = math.nan  # {MARK}\n")
            inserted += 1
            in_accel = False          # 每个函数只插一次
        out.append(ln)
    return "".join(out), inserted


def handle(path, dry):
    print(f"\n— {path}")
    if not os.path.isfile(path):
        print("  跳过：文件不存在")
        return 0
    with open(path, "r", encoding="utf-8") as f:
        src = f.read()
    if MARK in src:
        print("  跳过：已打过补丁")
        return 0
    if "import math" not in src:
        print("  ✗ 中止：文件里没有 `import math`，不认识的版本，请手工检查")
        return 0
    new, n = patch_text(src)
    if n == 0:
        print("  ✗ 中止：没找到 `self.cmd = cmd`（版本可能不同），请手工检查")
        return 0
    print(f"  将插入 {n} 处（预期 2 处）")
    if dry:
        import difflib
        for ln in difflib.unified_diff(
                src.splitlines(), new.splitlines(),
                fromfile="当前", tofile="打补丁后", lineterm="", n=2):
            print("    " + ln)
        return n
    shutil.copy2(path, path + ".bak")
    with open(path, "w", encoding="utf-8") as f:
        f.write(new)
    print(f"  ✓ 已写入（备份 {os.path.basename(path)}.bak）")
    return n


def main():
    ap = argparse.ArgumentParser(description="修 XTDrone2 accel 回调未设 yaw 的 bug")
    ap.add_argument("--dry-run", action="store_true", help="只看会改什么，不写文件")
    args = ap.parse_args()

    print("=" * 72)
    print("XTDrone2 yaw 补丁" + ("（dry-run）" if args.dry_run else ""))
    print("=" * 72)

    total = 0
    for p in (SRC, INST):
        total += handle(p, args.dry_run)

    print()
    if args.dry_run:
        print(f"dry-run 结束，共会插入 {total} 处。去掉 --dry-run 即真正写入。")
    elif total:
        print("完成。**不需要重新 colcon build** —— 已安装副本也改了，直接重跑即可。")
        print("⚠ 如果以后重新 colcon build，源码已打补丁，install 副本也会是打过补丁的。")
        print("⚠ 想回退：把同目录下的 .bak 覆盖回去。")
    else:
        print("没有需要修改的地方。")


if __name__ == "__main__":
    sys.exit(main())
