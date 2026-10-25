"""capture_manifest.json → images.csv

`capture_calibration.py` 存图时，给每张图记了一个宿主机墙钟时间戳
（manifest 里的 `host_receive_time_ns`，就是 `time.time_ns()`），
文件名形如 `cam0/1791886005145251904.png`。

标定程序要的是两列的 `images.csv`：`timestamp,filename`，
且 filename 相对该 csv 自己所在的目录。这个文件把两者对接起来，
顺便做几项体检（时间戳是否严格递增、有没有超长空档、文件是否都在）。
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent   # extrinsic/：唯一需要上 sys.path 的路径
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


class ManifestError(RuntimeError):
    pass


def load_manifest(session_dir):
    path = Path(session_dir) / "capture_manifest.json"
    if not path.is_file():
        raise ManifestError(
            f"没找到 {path}。相机那一步没跑成功？"
            "（这个文件由 vendor/capture_calibration.py 在采集结束时写出）")
    data = json.loads(path.read_text(encoding="utf-8"))
    saved = data.get("saved") or []
    if not saved:
        raise ManifestError(f"{path} 里一张图都没有 —— 采集是不是一开始就被中断了？")
    return data, saved


def build(session_dir, out_name="images.csv", log=print):
    """写出 images.csv，返回统计 dict。"""
    session_dir = Path(session_dir)
    data, saved = load_manifest(session_dir)

    rows, missing = [], []
    for entry in saved:
        stamp = int(entry["host_receive_time_ns"])
        name = entry["file"]
        if not (session_dir / name).is_file():
            missing.append(name)
        rows.append((stamp, name))

    if missing:
        raise ManifestError(
            f"manifest 里列了 {len(missing)} 张图，但磁盘上不存在，例如 {missing[:3]}。"
            "别继续 —— 这种数据标出来的外参没有意义。")

    stamps = [s for s, _ in rows]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ManifestError("图像时间戳不是严格递增的 —— manifest 被改过，或者文件名时间戳重复。")

    out = session_dir / out_name
    with out.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["timestamp", "filename"])
        writer.writerows(rows)

    gaps_ms = [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]
    gaps_sorted = sorted(gaps_ms)
    span = (stamps[-1] - stamps[0]) / 1e9
    stats = {
        "images": len(rows),
        "span_s": span,
        "observed_fps": (len(rows) - 1) / span if span > 0 else 0.0,
        "median_gap_ms": gaps_sorted[len(gaps_sorted) // 2] if gaps_sorted else 0.0,
        "max_gap_ms": max(gaps_ms) if gaps_ms else 0.0,
        "gaps_over_100ms": sum(1 for g in gaps_ms if g > 100.0),
        "gaps_over_350ms": sum(1 for g in gaps_ms if g > 350.0),
        "board_detected": sum(1 for e in saved if e.get("board_detected")),
        "csv": str(out),
        "manifest_mode": (data.get("mode") or {}).get("width", "?") ,
        "manifest_resolution": "%sx%s" % ((data.get("mode") or {}).get("width", "?"),
                                          (data.get("mode") or {}).get("height", "?")),
        "manifest_termination": data.get("termination"),
    }
    log(f"写出 {out} —— {stats['images']} 张，跨度 {span:.2f}s，实测 {stats['observed_fps']:.1f} fps")
    return stats


def main():
    from capture import capture_config as cfg

    print(f"会话目录：{cfg.SESSION_DIR}")
    try:
        stats = build(cfg.SESSION_DIR)
    except ManifestError as exc:
        print(f"❌ {exc}")
        return 2
    if stats["gaps_over_350ms"]:
        print(f"⚠️  有 {stats['gaps_over_350ms']} 处图像间隔超过 350 ms —— "
              "标定程序的 max_camera_gap_s 默认就是 0.35 s，"
              "只要这些空档落在你选的窗口里就会整段被拒。"
              "建议改用 configs/probe.yaml（camera_rate_hz=30）并挑一段连续可见的窗口。")
    print("✅ images.csv 就绪")
    return 0


if __name__ == "__main__":
    sys.exit(main())
