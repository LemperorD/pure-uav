#!/usr/bin/env python3
"""Board-based monocular camera–IMU extrinsic/time calibration; no ROS runtime."""
import argparse
import json
import sys
from dataclasses import asdict, fields
from pathlib import Path

from calib.data import CalibrationError, Camera, Target, load_dataset, read_yaml
from calib.solver import Problem, save_result
from calib.acquisition import acquisition_preflight
from calib.windows import describe_problem, diagnostics_from_extraction, visual_diagnostics
from extrinsics_config import (CAM_TOPIC, IMAGE_TIMESTAMP_UNIT, IMU_TIMESTAMP_UNIT,
                               IMU_TOPIC, PATHS, SETTINGS, Settings)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--camera", type=Path, default=PATHS["camera"])
    p.add_argument("--camera-name", default="cam0")
    p.add_argument("--target", type=Path, default=PATHS["target"])
    p.add_argument("--images", type=Path, default=PATHS["images"], help="timestamp,filename 图像清单")
    p.add_argument("--imu", type=Path, default=PATHS["imu"])
    p.add_argument("--bag", type=Path, default=PATHS["bag"], help="ROS1 .bag 或 ROS2 bag 目录，使用 header.stamp；仍需核实时钟域")
    p.add_argument("--output", type=Path, default=PATHS["output"])
    p.add_argument("--cam-topic", default=CAM_TOPIC)
    p.add_argument("--imu-topic", default=IMU_TOPIC)
    p.add_argument("--image-unit", choices=["s", "ms", "us", "ns"], default=IMAGE_TIMESTAMP_UNIT)
    p.add_argument("--imu-unit", choices=["s", "ms", "us", "ns"], default=IMU_TIMESTAMP_UNIT)
    p.add_argument("--settings", type=Path, help="覆盖 Settings 的 YAML 文件，未知参数报错")
    p.add_argument("--acquisition", type=Path, help="可选：采集滤波链/时钟/实例声明 YAML；未提供时明确警告")
    p.add_argument("--start", type=float)
    p.add_argument("--duration", type=float)
    p.add_argument("--min-duration", type=float, help="连续窗口最短时长（覆盖 Settings.min_duration_s）")
    p.add_argument("--detect-rate", type=float, help="检测抽样率 Hz；0=每帧都检测（默认）")
    p.add_argument("--no-auto-window", action="store_true",
                   help="关闭自动选窗：检出有空洞就直接拒绝整段（老行为）")
    p.add_argument("--knot-spacing", type=float)
    p.add_argument("--max-nfev", type=int)
    p.add_argument("--min-tags", type=int, help="AprilGrid 每帧最少 tag 数（覆盖 Settings.min_tags）；3x3 小板要调低")
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    # Never reuse a result directory: an unsuccessful run must not leave stale
    # calibration YAMLs that could be mistaken for a new successful result.
    if args.output.exists() and any(args.output.iterdir()):
        print(f"失败：输出目录非空 {args.output}；请使用新的目录。", file=sys.stderr)
        return 2
    args.output.mkdir(parents=True, exist_ok=True)
    acquisition = None
    diagnostics = None  # 写进失败报告：不能只留一句"大空洞"而没有证据
    window = None
    settings = None
    try:
        params = asdict(SETTINGS)
        if args.settings:
            update = read_yaml(args.settings)
            if not isinstance(update, dict):
                raise CalibrationError("Settings YAML 顶层必须为配置项映射。")
            unknown = set(update)-{f.name for f in fields(Settings)}
            if unknown:
                raise CalibrationError(f"未知配置项 {sorted(unknown)}")
            params.update(update)
        for arg, key in [("start", "start_s"), ("duration", "max_duration_s"),
                         ("min_duration", "min_duration_s"), ("detect_rate", "detect_rate_hz"),
                         ("knot_spacing", "knot_spacing_s"), ("max_nfev", "joint_max_nfev"),
                         ("min_tags", "min_tags")]:
            if getattr(args, arg) is not None:
                params[key] = getattr(args, arg)
        if args.no_auto_window:
            params["auto_window"] = 0
        settings = Settings(**params)
        acquisition = acquisition_preflight(args.acquisition)
        declared_topic = acquisition["operator_declarations"].get("imu_topic")
        if args.bag and declared_topic and declared_topic != args.imu_topic:
            raise CalibrationError("采集说明 imu_topic 与实际 --imu-topic 不一致；请核查说明属于哪条数据链。")
        for warning in acquisition["warnings"]:
            print(f"采集警告：{warning}", file=sys.stderr, flush=True)
        camera = Camera.load(args.camera, args.camera_name)
        target = Target(read_yaml(args.target), min_tags=settings.min_tags, april_corner_window_px=settings.april_corner_window_px)
        print("读取数据并检测标定板……", flush=True)
        observations, tc, ti, imu, epoch = load_dataset(
            camera, target, settings, args.output, images=args.images, imu=args.imu, bag=args.bag,
            image_unit=args.image_unit, imu_unit=args.imu_unit, cam_topic=args.cam_topic, imu_topic=args.imu_topic)
        extraction = json.loads((args.output/"extraction.json").read_text(encoding="utf-8"))
        diagnostics = visual_diagnostics(tc, extraction, max_gap_s=settings.max_camera_gap_s,
                                         min_frames=settings.min_frames,
                                         min_duration_s=settings.min_duration_s)
        print(f"有效图像 {len(observations)}，检测时间跨度 {tc[-1]-tc[0]:.3f}s", flush=True)
        advice = describe_problem(diagnostics, settings)
        for line in advice:
            print(f"检出诊断：{line}", file=sys.stderr, flush=True)
        if settings.auto_window:
            best = diagnostics["best_run"]
            if best is None:
                runs = diagnostics["runs"]
                detail = (f"最长连续段 {runs[0]['duration_s']:.2f}s（{runs[0]['frames']} 帧）" if runs
                          else "一帧都没连上")
                raise CalibrationError(
                    f"没有满足门槛的连续窗口：{detail}；要求 ≥{settings.min_duration_s:g}s、"
                    f"≥{settings.min_frames} 帧、相邻有效帧间隔 ≤{settings.max_camera_gap_s:g}s"
                    "（原因见上面的检出诊断，或报告里的 visual_diagnostics）。")
            starts, stops = best["start_index"], best["stop_index"]
            if starts > 0 or stops < len(observations)-1:
                window = {"auto_selected": True, "start_s": best["start_s"], "stop_s": best["stop_s"],
                          "duration_s": best["duration_s"], "frames": best["frames"],
                          "dropped_frames": len(observations)-best["frames"],
                          "reason": "检出空洞超过 max_camera_gap_s，只拟合最长连续段"}
                print(f"自动选窗：{best['start_s']:.3f}s→{best['stop_s']:.3f}s"
                      f"（{best['duration_s']:.2f}s，{best['frames']} 帧），丢弃 {window['dropped_frames']} 帧；"
                      f"整段有 {len(diagnostics['gaps_over_limit_s'])} 处超限空洞。", flush=True)
                observations, tc = observations[starts:stops+1], tc[starts:stops+1]
            else:
                window = {"auto_selected": False, "start_s": best["start_s"], "stop_s": best["stop_s"],
                          "duration_s": best["duration_s"], "frames": best["frames"],
                          "dropped_frames": 0, "reason": "整段没有超限空洞"}
        problem = Problem(camera, observations, tc, ti, imu, settings)
        report, x, _ = problem.solve()
        report["acquisition"] = acquisition
        report["visual_diagnostics"] = diagnostics
        report["window"] = window
        if window and window["dropped_frames"]:
            report["limitations"].insert(
                0, f"只拟合了 {window['start_s']:.2f}s→{window['stop_s']:.2f}s 的连续窗口"
                   f"（{window['frames']} 帧）；其余 {window['dropped_frames']} 帧因检出空洞被排除，"
                   "结果不代表整段录像，也不能用整段录像的时长描述这次标定的激励。")
        report["timestamp_origin_ns"] = epoch
        report["inputs"] = {k: str(getattr(args, k)) for k in ("camera", "target", "images", "imu", "bag", "cam_topic", "imu_topic", "acquisition")}
        save_result(args.output, camera, report, x, problem)
        if report["failure_reasons"]:
            raise CalibrationError("；".join(report["failure_reasons"]))
        print(json.dumps({k: report[k] for k in ("status", "T_cam_imu", "timeshift_cam_imu", "final_metrics")}, indent=2, ensure_ascii=False))
        print(f"标定通过残差/激励检查；文件保存至 {args.output.resolve()}")
        return 0
    except (CalibrationError, ValueError, KeyError, OSError, TypeError) as exc:
        report_path = args.output / "report.json"
        # 中途失败也要给证据：从 extraction.json 重建"哪些帧被接受、空洞在哪、
        # 是不是糊了"。否则用户只能看到一句结论，不知道下一步该改什么。
        if diagnostics is None and settings is not None:
            try:
                extraction = json.loads((args.output/"extraction.json").read_text(encoding="utf-8"))
                diagnostics = diagnostics_from_extraction(
                    extraction, max_gap_s=settings.max_camera_gap_s,
                    min_frames=settings.min_frames, min_duration_s=settings.min_duration_s)
                for line in describe_problem(diagnostics, settings) if diagnostics else []:
                    print(f"检出诊断：{line}", file=sys.stderr, flush=True)
            except (OSError, ValueError, KeyError):
                diagnostics = None
        if not report_path.exists():
            stub = {"status": "failed", "failure_reasons": [str(exc)], "acquisition": acquisition}
            if diagnostics is not None:
                stub["visual_diagnostics"] = diagnostics
                stub["window"] = window
            report_path.write_text(json.dumps(stub, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"标定失败：{exc}\n诊断：{report_path}", file=sys.stderr)
        return 2



if __name__ == "__main__":
    sys.exit(main())
