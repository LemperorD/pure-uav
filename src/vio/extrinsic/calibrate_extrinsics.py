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
    p.add_argument("--knot-spacing", type=float)
    p.add_argument("--max-nfev", type=int)
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
                         ("knot_spacing", "knot_spacing_s"), ("max_nfev", "joint_max_nfev")]:
            if getattr(args, arg) is not None:
                params[key] = getattr(args, arg)
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
        print(f"有效图像 {len(observations)}，检测时间跨度 {tc[-1]-tc[0]:.3f}s", flush=True)
        problem = Problem(camera, observations, tc, ti, imu, settings)
        report, x, _ = problem.solve()
        report["acquisition"] = acquisition
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
        if not report_path.exists():
            report_path.write_text(json.dumps({"status": "failed", "failure_reasons": [str(exc)], "acquisition": acquisition}, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"标定失败：{exc}\n诊断：{report_path}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
