#!/usr/bin/env python3
"""Create a separate monocular OpenVINS config from a passed calibration.
Needs a user's existing OpenVINS estimator/IMU-noise config; never modifies it.
This does not run VIO and does not claim any trajectory accuracy.
"""
import argparse
import json
import sys
from pathlib import Path

from calib.data import CalibrationError, read_yaml
from calib.solver import opencv_yaml


def prepare(calibration, template, output, imu_topic=None, cam_topic=None, timeoffset_mode="frozen"):
    calibration, template, output = Path(calibration), Path(template), Path(output)
    report = json.loads((calibration/"report.json").read_text())
    if report.get("status") != "passed_residual_and_excitation_checks":
        raise CalibrationError("标定报告没有通过；拒绝导出 OpenVINS 配置。")
    chain = read_yaml(calibration/"camchain-openvins.yaml")
    if list(chain) != ["cam0"]:
        raise CalibrationError("此导出器需要 cam0 单目配置。")
    estimator = read_yaml(template/"estimator_config.yaml")
    noise_path = template / estimator["relative_config_imu"]
    if not noise_path.is_file():
        raise CalibrationError(f"找不到模板的 IMU 噪声文件 {noise_path}")
    noise = read_yaml(noise_path)
    if not isinstance(noise, dict) or not isinstance(noise.get("imu0"), dict):
        raise CalibrationError("IMU 模板必须包含 imu0 映射。")
    if timeoffset_mode not in ("frozen", "online"):
        raise CalibrationError("timeoffset_mode 必须为 frozen 或 online。")
    # ROS2Visualizer reads chain rostopic AFTER the estimator/ROS parameter.
    # Update both sources, preserving all supplied sensor/noise parameters.
    recorded = report.get("inputs", {})
    bag_topics = recorded if recorded.get("bag") not in (None, "None", "") else {}
    effective_imu = imu_topic if imu_topic is not None else bag_topics.get("imu_topic", noise["imu0"].get("rostopic", estimator.get("topic_imu")))
    effective_cam = cam_topic if cam_topic is not None else bag_topics.get("cam_topic", chain["cam0"].get("rostopic", estimator.get("topic_camera0", "/cam0/image_raw")))
    for label, topic in (("imu_topic", effective_imu), ("cam_topic", effective_cam)):
        if not isinstance(topic, str) or not topic.strip():
            raise CalibrationError(f"{label} 不能为空；请显式指定话题。")
    noise["imu0"]["rostopic"] = effective_imu
    chain["cam0"]["rostopic"] = effective_cam
    if output.exists() and any(output.iterdir()):
        raise CalibrationError("OpenVINS 输出目录非空，请指定新目录。")
    # Frozen mode isolates the offline estimate during evaluation; online mode
    # changes only timeoffset estimation and must be assessed separately.
    estimator.update({"max_cameras": 1, "use_stereo": False,
                      "calib_cam_extrinsics": False, "calib_cam_timeoffset": timeoffset_mode == "online",
                      "calib_cam_intrinsics": False,
                      "relative_config_imucam": "kalibr_imucam_chain.yaml",
                      "relative_config_imu": "kalibr_imu_chain.yaml", "use_mask": False})
    estimator["topic_imu"] = effective_imu
    estimator["topic_camera0"] = effective_cam
    output.mkdir(parents=True, exist_ok=True)
    (output/"kalibr_imu_chain.yaml").write_text(opencv_yaml(noise), encoding="utf-8")
    (output/"kalibr_imucam_chain.yaml").write_text(opencv_yaml(chain), encoding="utf-8")
    (output/"estimator_config.yaml").write_text(opencv_yaml(estimator), encoding="utf-8")
    (output/"PROVENANCE.json").write_text(json.dumps({
        "calibration": str(calibration.resolve()), "template": str(template.resolve()),
        "warning": "IMU noise copied from supplied template. A EuRoC/other sensor template does NOT establish CUAV IMU noise parameters.",
        "offline_calibration_frozen": timeoffset_mode == "frozen",
        "timeoffset_mode": timeoffset_mode,
        "imu_topic": effective_imu, "cam_topic": effective_cam,
        "acquisition": report.get("acquisition", {"verification": "not recorded in source calibration"}),
        "online_timeoffset_warning": "Online estimation does not repair clock drift, unequal frequency-dependent IMU filtering, or rolling shutter.",
        "vio_performance_tested": False}, indent=2), encoding="utf-8")
    return output


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--calibration", type=Path, required=True)
    p.add_argument("--template-config", type=Path, required=True, help="含 estimator_config.yaml 与 IMU 噪声文件的目录")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--imu-topic", help="同时覆盖 estimator 与 imu0.rostopic；默认使用标定 bag 话题或模板话题")
    p.add_argument("--cam-topic", help="同时覆盖 estimator 与 cam0.rostopic")
    p.add_argument("--timeoffset-mode", choices=["frozen", "online"], default="frozen",
                   help="frozen: 隔离评估离线标定；online: 允许在线估计时移，不修复滤波/漂移")
    a = p.parse_args()
    try:
        print(prepare(a.calibration, a.template_config, a.output, a.imu_topic, a.cam_topic, a.timeoffset_mode))
        return 0
    except (CalibrationError, OSError, ValueError, KeyError) as exc:
        print(f"OpenVINS 配置导出失败：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
