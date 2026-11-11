#!/usr/bin/env python3
"""Exercise the actual main config path without opening a camera or serial port."""
import json
import pathlib
import re
import subprocess
import sys
import tempfile

binary = pathlib.Path(sys.argv[1]).resolve()
root = pathlib.Path(sys.argv[2]).resolve()
app_text = (root / "config/app.yaml").read_text()
vio_text = (root / "config/vio.yaml").read_text()
checks = 0


def run(args, expected=0, contains=None):
    global checks
    result = subprocess.run([str(binary), *args], cwd="/tmp", capture_output=True, text=True, timeout=10)
    output = result.stdout + result.stderr
    assert result.returncode == expected, (args, result.returncode, output)
    if contains:
        assert contains in output, (contains, output)
    checks += 1
    return output


run(["--help"], contains="--vio-config")
run(["--help"], contains="PX4")
run(["--check-config"], contains="flight_stack: PX4")
run(["--help"], contains="--check-config")
run(["--help"], contains="--no-detection")
run(["--help"], contains="--mission-config")
run(["--check-config"], contains="mission_enabled: false")
run(["--mission", "--check-config"], contains="mission_enabled: true")
run(["--mission", "--no-send", "--check-config"], contains="mission_send_enabled: false")
run(["--mission", "--guidance-config", "missing-guidance.yaml", "--check-config"], 2, "Configuration error")
run(["--mission", "--mission-config", "missing-mission.yaml", "--check-config"], 2, "Configuration error")
run(["--mission-config"], 2, "缺少参数值")
run(["--help"], contains="--detect-bin")
run(["--no-detection", "--check-config"], contains="detection_enabled: false")
# 校验路径只解析进程参数，不启动子进程或加载检测模型。
run(["--detect-bin", "/not-a-detector", "--detection-config", "/not-a-config", "--detect-arg", "--json", "--check-config"], contains="Configuration OK")
for flag in ("--detect-bin", "--detection-config", "--detect-arg"):
    run([flag], 2, "缺少参数值")
run(["--no-send", "--check-config"], contains="odometry_send_enabled: false")
effective = run(["--check-config"], contains="Configuration OK")
assert str(root / "config/app.yaml") in effective
assert str(root / "config/vio.yaml") in effective
assert "vio.use_depth/use_depth_init: true/true" in effective
assert "vio.init_vel_zero/static_init/online_timeoffset: true/false/true" in effective
run(["--config"], 2, "缺少参数值")
run(["--vio-config"], 2, "缺少参数值")
run(["--unknown"], 2, "未知参数")
with tempfile.TemporaryDirectory(prefix="app-config-", dir=root / "build") as directory:
    tmp = pathlib.Path(directory)

    invalid_mission = tmp / "invalid-mission.yaml"
    for content in ("mission: {typo: 1}", "mission: {takeoff_height_m: .nan}", "mission: {target_confirm_frames: -1}", "mission: {prestream_s: 0.01}"):
        invalid_mission.write_text(content)
        run(["--mission", "--mission-config", str(invalid_mission), "--check-config"], 2, "Configuration error")

    def variant(name, text, old, new, flag, error):
        path = tmp / (name + ".yaml")
        assert old in text
        path.write_text(text.replace(old, new, 1))
        run([flag, str(path), "--check-config"], 2, error)
        return path

    def app_variant(name, old, new, error):
        return variant(name, app_text, old, new, "--config", error)

    def vio_variant(name, old, new, error):
        return variant(name, vio_text, old, new, "--vio-config", error)

    app_variant("bad-loop", "main_loop_hz: 50", "main_loop_hz: 0", "main_loop_hz")
    app_variant("nan-wait", "heartbeat_wait_s: 5.0", "heartbeat_wait_s: .nan", "heartbeat_wait_s")
    app_variant("old-camera", "main_loop_hz: 50", "camera: {}", "Unknown setting app.camera")
    app_variant("old-vio", "main_loop_hz: 50", "vio: {}", "Unknown setting app.vio")
    app_variant("unknown", "main_loop_hz: 50", "main_loop_typo: 50", "Unknown setting")
    app_variant("guidance", "guidance_enabled: false", "guidance_enabled: true", "guidance_enabled")
    missing = vio_variant("missing-dir", "src/vio/estimator/config/gemini_1920x1080", "not-a-directory", "config_dir")
    run(["--vio-config", str(missing), "--config-dir", str(root / "src/vio/estimator/config/gemini_1920x1080"), "--check-config"], contains="Configuration OK")
    vio_variant("bad-age", "max_age_ms: 150", "max_age_ms: -1", "max_age_ms")
    vio_variant("resolution", "width: 1920", "width: 1280", "cam0.resolution")
    vio_variant("depth", "use_depth: true", "use_depth: false", "use_depth_init")
    vio_variant("quality", "quality_ok: 90", "quality_ok: 101", "quality_ok")
    vio_variant("rate", "imu_hz: 200", "imu_hz: 100", "highres_imu_us")
    vio_variant("bad-input", "timeout_ms: 5000", "timeout_ms: -1", "timeout_ms")
    vio_variant("bad-channel", "shm_vio: /pure_uav_camera_vio", "shm_vio: no-leading-slash", "shm_vio")
    vio_variant("bad-fps", "fps: 30", "fps: 0", "vio.fps")
    vio_variant("old-camera-vio", "fps: 30", "exposure: 100", "Unknown setting vio.exposure")
    custom = tmp / "custom-vio.yaml"
    custom.write_text(vio_text.replace("fps: 30", "fps: 45").replace("quality_ok: 90", "quality_ok: 80").replace("use_depth_init: true", "use_depth_init: false"))
    custom_app = tmp / "custom-app.yaml"
    custom_app.write_text(json.dumps({"app": {"main_loop_hz": 25, "vio_config": str(custom)}}))
    output = run(["--config", str(custom_app), "--check-config"], contains="main_loop_hz: 25")
    assert "vio.resolution/fps: 1920x1080@45" in output
    assert "vio.pose.quality_ok/degraded: 80/50" in output
    assert "vio.use_depth/use_depth_init: true/false" in output
    run(["--config", str(custom_app), "--vio-config", "config/vio.yaml", "--check-config"], contains="vio.resolution/fps: 1920x1080@30")
    run(["--shm-vio", "/test_vio", "--shm-timeout-ms", "123", "--check-config"], contains="vio.input.shm_vio: /test_vio")
    run(["--shm-timeout-ms", "0", "--check-config"], contains="vio.input.timeout_ms: 0")
    for value in ("-1", "60001", "abc", "1x", "9999999999999999999"):
        run(["--shm-timeout-ms", value, "--check-config"], 2)
    for flag, name in (("--config", "bad-app"), ("--vio-config", "bad-vio")):
        path = tmp / (name + ".yaml")
        path.write_text("root: [")
        run([flag, str(path), "--check-config"], 2, "Configuration error")
        run([flag, str(tmp / "missing.yaml"), "--check-config"], 2, "Configuration error")
    path = tmp / "driver.yaml"
    path.write_text("communicate:\n  file: /not-a-device\n  baudrate: 1\n")
    run(["--driver-config", str(path), "--check-config"], 2, "baudrate")
    custom_app.write_text(json.dumps({"app": {"driver_config": "missing-driver", "vio_config": "missing-vio"}}))
    run(["--config", str(custom_app), "--driver-config", "config/driver.yaml", "--vio-config", "config/vio.yaml", "--check-config"], contains="Configuration OK")

    # 任务制导：onboard-guidance 与 guidance-camera 分开加载。
    camera_text = (root / "config/guidance-camera.yaml").read_text()
    camera_false = tmp / "camera-false.yaml"
    camera_false.write_text(camera_text.replace("calibrated: true", "calibrated: false"))
    run(["--mission", "--no-send", "--camera-calibration", str(camera_false), "--check-config"], contains="mission_guidance_ready: false")
    camera_true = tmp / "camera-true.yaml"
    camera_true.write_text(
        re.sub(
            r"R_body_cam:\s*\[[^\]]*\]",
            "R_body_cam: [1, 0, 0, 0, 1, 0, 0, 0, 1]",
            camera_text.replace("calibrated: false", "calibrated: true", 1),
            flags=re.S,
        )
    )
    run(["--mission", "--no-send", "--camera-calibration", str(camera_true), "--check-config"], contains="mission_guidance_ready: true")

    camera_bad_r = tmp / "camera-bad-r.yaml"
    camera_bad_r.write_text(
        re.sub(
            r"R_body_cam:\s*\[[^\]]*\]",
            "R_body_cam: [0, 0, 0, 0, 0, 0, 0, 0, 0]",
            camera_text.replace("calibrated: false", "calibrated: true", 1),
            flags=re.S,
        )
    )
    run(["--mission", "--no-send", "--camera-calibration", str(camera_bad_r), "--check-config"], 2, "Configuration error")
    run(["--mission", "--no-send", "--camera-calibration", str(tmp / "missing-camera.yaml"), "--check-config"], 2, "Configuration error")
print(f"PASS: {checks} application/VIO config checks; no hardware opened")
