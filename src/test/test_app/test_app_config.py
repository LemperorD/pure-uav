#!/usr/bin/env python3
"""Exercise the actual main config path without opening a camera or serial port."""
import json
import pathlib
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
run(["--help"], contains="--check-config")
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
print(f"PASS: {checks} application/VIO config checks; no hardware opened")
