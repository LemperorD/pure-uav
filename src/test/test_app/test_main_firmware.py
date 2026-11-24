#!/usr/bin/env python3
"""Run the real main against PTY heartbeats and a fake camera service."""
import errno
import os
import pathlib
import pty
import shutil
import struct
import subprocess
import sys
import tempfile
import threading

binary = pathlib.Path(sys.argv[1]).resolve()
root = pathlib.Path(sys.argv[2]).resolve()


def heartbeat(sequence, autopilot):
    # MAVLink v1 HEARTBEAT: message id 0, payload 9 bytes, CRC_EXTRA 50.
    payload = struct.pack("<IBBBBB", 3 << 16, 2, autopilot, 1, 3, 3)
    body = bytes((len(payload), sequence, 1, 1, 0)) + payload
    crc = 0xFFFF
    for value in body + bytes((50,)):
        tmp = value ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return b"\xfe" + body + struct.pack("<H", crc)


def run(autopilot, mission, camera_mode=None):
    master, slave = pty.openpty()
    stop = threading.Event()
    writer = None
    process = None
    try:
        device = os.ttyname(slave)
        with tempfile.TemporaryDirectory(prefix="main-firmware-", dir=root / "build") as directory:
            fixture = pathlib.Path(directory)
            driver = fixture / "driver.yaml"
            driver.write_text(
                "communicate:\n"
                f"  file: {device}\n"
                "  baudrate: 921600\n"
                "  heartbeat_timeout_ms: 2000\n"
                "  stream:\n"
                "    highres_imu_us: 5000\n"
            )
            entrypoint = binary
            if camera_mode:
                entrypoint = fixture / "main"
                shutil.copy2(binary, entrypoint)
                camera = fixture / "camera_server"
                camera.write_text(f"#!{sys.executable}\n" + '''
import argparse
import os
import pathlib
import signal

parser = argparse.ArgumentParser()
parser.add_argument("--config", type=pathlib.Path, required=True)
parser.add_argument("--vio-channel", required=True)
parser.add_argument("--ready-fd", type=int, required=True)
args = parser.parse_args()
assert args.vio_channel.startswith("/pure_uav_missing_camera_")
marker = args.config.with_name("camera-status")

def stop(*_):
    marker.write_text("stopped")
    raise SystemExit(0)

mode = args.config.with_name("camera-mode").read_text()
signal.signal(signal.SIGTERM, stop)
marker.write_text("started")
if mode == "failed":
    raise SystemExit(7)
if mode == "ready":
    os.write(args.ready_fd, b"R")
    os.close(args.ready_fd)
while True:
    signal.pause()
''')
                camera.chmod(0o755)
                camera_config = fixture / "camera.yaml"
                camera_text = (root / "config/camera.yaml").read_text()
                camera_config.write_text(camera_text.replace("ready_timeout_ms: 15000", "ready_timeout_ms: 1000"))
                (fixture / "camera-mode").write_text(camera_mode)
            arguments = [
                str(entrypoint), "--no-send", "--no-detection",
                "--driver-config", str(driver),
                "--shm-vio", f"/pure_uav_missing_camera_{os.getpid()}_{autopilot}",
            ]
            if camera_mode:
                arguments += ["--camera-config", str(camera_config.relative_to(root))]
            else:
                arguments += ["--external-camera", "--shm-timeout-ms", "0"]
            if mission:
                arguments.append("--mission")
            process = subprocess.Popen(arguments, cwd="/tmp", stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

            def feed():
                sequence = 0
                while not stop.is_set():
                    try:
                        os.write(master, heartbeat(sequence, autopilot))
                    except OSError as error:
                        if error.errno not in (errno.EIO, errno.EBADF):
                            raise
                        return
                    sequence = (sequence + 1) & 0xFF
                    stop.wait(0.03)

            writer = threading.Thread(target=feed)
            writer.start()
            output, _ = process.communicate(timeout=8)
            assert process.returncode == 1, (autopilot, mission, process.returncode, output)
            assert "未收到飞控心跳" not in output, output
            if autopilot == 12:
                assert "[MAIN] 已连接，固件：PX4" in output, output
                assert "只接受" not in output, output
                if camera_mode in (None, "ready"):
                    assert "[CAM] 连接 camera_server 失败" in output, output
                elif camera_mode == "silent":
                    assert "等待 camera_server 就绪超时" in output, output
                else:
                    assert "camera_server 在就绪前退出" in output, output
                if camera_mode:
                    expected = "started" if camera_mode == "failed" else "stopped"
                    assert (fixture / "camera-status").read_text() == expected, output
            else:
                assert "当前入口只接受 PX4" in output, output
                assert "[MAIN] 已连接" not in output, output
                assert "[CAM]" not in output, output
    finally:
        stop.set()
        if process is not None and process.poll() is None:
            process.kill()
            process.communicate(timeout=3)
        if writer is not None:
            writer.join(timeout=1)
        os.close(slave)
        os.close(master)


for mission in (False, True):
    run(12, mission)  # MAV_AUTOPILOT_PX4
    run(3, mission)   # MAV_AUTOPILOT_ARDUPILOTMEGA
for camera_mode in ("ready", "silent", "failed"):
    run(12, False, camera_mode)
print("PASS: firmware gate, managed camera readiness, timeout and cleanup")
