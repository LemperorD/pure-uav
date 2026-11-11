#!/usr/bin/env python3
"""Run the real main against PTY heartbeats, stopping before camera/VIO startup."""
import errno
import os
import pathlib
import pty
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


def run(autopilot, mission):
    master, slave = pty.openpty()
    stop = threading.Event()
    writer = None
    process = None
    try:
        device = os.ttyname(slave)
        with tempfile.TemporaryDirectory(prefix="main-firmware-", dir=root / "build") as directory:
            driver = pathlib.Path(directory) / "driver.yaml"
            driver.write_text(
                "communicate:\n"
                f"  file: {device}\n"
                "  baudrate: 921600\n"
                "  heartbeat_timeout_ms: 2000\n"
                "  stream:\n"
                "    highres_imu_us: 5000\n"
            )
            arguments = [
                str(binary), "--no-send", "--no-detection",
                "--driver-config", str(driver),
                "--shm-vio", f"/pure_uav_missing_camera_{os.getpid()}_{autopilot}",
                "--shm-timeout-ms", "0",
            ]
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
                # Deliberately absent camera: passing the real gate reaches this error.
                assert "[MAIN] 已连接，固件：PX4" in output, output
                assert "[CAM] 连接 camera_server 失败" in output, output
                assert "只接受" not in output, output
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
print("PASS: real main accepts PX4 and rejects unsupported firmware, with and without mission")
