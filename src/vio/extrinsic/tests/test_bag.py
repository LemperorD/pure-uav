"""ROS1/ROS2 writer-reader roundtrip with deliberately WRONG record times.
Actual header times match the image/IMU CSV. No ROS install is required.
"""
import csv
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from calib.data import Camera, Target, load_dataset, read_imu_csv, ros_stamp
from extrinsics_config import Settings


class RosbagTests(unittest.TestCase):
    def test_header_timestamps_and_compressed_images(self):
        self._roundtrip()

    def test_ros2_humble_header_timestamps(self):
        self._roundtrip(ros2=True)

    def test_same_bag_does_not_remap_clock_domains(self):
        self._roundtrip(ros2=True, imu_shift_ns=2000000000)

    def test_monotonic_samples_with_changed_frame_rejected(self):
        self._roundtrip(ros2=True, frame_switch=True)

    def _roundtrip(self, ros2=False, imu_shift_ns=0, frame_switch=False):
        try:
            if ros2:
                from rosbags.rosbag2 import Writer
            else:
                from rosbags.rosbag1 import Writer
            from rosbags.typesys import Stores, get_typestore
        except ImportError:
            self.skipTest("rosbags not installed")
        ts = get_typestore(Stores.ROS2_HUMBLE if ros2 else Stores.ROS1_NOETIC)
        types = ts.types
        Time = types["builtin_interfaces/msg/Time"]
        Header = types["std_msgs/msg/Header"]
        Compressed = types["sensor_msgs/msg/CompressedImage"]
        Imu = types["sensor_msgs/msg/Imu"]
        Vector = types["geometry_msgs/msg/Vector3"]
        Quaternion = types["geometry_msgs/msg/Quaternion"]
        epoch = 1700000000000000000
        def header(stamp, frame="sensor"):
            sec, ns = divmod(stamp, 1000000000)
            return Header(stamp=Time(sec=sec, nanosec=ns), frame_id=frame, **({} if ros2 else {"seq": 0}))
        image = np.full((480, 640), 235, np.uint8)
        for r in range(7):
            for c in range(9):
                image[80+r*40:120+r*40, 120+c*40:160+c*40] = 20 if (r+c) % 2 == 0 else 235
        camera = Camera({"camera_model": "pinhole", "distortion_model": "radtan",
                         "intrinsics": [700, 700, 320, 240], "distortion_coeffs": [0, 0, 0, 0],
                         "resolution": [640, 480]})
        target = Target({"target_type": "checkerboard", "cols": 8, "rows": 6, "square_size": 0.04})
        cfg = Settings(min_frames=3, max_duration_s=1., camera_rate_hz=10, save_detection_every=0)
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            bag = root / ("synthetic_ros2" if ros2 else "synthetic_headers.bag")
            with Writer(bag, **({"version": 9} if ros2 else {})) as writer:
                ci = writer.add_connection("/cam0/image_raw", Compressed.__msgtype__, typestore=ts)
                cu = writer.add_connection("/imu0", Imu.__msgtype__, typestore=ts)
                for i in range(50):
                    stamp = epoch+i*10000000
                    imu = Imu(header=header(stamp+imu_shift_ns, "other" if frame_switch and i >= 20 else "sensor"), orientation=Quaternion(x=0., y=0., z=0., w=1.),
                              orientation_covariance=np.zeros(9), angular_velocity=Vector(x=0.1, y=0.2, z=0.3),
                              angular_velocity_covariance=np.zeros(9), linear_acceleration=Vector(x=0., y=0., z=9.8),
                              linear_acceleration_covariance=np.zeros(9))
                    # Record times use a separate epoch and burst-like steps.
                    record = 1800000000000000000+(i//5)*50000000
                    writer.write(cu, record, (ts.serialize_cdr if ros2 else ts.serialize_ros1)(imu, imu.__msgtype__))
                    if i % 10 == 0:
                        msg = Compressed(header=header(stamp), format="png", data=cv2.imencode(".png", image)[1])
                        writer.write(ci, record, (ts.serialize_cdr if ros2 else ts.serialize_ros1)(msg, msg.__msgtype__))
            if frame_switch:
                from calib.data import CalibrationError
                with self.assertRaisesRegex(CalibrationError, "frame_id"):
                    load_dataset(camera, target, cfg, root/"result", bag=bag)
                return
            obs, tc, ti, values, origin = load_dataset(camera, target, cfg, root/"result", bag=bag)
            self.assertEqual(origin, epoch)
            np.testing.assert_allclose(tc, np.arange(5)*0.1, atol=1e-15)
            np.testing.assert_allclose(ti, np.arange(50)*0.01+imu_shift_ns*1e-9, atol=1e-15)
            np.testing.assert_allclose(values, np.tile([0., 0., 9.8, 0.1, 0.2, 0.3], (50, 1)))
            self.assertEqual(len(obs), 5)


if __name__ == "__main__":
    unittest.main()
