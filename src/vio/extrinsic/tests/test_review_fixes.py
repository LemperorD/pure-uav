"""Focused regressions for review findings, not real-hardware certification."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import cv2
import numpy as np
import yaml

from calib.acquisition import acquisition_preflight
from calib.data import CalibrationError, Camera, Observation, read_yaml
from calib.solver import Problem, save_result, opencv_yaml
from calib.spline import PoseSpline
from extrinsics_config import Settings
from prepare_openvins import prepare

ROOT = Path(__file__).resolve().parents[1]


def camera():
    return Camera({"camera_model": "pinhole", "distortion_model": "radtan",
                   "intrinsics": [600., 610., 400., 300.], "distortion_coeffs": [0., 0., 0., 0.],
                   "resolution": [800, 600], "rostopic": "/template/cam"})


class ReviewTests(unittest.TestCase):
    def test_settings_scientific_strings(self):
        d = yaml.safe_load("accel_sigma_m_s2: 1e-4\njoint_max_nfev: '300'\n")
        self.assertIsInstance(d["accel_sigma_m_s2"], str)
        cfg = Settings(**d)
        self.assertEqual(cfg.accel_sigma_m_s2, 1e-4)
        self.assertIsInstance(cfg.joint_max_nfev, int)

    def test_settings_invalid_types_and_ranges(self):
        for key, value in (("gyro_sigma_rad_s", True), ("start_s", -1), ("min_frames", 2.5),
                           ("joint_max_nfev", 0), ("seed", -1), ("verbose", 3),
                           ("image_sigma_px", "nan"), ("camera_rate_hz", "inf"),
                           ("depth_sigma_m", None), ("optimizer_ftol", 1e-20)):
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                Settings(**{key: value})

    def test_settings_cli_diagnostics(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for i, text in enumerate(("min_frames: 3.5", "gyro_sigma_rad_s: true", "[]", "not_a_field: 4")):
                settings = root/f"s{i}.yaml"
                settings.write_text(text)
                out = root/f"out{i}"
                proc = subprocess.run([sys.executable, str(ROOT/"calibrate_extrinsics.py"),
                                       "--settings", str(settings), "--output", str(out)],
                                      capture_output=True, text=True)
                self.assertEqual(proc.returncode, 2)
                self.assertNotIn("Traceback", proc.stderr)
                self.assertEqual(json.loads((out/"report.json").read_text())["status"], "failed")
                self.assertFalse((out/"camchain-kalibr.yaml").exists())

    def test_acquisition_unknown_is_warning_not_certification(self):
        info = acquisition_preflight()
        self.assertEqual(len(info["warnings"]), 3)
        self.assertIn("not inferred", info["verification"])

    def test_acquisition_blockers_and_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/"acq.yaml"
            for item in ({"imu_processing": "filtered"}, {"clock_domain": "different"},
                         {"imu_instance": "mixed"}, {"gyro_instance": "0", "accel_instance": "1"},
                         {"imu_processing": True}, {"firmware": 10}, {"imuprocessing": "raw"}):
                p.write_text(yaml.safe_dump(item))
                with self.subTest(item=item), self.assertRaises(CalibrationError):
                    acquisition_preflight(p)
            p.write_text("imu_processing: raw\nclock_domain: shared\nimu_instance: single\n")
            self.assertEqual(acquisition_preflight(p)["warnings"], [])

    def test_declared_filtered_data_cli_rejected_before_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            acq = root/"acq.yaml"
            acq.write_text("imu_processing: filtered\n")
            proc = subprocess.run([sys.executable, str(ROOT/"calibrate_extrinsics.py"),
                                   "--acquisition", str(acq), "--output", str(root/"out")],
                                  capture_output=True, text=True)
            self.assertEqual(proc.returncode, 2)
            report = json.loads((root/"out/report.json").read_text())
            self.assertIn("滤波", report["failure_reasons"][0])
            self.assertFalse((root/"out/camchain-kalibr.yaml").exists())

    def _template(self, root, bag=False):
        report = {"status": "passed_residual_and_excitation_checks", "T_cam_imu": np.eye(4).tolist(),
                  "T_imu_cam": np.eye(4).tolist(), "timeshift_cam_imu": 0.012}
        if bag:
            report["inputs"] = {"bag": "recording", "cam_topic": "/recorded/cam", "imu_topic": "/recorded/imu"}
        save_result(root/"cal", camera(), report)
        (root/"template").mkdir()
        noise = {"imu0": {"rostopic": "/template/imu", "gyroscope_noise_density": 1.0e-4,
                          "T_i_b": np.eye(4).tolist(), "custom_sensor_value": 42}}
        (root/"template/imu.yaml").write_text(opencv_yaml(noise))
        (root/"template/estimator_config.yaml").write_text(opencv_yaml({
            "relative_config_imu": "imu.yaml", "calib_cam_timeoffset": True, "topic_imu": "/old/imu"}))
        return noise

    def test_openvins_overrides_both_chains_and_preserves_noise(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            original = self._template(root)
            template_before = (root/"template/imu.yaml").read_bytes()
            prepare(root/"cal", root/"template", root/"ov", "/cuav/raw", "/orbbec/rgb", "online")
            estimator = read_yaml(root/"ov/estimator_config.yaml")
            imu = read_yaml(root/"ov/kalibr_imu_chain.yaml")
            cam = read_yaml(root/"ov/kalibr_imucam_chain.yaml")
            self.assertEqual(imu["imu0"]["rostopic"], estimator["topic_imu"])
            self.assertEqual(estimator["topic_imu"], "/cuav/raw")
            self.assertEqual(cam["cam0"]["rostopic"], estimator["topic_camera0"])
            self.assertEqual(estimator["topic_camera0"], "/orbbec/rgb")
            self.assertTrue(estimator["calib_cam_timeoffset"])
            imu["imu0"]["rostopic"] = original["imu0"]["rostopic"]
            self.assertEqual(imu, original)
            self.assertEqual((root/"template/imu.yaml").read_bytes(), template_before)
            for name in ("kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml", "estimator_config.yaml"):
                f = cv2.FileStorage(str(root/"ov"/name), cv2.FILE_STORAGE_READ)
                self.assertTrue(f.isOpened())
                f.release()

    def test_openvins_default_topics_and_frozen_mode(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._template(root)
            prepare(root/"cal", root/"template", root/"ov")
            self.assertEqual(read_yaml(root/"ov/kalibr_imu_chain.yaml")["imu0"]["rostopic"], "/template/imu")
            self.assertEqual(read_yaml(root/"ov/kalibr_imucam_chain.yaml")["cam0"]["rostopic"], "/template/cam")
            self.assertFalse(read_yaml(root/"ov/estimator_config.yaml")["calib_cam_timeoffset"])

    def test_openvins_recorded_bag_topics_take_precedence(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._template(root, bag=True)
            prepare(root/"cal", root/"template", root/"ov")
            self.assertEqual(read_yaml(root/"ov/kalibr_imu_chain.yaml")["imu0"]["rostopic"], "/recorded/imu")
            self.assertEqual(read_yaml(root/"ov/kalibr_imucam_chain.yaml")["cam0"]["rostopic"], "/recorded/cam")

    def test_physical_projection_rejects_invalid_depth(self):
        c = camera()
        for z in (-1., 0.):
            xyz = np.array([[0.001, 0., z]])
            with self.assertRaises(CalibrationError):
                c.project(xyz)
            self.assertTrue(np.all(np.isfinite(c.project(xyz, trial_depth_floor=0.01))))

    def test_depth_barrier_sparsity_with_independent_finite_differences(self):
        # Tiny explicitly constructed problem: bypass motion-quality checks,
        # testing ONLY residual dependencies and behind-camera trial behavior.
        p = Problem.__new__(Problem)
        p.cfg, p.camera = Settings(), camera()
        p.spline = PoseSpline(0., 0.2, 9)
        p.tc, p.ti = np.array([0.55, 0.71]), np.array([0.58, 0.65, 0.74])
        p.xyz = np.tile([[0.001, 0., 0.]], (4, 1))
        p.uv = np.tile([[400., 300.]], (4, 1))
        p.frame_ids = np.array([0, 0, 1, 1])
        p.imu = np.tile([0., 0., 9.80665, 0., 0., 0.], (3, 1))
        p.npix = p.uv.size
        p.obs = [Observation(0, p.xyz[:2], p.uv[:2], np.eye(3), np.zeros(3), 0.),
                 Observation(1, p.xyz[2:], p.uv[2:], np.eye(3), np.zeros(3), 0.)]
        ctrl = np.zeros((9, 6))
        ctrl[:, 5] = 0.10  # z_cam=-0.10: intentionally invalid trial pose
        p.edge_ids = np.array([0, 1, 7, 8])
        p.edge_ref = ctrl[p.edge_ids].copy()
        x = np.r_[np.zeros(13), [0., 0., -9.80665], ctrl.ravel()]
        r = p.residual(x)
        self.assertTrue(np.all(r[p.npix:p.npix+4] > 0))
        mask = p.sparsity().toarray().astype(bool)
        self.assertEqual(mask.shape, (len(r), len(x)))
        for j in range(len(x)):
            xp, xm = x.copy(), x.copy()
            xp[j] += 1e-6
            xm[j] -= 1e-6
            numeric = (p.residual(xp)-p.residual(xm))/(2e-6)
            self.assertTrue(np.all(mask[np.abs(numeric) > 1e-5, j]), f"missing dependencies column {j}")
        ctrl[:, 5] = -1.
        good = np.r_[x[:16], ctrl.ravel()]
        np.testing.assert_array_equal(p.residual(good)[p.npix:p.npix+4], np.zeros(4))


if __name__ == "__main__":
    unittest.main()
