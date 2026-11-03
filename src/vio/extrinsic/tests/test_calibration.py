"""Model/convention/IO tests: python -m unittest discover -s tests -v."""
import unittest
import tempfile
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from calib.data import Camera, CalibrationError, Target, check_stamps, decode_ros_image, stamp_ns
from calib.spline import PoseSpline, invert_transform, predict_imu


class CalibrationTests(unittest.TestCase):
    def test_spline_analytic_derivatives(self):
        rng = np.random.default_rng(5)
        rv, p = rng.normal(0, 0.18, (12, 3)), rng.normal(0, 0.1, (12, 3))
        s = PoseSpline(0, 0.2, 12)
        times = np.array([0.31, 0.4, 0.72, 1.0])
        eps = 1e-5
        r, x, a, w, alpha = s.evaluate(times, rv, p)
        rp, xp, _, wp, _ = s.evaluate(times+eps, rv, p)
        rm, xm, _, wm, _ = s.evaluate(times-eps, rv, p)
        numeric_w = Rotation.from_matrix(np.swapaxes(rm, -1, -2) @ rp).as_rotvec()/(2*eps)
        np.testing.assert_allclose(w, numeric_w, atol=1e-7)
        np.testing.assert_allclose(alpha[[0, 2]], ((wp-wm)/(2*eps))[[0, 2]], atol=2e-7)
        # At knots the cubic third derivative jumps; central differences have O(eps) error.
        np.testing.assert_allclose(alpha, (wp-wm)/(2*eps), atol=1e-3)
        np.testing.assert_allclose(a[[0, 2]], ((xp-2*x+xm)/eps**2)[[0, 2]], atol=2e-6)
        np.testing.assert_allclose(a, (xp-2*x+xm)/eps**2, atol=1e-3)

    def test_lever_arm_and_transform_conventions(self):
        rng = np.random.default_rng(14)
        s = PoseSpline(-0.5, 0.15, 16)
        rv, p = rng.normal(0, 0.15, (16, 3)), rng.normal(0, 0.1, (16, 3))
        t = np.array([0.23, 0.552, 0.81])
        rci = Rotation.from_euler("xyz", [25, -17, 33], degrees=True).as_matrix()
        pci, gravity = np.array([0.07, -0.03, 0.04]), np.array([0, 0, -9.80665])
        r, _, a, w, alpha = s.evaluate(t, rv, p)
        gyro, accel = predict_imu(r, a, w, alpha, rci, pci, gravity, np.zeros(3), np.zeros(3))
        eps = 1e-5
        def pose(tt):
            rr, pp, *_ = s.evaluate(tt, rv, p)
            return rr @ rci, pp+np.einsum("nij,j->ni", rr, pci)
        ri, pi = pose(t)
        rip, pip = pose(t+eps)
        rim, pim = pose(t-eps)
        numeric_g = Rotation.from_matrix(np.swapaxes(rim, -1, -2) @ rip).as_rotvec()/(2*eps)
        numeric_a = np.einsum("nji,nj->ni", ri, (pip-2*pi+pim)/eps**2-gravity)
        np.testing.assert_allclose(gyro, numeric_g, atol=2e-7)
        np.testing.assert_allclose(accel, numeric_a, atol=3e-6)
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = rci, pci
        np.testing.assert_allclose(transform @ invert_transform(transform), np.eye(4), atol=1e-14)

    def test_epoch_precision_and_bad_timestamps(self):
        a, b = stamp_ns("1700000000.000000001", "s"), stamp_ns("1700000000.000000002", "s")
        self.assertEqual(b-a, 1)
        for stamps in ([10, 10, 11], [11, 10]):
            with self.assertRaises(CalibrationError):
                check_stamps(stamps, "test")
        with self.assertRaises(CalibrationError):
            stamp_ns("0.0000000001", "s")

    def test_projection_matches_opencv(self):
        c = Camera({"camera_model": "pinhole", "distortion_model": "radtan", "intrinsics": [700, 710, 400, 300],
                    "distortion_coeffs": [-0.12, 0.03, 0.001, -0.002, 0.01], "resolution": [800, 600]})
        xyz = np.array([[0.2, 0.1, 1.], [-0.3, 0.2, 0.9], [0.1, -0.1, 2.]])
        expected = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), c.k, c.dist)[0].reshape(-1, 2)
        np.testing.assert_allclose(c.project(xyz), expected, atol=1e-12)

    def test_aprilgrid_detection_geometry(self):
        # 自己用 generateImageMarker 拼的板子：tag 正着贴，tag_rotation_deg 必须是 0。
        target = Target({"target_type": "aprilgrid", "tagRows": 3, "tagCols": 3,
                         "tagSize": 0.04, "tagSpacing": 0.5, "tag_rotation_deg": 0}, min_tags=4)
        image = np.full((500, 500), 255, np.uint8)
        for row in range(3):
            for col in range(3):
                tag_id = (2-row)*3+col
                tag = (cv2.aruco.generateImageMarker(target.dictionary, tag_id, 100)
                       if hasattr(cv2.aruco, "generateImageMarker")
                       else cv2.aruco.drawMarker(target.dictionary, tag_id, 100))
                image[50+150*row:150+150*row, 50+150*col:150+150*col] = tag
        points, uv = target.detect(image)
        self.assertEqual(len(points), 36)
        expected = np.column_stack([50+2500*points[:, 0], 450-2500*points[:, 1]])
        np.testing.assert_allclose(uv, expected, atol=1.1)

    def test_aprilgrid_tag_rotation_matches_board(self):
        """Kalibr PDF 的板子每张 tag 转了 180°，tag_rotation_deg 必须跟着填 180。

        2026-10-18 真机踩到：填错时检测照样成功，但棋盘坐标和图像角点的对应关系
        整体转了 180°，平面 PnP 残差 130 px，标定必失败。这里用"平面→图像是同一个
        仿射变换"这个不变量把它锁住：对应关系对，四组角点能被同一个仿射变换拟合到
        亚像素；错，则残差上百像素。
        """
        def board(rotated):
            tgt = Target({"target_type": "aprilgrid", "tagRows": 3, "tagCols": 3,
                          "tagSize": 0.04, "tagSpacing": 0.5,
                          "tag_rotation_deg": 180 if rotated else 0}, min_tags=4)
            image = np.full((500, 500), 255, np.uint8)
            for row in range(3):
                for col in range(3):
                    tag = cv2.aruco.generateImageMarker(tgt.dictionary, (2-row)*3+col, 100)
                    if rotated:
                        tag = np.rot90(tag, 2)
                    image[50+150*row:150+150*row, 50+150*col:150+150*col] = tag
            return tgt, image

        def affine_residual(points, uv):
            design = np.column_stack([points[:, 0], points[:, 1], np.ones(len(points))])
            coef, *_ = np.linalg.lstsq(design, uv, rcond=None)
            return float(np.sqrt(np.mean(np.sum((design@coef-uv)**2, axis=1))))

        for tag_rotation, expect_small in ((0, True), (180, False)):
            upright, upright_image = board(False)
            upright.tag_rotation, upright.roll = tag_rotation, (tag_rotation//90) % 4
            rotated, rotated_image = board(True)
            rotated.tag_rotation, rotated.roll = tag_rotation, (tag_rotation//90) % 4
            resid_upright = affine_residual(*upright.detect(upright_image))
            resid_rotated = affine_residual(*rotated.detect(rotated_image))
            with self.subTest(tag_rotation=tag_rotation):
                if expect_small:
                    self.assertLess(resid_upright, 1.0)
                    self.assertGreater(resid_rotated, 50.0)   # 填 0 却用 Kalibr 板：必崩
                else:
                    self.assertGreater(resid_upright, 50.0)   # 填 180 却用自拼板：必崩
                    self.assertLess(resid_rotated, 1.0)

    def test_opencv_yaml_and_inverse_export(self):
        from calib.solver import save_result
        from calib.data import read_yaml
        c = Camera({"camera_model": "pinhole", "distortion_model": "radtan",
                    "intrinsics": [700., 710., 400., 300.], "distortion_coeffs": [0., 0., 0., 0.],
                    "resolution": [800, 600], "rostopic": "/cam0/image_raw"})
        t = np.eye(4)
        t[:3, :3] = Rotation.from_euler("xyz", [20, 10, -25], degrees=True).as_matrix()
        t[:3, 3] = [0.04, -0.02, 0.03]
        report = {"status": "passed", "T_cam_imu": t.tolist(),
                  "T_imu_cam": invert_transform(t).tolist(), "timeshift_cam_imu": -0.013}
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            save_result(folder, c, report)
            f = cv2.FileStorage(str(folder/"camchain-openvins.yaml"), cv2.FILE_STORAGE_READ)
            self.assertTrue(f.isOpened())
            node = f.getNode("cam0")
            values = np.array([[node.getNode("T_imu_cam").at(r).at(col).real() for col in range(4)] for r in range(4)])
            np.testing.assert_allclose(values, invert_transform(t))
            self.assertAlmostEqual(node.getNode("timeshift_cam_imu").real(), -0.013)
            self.assertEqual(node.getNode("intrinsics").size(), 4)
            f.release()
            np.testing.assert_allclose(read_yaml(folder/"camchain-kalibr.yaml")["cam0"]["T_cam_imu"], t)

    def test_openvins_monocular_config_export(self):
        import json
        from calib.data import read_yaml
        from calib.solver import save_result, opencv_yaml
        from prepare_openvins import prepare
        c = Camera({"camera_model": "pinhole", "distortion_model": "radtan",
                    "intrinsics": [700., 710., 400., 300.], "distortion_coeffs": [0., 0., 0., 0.],
                    "resolution": [800, 600]})
        report = {"status": "passed_residual_and_excitation_checks", "T_cam_imu": np.eye(4).tolist(),
                  "T_imu_cam": np.eye(4).tolist(), "timeshift_cam_imu": 0.012}
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            save_result(root/"cal", c, report)
            template = root/"template"
            template.mkdir()
            (template/"imu.yaml").write_text("imu0: {rostopic: /imu0}\n")
            (template/"estimator_config.yaml").write_text(opencv_yaml({
                "relative_config_imu": "imu.yaml", "max_cameras": 2, "use_stereo": True,
                "calib_cam_extrinsics": True, "calib_cam_timeoffset": True, "use_mask": True}))
            prepare(root/"cal", template, root/"ov", "/cuav/imu")
            estimator = read_yaml(root/"ov/estimator_config.yaml")
            self.assertEqual(estimator["max_cameras"], 1)
            self.assertFalse(estimator["calib_cam_extrinsics"])
            self.assertFalse(estimator["calib_cam_timeoffset"])
            f = cv2.FileStorage(str(root/"ov/estimator_config.yaml"), cv2.FILE_STORAGE_READ)
            self.assertTrue(f.isOpened())
            f.release()
            with self.assertRaises(CalibrationError):
                prepare(root/"cal", template, root/"ov", "/cuav/imu")

    def test_failed_and_five_coefficient_exports(self):
        from calib.solver import save_result
        c = Camera({"camera_model": "pinhole", "distortion_model": "radtan",
                    "intrinsics": [700., 710., 400., 300.], "distortion_coeffs": [0., 0., 0., 0., 0.01],
                    "resolution": [800, 600]})
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            save_result(root/"failed", c, {"status": "failed"})
            self.assertFalse((root/"failed/camchain-kalibr.yaml").exists())
            report = {"status": "passed", "T_cam_imu": np.eye(4).tolist(),
                      "T_imu_cam": np.eye(4).tolist(), "timeshift_cam_imu": 0.}
            save_result(root/"five", c, report)
            self.assertFalse((root/"five/camchain-kalibr.yaml").exists())
            self.assertTrue((root/"five/camchain-opencv.yaml").exists())
            self.assertFalse((root/"five/camchain-openvins.yaml").exists())

    def test_clock_affine_fit_and_failure(self):
        from remap_imu_clock import fit_clock
        imu = np.arange(0, 11000000000, 1000000000, dtype=np.int64)+123456789
        host = np.rint(1.00004*(imu-imu[0])).astype(np.int64)+1700000000123456789
        scale, offset, _, _, residual = fit_clock(imu, host)
        self.assertAlmostEqual(scale, 1.00004, places=10)
        self.assertLess(np.max(np.abs(residual)), 1e-6)
        broken = host.copy()
        broken[4] += 50000000
        with self.assertRaises(CalibrationError):
            fit_clock(imu, broken)

    def test_static_motion_rejected(self):
        from calib.data import Observation
        from calib.solver import Problem
        from extrinsics_config import Settings
        c = Camera({"camera_model": "pinhole", "distortion_model": "radtan",
                    "intrinsics": [700, 710, 400, 300], "distortion_coeffs": [0, 0, 0, 0],
                    "resolution": [800, 600]})
        xyz = np.array([[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0.1, 0.1, 0]], float)
        tc = np.arange(0, 6.01, 0.1)
        pp = np.array([0.05, 0.05, -1.])
        uv = c.project(xyz-pp)
        obs = [Observation(int(t*1e9), xyz, uv, np.eye(3), pp, 0.) for t in tc]
        ti = np.arange(-0.2, 6.2, 0.01)
        imu = np.tile([0, 0, 9.80665, 0, 0, 0], (len(ti), 1))
        problem = Problem(c, obs, tc, ti, imu, Settings(verbose=0))
        with self.assertRaises(CalibrationError):
            problem.initialize_globals()

    def test_ros_image_padding_and_rgb(self):
        from types import SimpleNamespace
        data = np.array([255, 0, 0, 0, 255, 0, 99, 99], np.uint8)
        msg = SimpleNamespace(data=data, height=1, width=2, step=8, encoding="rgb8")
        expected = cv2.cvtColor(np.array([[[255, 0, 0], [0, 255, 0]]], np.uint8), cv2.COLOR_RGB2GRAY)
        np.testing.assert_array_equal(decode_ros_image(msg), expected)


if __name__ == "__main__":
    unittest.main()
