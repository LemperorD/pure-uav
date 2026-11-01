"""连续窗口选择 + 检出诊断（纯时间序列，不读图像）。

这些用例守的是 2026-10-18 那次"每个 session 都报大空洞"的回归：
    - 检出有空洞时，程序要自己挑出最长连续段，而不是整段拒绝；
    - 挑不出来时，必须说清空洞在哪、多长、最长连续段多长、为什么被拒（运动模糊）；
    - 检测抽样率与求解抽样率必须解耦：决定窗口边界的是检出，不是抽样。
"""
import unittest

import numpy as np

from calib.windows import (continuous_runs, describe_problem, gaps_over, qualifying_runs,
                           run_info, visual_diagnostics)
from extrinsics_config import Settings


def fake_extraction(times, accepted, sharpness, p90=120.0):
    return [{"timestamp_ns": int(round(t*1e9)), "detected": True, "accepted": bool(a),
             "sharpness": float(s), "p90": float(p90), "pnp_rms_px": 1.0 if a else None}
            for t, a, s in zip(times, accepted, sharpness)]


class WindowTests(unittest.TestCase):
    def test_runs_split_on_gap(self):
        self.assertEqual(continuous_runs([0.0, 0.1, 0.2, 1.0, 1.1, 1.2], 0.35),
                         [(0, 2), (3, 5)])
        self.assertEqual(continuous_runs([], 0.35), [])
        self.assertEqual(continuous_runs([3.0], 0.35), [(0, 0)])

    def test_gaps_are_sorted_by_length(self):
        gaps = gaps_over([0.0, 0.9, 1.0, 1.4], 0.35)
        self.assertEqual([t for t, _ in gaps], [0.0, 1.0])
        np.testing.assert_allclose([g for _, g in gaps], [0.9, 0.4])
        self.assertEqual(gaps_over([0.0, 0.1, 0.2], 0.35), [])

    def test_longest_qualifying_run_wins(self):
        # 段 A: 0→4.9s（50 帧，差一点点不够 5s）；段 B: 6→12s（61 帧，6s 够）
        times = np.r_[np.arange(0.0, 5.0, 0.1), np.arange(6.0, 12.1, 0.1)]
        best = qualifying_runs(times, max_gap_s=0.35, min_frames=35, min_duration_s=5.0)
        self.assertEqual(len(best), 1)
        self.assertAlmostEqual(best[0]["start_s"], 6.0)
        self.assertAlmostEqual(best[0]["stop_s"], 12.0)
        self.assertEqual(best[0]["frames"], len(times)-len(np.arange(0.0, 5.0, 0.1)))
        self.assertEqual((best[0]["start_index"], best[0]["stop_index"]),
                         (len(times)-best[0]["frames"], len(times)-1))
        # 门槛提到 7s：两段都不够，必须返回空 —— 不能悄悄退而求其次
        self.assertEqual(qualifying_runs(times, max_gap_s=0.35, min_frames=35, min_duration_s=7.0), [])

    def test_diagnostics_report_gaps_and_blur(self):
        times = np.array([0.0, 0.1, 0.2, 2.0, 2.1, 2.2])
        extraction = fake_extraction(times, [True, True, True, False, False, False],
                                     [1000.0, 1000.0, 1000.0, 300.0, 300.0, 300.0])
        diag = visual_diagnostics(times, extraction, max_gap_s=0.35, min_frames=35,
                                  min_duration_s=5.0)
        self.assertAlmostEqual(diag["max_gap_s"], 1.8)
        self.assertEqual(len(diag["gaps_over_limit_s"]), 1)
        self.assertIsNone(diag["best_run"])
        self.assertTrue(diag["motion_blur_suspected"])
        self.assertAlmostEqual(diag["sharpness"]["rejected_to_accepted_ratio"], 0.3)
        text = "\n".join(describe_problem(diag, Settings()))
        self.assertIn("检出空洞 1 处", text)
        self.assertIn("1.80s", text)
        self.assertIn("运动模糊", text)
        self.assertIn("放慢转动", text)

    def test_sharp_accepted_only_is_not_blur(self):
        times = np.arange(0.0, 6.0, 0.1)
        extraction = fake_extraction(times, [True]*len(times), [900.0]*len(times))
        diag = visual_diagnostics(times, extraction, max_gap_s=0.35, min_frames=35,
                                  min_duration_s=5.0)
        self.assertFalse(diag["motion_blur_suspected"])
        self.assertIsNotNone(diag["best_run"])
        self.assertEqual(diag["best_run"]["frames"], len(times))

    def test_dark_recording_blames_light_not_blur(self):
        """房间关灯时清晰度同样会掉 —— 诊断必须说"太暗"，不能误导成"运动模糊"。"""
        times = np.arange(0.0, 6.0, 0.1)
        extraction = fake_extraction(times, [True, False]*len(times), [400.0, 300.0]*len(times),
                                     p90=30.0)   # 白格只有 30/255
        diag = visual_diagnostics(times, extraction, max_gap_s=0.35, min_frames=35,
                                  min_duration_s=5.0)
        self.assertTrue(diag["too_dark"])
        self.assertFalse(diag["motion_blur_suspected"])
        text = "\n".join(describe_problem(diag, Settings()))
        self.assertIn("太暗", text)
        self.assertIn("补光", text)
        self.assertNotIn("被拒帧清晰度", text)   # 不要给出"运动模糊"的证据链

    def test_bright_recording_still_reports_blur(self):
        times = np.arange(0.0, 6.0, 0.1)
        extraction = fake_extraction(times, [True, False]*len(times), [1000.0, 300.0]*len(times),
                                     p90=140.0)
        diag = visual_diagnostics(times, extraction, max_gap_s=0.35, min_frames=35,
                                  min_duration_s=5.0)
        self.assertFalse(diag["too_dark"])
        self.assertTrue(diag["motion_blur_suspected"])

    def test_no_detection_at_all_still_blames_darkness(self):
        """一帧都没通过（accepted 组为空）时也必须报"太暗"，不能什么都不说。"""
        extraction = [{"timestamp_ns": int(i*1e8), "detected": False, "accepted": False,
                       "sharpness": 200.0, "p90": 30.0, "pnp_rms_px": None} for i in range(20)]
        diag = visual_diagnostics(np.zeros(0), extraction, max_gap_s=0.35,
                                  min_frames=35, min_duration_s=5.0)
        self.assertTrue(diag["too_dark"])          # 退到 all_p90 再判
        text = "\n".join(describe_problem(diag, Settings()))
        self.assertIn("太暗", text)
        self.assertIn("一个 tag 都没解出", text)

    def test_run_info_is_index_consistent(self):
        times = np.array([0.0, 0.1, 0.2])
        info = run_info(times, 1, 2)
        self.assertEqual((info["start_index"], info["stop_index"]), (1, 2))
        self.assertAlmostEqual(info["duration_s"], 0.1)
        self.assertEqual(info["frames"], 2)


class WindowSettingsTests(unittest.TestCase):
    def test_detect_rate_zero_is_legal(self):
        self.assertEqual(Settings(detect_rate_hz=0).detect_rate_hz, 0.0)
        self.assertEqual(Settings(detect_rate_hz=15).detect_rate_hz, 15.0)

    def test_auto_window_switch_and_rejections(self):
        self.assertEqual(Settings().auto_window, 1)   # 默认自动选窗
        self.assertEqual(Settings(auto_window=0).auto_window, 0)
        for bad in ({"auto_window": 2}, {"detect_rate_hz": -1}, {"min_duration_s": 0}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                Settings(**bad)

    def test_cli_flags_exist(self):
        from calibrate_extrinsics import parser
        args = parser().parse_args(["--no-auto-window", "--min-duration", "4.0",
                                    "--detect-rate", "0"])
        self.assertTrue(args.no_auto_window)
        self.assertEqual(args.min_duration, 4.0)
        self.assertEqual(args.detect_rate, 0.0)


if __name__ == "__main__":
    unittest.main()
