"""飞控 time_usec → 宿主时钟锚定的纯逻辑测试（不碰串口/硬件）。

守的是 2026-10-18 那个坑：同一台设备、同一条安装，六次录制估出的相机-IMU 时移在
−27…+9 ms 之间跳，而相机侧到包延迟恒定 28.1~28.8 ms —— 变量在 IMU 这条链的
"读线程积压"上。`capture/imu_timebase.py` 用全段下包络把它去掉。
"""
import unittest

import numpy as np

from capture.imu_timebase import anchor_timestamps, fit_timebase

HZ = 200.0
F0 = 1_000_000_000          # 飞控开机 1000 s，单位 µs


def synthetic(seconds=10.0, delay_ms=None, jitter_ms=0.1, drift_ppm=0.0,
              const_ms=0.0, seed=0):
    """造 (fc_usec, host_ns, provisional_ns)。

    host = 真实时刻(飞控轴) + 固定延迟 + 每帧额外延迟（积压） + 抖动。
    provisional 用 host 本身 —— 就是旧的"按读时间打戳"方案。
    """
    rng = np.random.default_rng(seed)
    n = int(seconds*HZ)
    fc = [F0 + int(round(i*1e6/HZ)) for i in range(n)]
    host = []
    for i, f in enumerate(fc):
        extra = 0.0 if delay_ms is None else float(delay_ms[i])*1e6
        host.append(int(f*1000*(1.0+drift_ppm*1e-6) + const_ms*1e6 + extra
                         + rng.normal(0, jitter_ms)*1e6))
    return fc, host, list(host)


def spread(err_ms):
    a = np.asarray(err_ms, dtype=float)
    return float(a.max()-a.min())


class TimebaseTests(unittest.TestCase):
    def test_removes_reader_backlog(self):
        """读线程积压（成片的 5~40 ms 额外延迟）必须被下包络去掉。"""
        rng = np.random.default_rng(1)
        n = int(10.0*HZ)
        delay = np.where(rng.random(n) < 0.3, rng.uniform(5, 40, n), 0.0)
        fc, host, prov = synthetic(10.0, delay_ms=delay, jitter_ms=0.15)
        truth = np.array(fc, dtype=float)*1000
        stamps, info = anchor_timestamps(fc, host, prov)
        self.assertTrue(info["ok"], info.get("reason"))
        err_prov = np.asarray(prov, dtype=float)-truth
        err_anch = np.asarray(stamps, dtype=float)-truth
        self.assertGreater(spread(err_prov)/1e6, 20.0)      # 旧方案抖动 >20 ms
        self.assertLess(spread(err_anch)/1e6, 1.0)          # 新方案压到 <1 ms
        self.assertGreater(info["shift_median_ms"], 0.0)    # 确实回拨了
        # 锚定后不会早于真实时刻（下包络只会留固定延迟，不会穿越）
        self.assertGreater(err_anch.min()/1e6, -1.0)

    def test_constant_latency_is_kept_as_constant(self):
        """固定延迟是常数，标定的 offset 会吸收；要保证的是它不再"抖"。"""
        fc, host, prov = synthetic(8.0, const_ms=30.0, jitter_ms=0.2)
        truth = np.array(fc, dtype=float)*1000
        stamps, info = anchor_timestamps(fc, host, prov)
        self.assertTrue(info["ok"])
        err = np.asarray(stamps, dtype=float)-truth
        self.assertAlmostEqual(float(np.median(err))/1e6, 30.0, delta=1.0)
        self.assertLess(spread(err)/1e6, 0.8)

    def test_estimates_clock_drift(self):
        fc, host, prov = synthetic(20.0, const_ms=2.0, jitter_ms=0.2, drift_ppm=20.0)
        stamps, info = anchor_timestamps(fc, host, prov)
        self.assertTrue(info["ok"])
        self.assertAlmostEqual(info["drift_ppm"], 20.0, delta=8.0)  # 下包络斜率精度 ~2 ppm
        # 漂移被扣掉后，两端都不该出现系统性斜面
        truth = np.array(fc, dtype=float)*1000
        err = np.asarray(stamps, dtype=float)-truth
        self.assertLess(abs(err[-1]-err[0])/1e6, 0.5)

    def test_falls_back_without_fc_time(self):
        fc = [0]*int(2.0*HZ)
        host = [int(1.7e18+i*5e6) for i in range(len(fc))]
        prov = list(host)
        stamps, info = anchor_timestamps(fc, host, prov)
        self.assertFalse(info["ok"])
        self.assertEqual(stamps, prov)
        self.assertIn("飞控时间", info["reason"])

    def test_falls_back_on_non_monotonic_fc_time(self):
        fc, host, prov = synthetic(5.0)
        fc[100] = fc[50]                      # 飞控重启/回绕
        stamps, info = anchor_timestamps(fc, host, prov)
        self.assertFalse(info["ok"])
        self.assertEqual(stamps, prov)

    def test_falls_back_on_short_recording(self):
        fc, host, prov = synthetic(0.2)
        stamps, info = anchor_timestamps(fc, host, prov)
        self.assertFalse(info["ok"])
        self.assertEqual(stamps, prov)

    def test_mixed_samples_stay_strictly_increasing(self):
        fc, host, prov = synthetic(6.0, const_ms=1.0)
        for i in (500, 501, 900):
            fc[i] = 0                         # 个别帧没有 time_usec
        stamps, info = anchor_timestamps(fc, host, prov)
        self.assertTrue(info["ok"], info.get("reason"))
        self.assertGreaterEqual(info["fallback_samples"], 3)
        self.assertTrue(all(b > a for a, b in zip(stamps, stamps[1:])))

    def test_fit_reports_delay_distribution(self):
        rng = np.random.default_rng(3)
        n = int(10.0*HZ)
        delay = np.where(rng.random(n) < 0.1, 20.0, 0.0)
        fc, host, prov = synthetic(10.0, delay_ms=delay, jitter_ms=0.1)
        fit = fit_timebase(fc, host)
        self.assertTrue(fit["ok"])
        self.assertLess(fit["delay_median_ms"], 2.0)
        self.assertGreater(fit["delay_p95_ms"], 10.0)


if __name__ == "__main__":
    unittest.main()
