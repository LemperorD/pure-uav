#!/usr/bin/env python3
"""Reproduce bandwidth/filter counterexamples on rendered SYNTHETIC data only.
Two solver processes at most; complete data/logs remain under --output.
A passed solver with a failed truth comparison is an expected filter warning,
not evidence that a physical flight controller has these exact filters.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np
from scipy.signal import butter, sosfilt

from verify_synthetic import ROOT, compare, generate


def run_review(output):
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use an empty output directory")
    output.mkdir(parents=True, exist_ok=True)
    print("Generating slow and high-frequency synthetic images...", flush=True)
    generate(output/"slow", seed=101, shift=0.037)
    generate(output/"fast", seed=202, shift=0.021, motion="fast")
    filtered = output/"filtered"
    filtered.mkdir()
    for name in ("camera.yaml", "target.yaml", "synthetic_truth.json"):
        shutil.copyfile(output/"slow"/name, filtered/name)
    # Manifest paths are relative to the variant, sharing the same images.
    with (output/"slow/images.csv").open() as src, (filtered/"images.csv").open("w", newline="") as dst:
        reader, writer = csv.reader(src), csv.writer(dst)
        writer.writerow(next(reader))
        writer.writerows((stamp, "../slow/"+name) for stamp, name in reader)
    with (output/"slow/imu.csv").open() as src:
        reader = csv.reader(src)
        header, rows = next(reader), list(reader)
    values = np.asarray([r[1:] for r in rows], dtype=float)
    for sl, cutoff in ((slice(3, 6), 20.), (slice(0, 3), 10.)):
        values[:, sl] = sosfilt(butter(2, cutoff, fs=200., output="sos"), values[:, sl], axis=0)
    with (filtered/"imu.csv").open("w", newline="") as dst:
        writer = csv.writer(dst)
        writer.writerow(header)
        writer.writerows((r[0], *v) for r, v in zip(rows, values))
    truth = json.loads((filtered/"synthetic_truth.json").read_text())
    truth["synthetic_causal_filter"] = {"gyro_cutoff_hz": 20., "accel_cutoff_hz": 10.,
                                       "model": "digital second-order Butterworth, fs=200Hz, zero-state, no timestamp correction"}
    (filtered/"synthetic_truth.json").write_text(json.dumps(truth, indent=2))
    cases = [("slow_default", "slow", []), ("filtered_default", "filtered", []),
             ("fast_coarse", "fast", ["--knot-spacing", "0.12", "--max-nfev", "120"]),
             ("fast_default", "fast", [])]

    def run(case):
        name, dataset, flags = case
        data, result = output/dataset, output/name
        cmd = [sys.executable, str(ROOT/"calibrate_extrinsics.py"), "--camera", str(data/"camera.yaml"),
               "--target", str(data/"target.yaml"), "--images", str(data/"images.csv"),
               "--imu", str(data/"imu.csv"), "--output", str(result), "--duration", "12", *flags]
        start = time.perf_counter()
        env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
        with (output/f"{name}.log").open("w") as log:
            proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, env=env)
        report = json.loads((result/"report.json").read_text())
        truth = json.loads((data/"synthetic_truth.json").read_text())
        row = {"case": name, "command": cmd, "returncode": proc.returncode,
               "elapsed_s": time.perf_counter()-start, "status": report["status"],
               "failure_reasons": report.get("failure_reasons"),
               "truth": truth, "metrics": report.get("final_metrics"),
               "estimated_shift_s": report.get("timeshift_cam_imu")}
        if "T_cam_imu" in report:
            row["comparison"] = compare(report, truth)
        (output/f"{name}_comparison.json").write_text(json.dumps(row, indent=2))
        print(json.dumps({k: v for k, v in row.items() if k not in ("command", "truth")}), flush=True)
        return row

    with ThreadPoolExecutor(max_workers=2) as pool:
        rows = list(pool.map(run, cases))
    (output/"summary.json").write_text(json.dumps(rows, indent=2))
    by_name = {r["case"]: r for r in rows}
    ok = all(by_name[k]["returncode"] == 0 and by_name[k]["comparison"]["passed"]
             for k in ("slow_default", "fast_default"))
    # Both are deliberate counterexamples: do not weaken the production gates.
    ok &= by_name["fast_coarse"]["returncode"] != 0
    f = by_name["filtered_default"]
    ok &= f["returncode"] == 0 and not f["comparison"]["passed"]
    return 0 if ok else 1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    return run_review(p.parse_args().output.resolve())


if __name__ == "__main__":
    sys.exit(main())
