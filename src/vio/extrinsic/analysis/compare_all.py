#!/usr/bin/env python3
"""Compare the self-written calibration against Kalibr, same-session and cross-session.

Conventions (both pipelines): p_C = T_cam_imu @ p_I ; t_imu = t_cam + timeshift_cam_imu.
"""
from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
SELF = ROOT / "results"
KAL = ROOT / "kalib" / "results"


def load_yaml_matrix(path: Path):
    d = yaml.safe_load(path.read_text())
    cam = d["cam0"]
    T = np.array(cam["T_cam_imu"], dtype=float)
    ts = float(cam.get("timeshift_cam_imu", 0.0))
    intr = np.array(cam["intrinsics"], dtype=float)
    dist = np.array(cam["distortion_coeffs"], dtype=float)
    return T, ts, intr, dist


def rot_angle_deg(R1, R2):
    c = (np.trace(R1 @ R2.T) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def rot_vec(R):
    return np.degrees(np.array([np.arctan2(R[2, 1], R[2, 2]),
                                np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0])),
                                np.arctan2(R[1, 0], R[0, 0])]))


def parse_kalibr_txt(path: Path):
    txt = path.read_text()
    out = {}
    m = re.search(r"Reprojection error \(cam0\) \[px\]:\s+mean ([-\d.eE+]+)", txt)
    if m:
        out["reproj_px"] = float(m.group(1))
    m = re.search(r"Gyroscope error \(imu0\) \[rad/s\]:\s+mean ([-\d.eE+]+)", txt)
    if m:
        out["gyro"] = float(m.group(1))
    m = re.search(r"Accelerometer error \(imu0\) \[m/s\^2\]:\s+mean ([-\d.eE+]+)", txt)
    if m:
        out["accel"] = float(m.group(1))
    m = re.search(r"timeshift cam0 to imu0: \[s\] \(t_imu = t_cam \+ shift\)\s*\n\s*([-\d.eE+]+)", txt)
    if m:
        out["timeshift"] = float(m.group(1))
    m = re.search(r"Gravity vector in target coords: \[m/s\^2\]\s*\n\s*\[([^\]]+)\]", txt)
    if m:
        out["gravity"] = np.array([float(x) for x in m.group(1).split()])
    return out


def collect():
    selfs = {}
    for rep in sorted(SELF.glob("session*/report.json")):
        sess = rep.parent.name
        chain = rep.parent / "camchain-kalibr.yaml"
        d = json.loads(rep.read_text())
        if not chain.exists():
            selfs[sess] = {"status": d.get("status"), "no_chain": True,
                           "final": d.get("final_metrics", {}),
                           "counts": d.get("counts", {}),
                           "duration_s": d.get("duration_s")}
            continue
        T, ts, intr, dist = load_yaml_matrix(chain)
        selfs[sess] = {"status": d.get("status"), "T": T, "ts": ts,
                       "intr": intr, "dist": dist,
                       "final": d.get("final_metrics", {}),
                       "initial": d.get("initial_metrics", {}),
                       "counts": d.get("counts", {}),
                       "duration_s": d.get("duration_s"),
                       "settings": d.get("settings", {}),
                       "excitation": d.get("excitation", {}),
                       "limitations": d.get("limitations", [])}
    kals = {}
    for d in sorted(KAL.glob("session*")):
        chain = d / f"{d.name}-camchain-imucam.yaml"
        txt = d / f"{d.name}-results-imucam.txt"
        if not chain.exists():
            continue
        T, ts, intr, dist = load_yaml_matrix(chain)
        kals[d.name] = {"T": T, "ts": ts, "intr": intr, "dist": dist,
                        "metrics": parse_kalibr_txt(txt) if txt.exists() else {}}
    return selfs, kals


def main():
    selfs, kals = collect()

    print("=" * 100)
    print("PHASE 1 — self-written results inventory")
    print("=" * 100)
    print(f"{'session':<16}{'status':<46}{'frames':>7}{'dur[s]':>8}{'reproj':>8}{'gyro':>9}{'accel':>8}{'tshift[ms]':>11}")
    for s, r in selfs.items():
        st = r.get("status", "?")
        fm = r.get("final", {})
        c = r.get("counts", {})
        if r.get("no_chain"):
            print(f"{s:<16}{str(st):<46}{'-':>7}{'-':>8}{'-':>8}{'-':>9}{'-':>8}{'-':>11}")
        else:
            print(f"{s:<16}{str(st):<46}{c.get('frames','-'):>7}{r.get('duration_s',0):>8.2f}"
                  f"{fm.get('reprojection_rms_px',float('nan')):>8.2f}"
                  f"{fm.get('gyro_rms_rad_s',float('nan')):>9.4f}"
                  f"{fm.get('accel_rms_m_s2',float('nan')):>8.3f}"
                  f"{1000*r.get('ts',0):>11.2f}")

    print()
    print("=" * 100)
    print("PHASE 2 — Kalibr results inventory")
    print("=" * 100)
    print(f"{'session':<16}{'reproj[px]':>11}{'gyro':>9}{'accel':>9}{'tshift[ms]':>12}{'|g|':>7}")
    for s, r in kals.items():
        m = r["metrics"]
        g = m.get("gravity")
        print(f"{s:<16}{m.get('reproj_px',float('nan')):>11.2f}{m.get('gyro',float('nan')):>9.4f}"
              f"{m.get('accel',float('nan')):>9.4f}{1000*m.get('timeshift',float('nan')):>12.2f}"
              f"{np.linalg.norm(g) if g is not None else float('nan'):>7.3f}")

    # intrinsics identity check
    print()
    print("=" * 100)
    print("PHASE 3 — intrinsics used by each pipeline")
    print("=" * 100)
    ref = None
    for name, r in list(selfs.items()) + list(kals.items()):
        if "intr" not in r:
            continue
        key = np.round(r["intr"], 6).tolist()
        if ref is None:
            ref = key
        same = "SAME" if key == ref else "DIFF"
        print(f"{name:<16}{same}  fx={r['intr'][0]:.4f} fy={r['intr'][1]:.4f} "
              f"cx={r['intr'][2]:.4f} cy={r['intr'][3]:.4f}")

    # same-session comparison
    print()
    print("=" * 100)
    print("PHASE 4 — SAME-SESSION comparison (self vs Kalibr)")
    print("=" * 100)
    both = [s for s in selfs if s in kals and not selfs[s].get("no_chain")]
    rows = []
    for s in both:
        a, b = selfs[s], kals[s]
        dR = rot_angle_deg(a["T"][:3, :3], b["T"][:3, :3])
        dt = (a["T"][:3, 3] - b["T"][:3, 3]) * 1000.0
        dts = (a["ts"] - b["ts"]) * 1000.0
        rows.append((s, dR, np.linalg.norm(dt), dt, dts,
                     a["final"].get("reprojection_rms_px"),
                     a["final"].get("gyro_rms_rad_s"),
                     b["metrics"].get("reproj_px"),
                     b["metrics"].get("gyro")))
    print(f"{'session':<10}{'dRot[deg]':>11}{'|dT|[mm]':>10}{'dx':>8}{'dy':>8}{'dz':>8}{'dShift[ms]':>12}")
    for s, dR, dtn, dt, dts, *_ in rows:
        print(f"{s:<10}{dR:>11.3f}{dtn:>10.2f}{dt[0]:>8.2f}{dt[1]:>8.2f}{dt[2]:>8.2f}{dts:>12.2f}")
    if rows:
        arr = np.array([[r[1], r[2], r[4]] for r in rows])
        print(f"{'MEAN':<10}{arr[:,0].mean():>11.3f}{arr[:,1].mean():>10.2f}{'':>8}{'':>8}{'':>8}{arr[:,2].mean():>12.2f}")
        print(f"{'STD':<10}{arr[:,0].std():>11.3f}{arr[:,1].std():>10.2f}{'':>8}{'':>8}{'':>8}{arr[:,2].std():>12.2f}")

    # pairwise cross-session consistency
    def pairwise(name, dct):
        keys = [k for k in dct if "T" in dct[k]]
        print()
        print(f"--- {name}: pairwise across {len(keys)} sessions ---")
        print(f"{'pair':<26}{'dRot[deg]':>11}{'dT[mm]':>9}{'dShift[ms]':>12}")
        vals = []
        for i in range(len(keys)):
            for j in range(i + 1, len(keys)):
                a, b = dct[keys[i]], dct[keys[j]]
                dR = rot_angle_deg(a["T"][:3, :3], b["T"][:3, :3])
                dt = np.linalg.norm(a["T"][:3, 3] - b["T"][:3, 3]) * 1000
                dts = (a["ts"] - b["ts"]) * 1000
                vals.append((dR, dt, dts))
                print(f"{keys[i][-12:]+' vs '+keys[j][-12:]:<26}{dR:>11.3f}{dt:>9.2f}{dts:>12.2f}")
        if vals:
            v = np.array(vals)
            print(f"{'median':<26}{np.median(v[:,0]):>11.3f}{np.median(v[:,1]):>9.2f}{np.median(v[:,2]):>12.2f}")
            print(f"{'max':<26}{v[:,0].max():>11.3f}{v[:,1].max():>9.2f}{v[:,2].max():>12.2f}")

    pairwise("SELF", selfs)
    pairwise("KALIBR", kals)

    # detailed per-session transforms
    print()
    print("=" * 100)
    print("PHASE 5 — transforms")
    print("=" * 100)
    for s in sorted(set(list(selfs) + list(kals))):
        print(f"--- {s}")
        for name, dct in (("self", selfs), ("kalibr", kals)):
            if s in dct and "T" in dct[s]:
                T = dct[s]["T"]
                ax = T[:3, 0]; ay = T[:3, 1]
                print(f"  {name:<7} t=[{T[0,3]*1000:7.2f} {T[1,3]*1000:7.2f} {T[2,3]*1000:7.2f}] mm  "
                      f"shift={1000*dct[s]['ts']:+7.2f} ms")
                print(f"          R rows: " + " | ".join("[" + " ".join(f"{x:+.4f}" for x in row) + "]" for row in T[:3, :3]))

    # save machine-readable summary
    out = {
        "self": {s: {"T": r["T"].tolist(), "ts": r["ts"], "final": r.get("final", {}),
                     "duration_s": r.get("duration_s"), "counts": r.get("counts", {})}
                 for s, r in selfs.items() if "T" in r},
        "kalibr": {s: {"T": r["T"].tolist(), "ts": r["ts"], "metrics": r["metrics"]}
                   for s, r in kals.items()},
    }
    outp = ROOT / "analysis" / "summary.json"
    outp.write_text(json.dumps(out, indent=2, default=float))
    print(f"\nsaved {outp}")


if __name__ == "__main__":
    main()
