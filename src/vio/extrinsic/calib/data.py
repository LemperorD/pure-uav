"""Explicit timestamps/units, target detection, PnP, ROS-free bag loading."""
import csv
import json
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


class CalibrationError(RuntimeError):
    pass


def read_yaml(path):
    # OpenCV writes a nonstandard %YAML:1.0 directive; safe_load needs it removed.
    text = Path(path).read_text(encoding="utf-8-sig")
    text = "\n".join(line for line in text.splitlines() if not line.startswith("%YAML:"))
    return yaml.safe_load(text)


def stamp_ns(value, unit):
    scale = {"ns": 1, "us": 1000, "ms": 1000000, "s": 1000000000}[unit]
    number = Decimal(str(value))*scale
    if not number.is_finite() or number != number.to_integral_value():
        raise CalibrationError(f"时间戳 {value!r} 不能按 {unit} 表达为整数纳秒。")
    return int(number)


def check_stamps(stamps, name):
    a = np.asarray(stamps, dtype=np.int64)
    if len(a) < 2 or np.any(np.diff(a) <= 0):
        raise CalibrationError(f"{name}: 至少2个严格递增时间戳；不允许重复、乱序或飞控时钟重启。")
    return a


@dataclass
class Camera:
    entry: dict
    name: str = "cam0"

    def __post_init__(self):
        if self.entry.get("camera_model") != "pinhole" or self.entry.get("distortion_model") != "radtan":
            raise CalibrationError("本实现只支持 pinhole-radtan；其他模型不得冒充此模型输入。")
        self.params = np.asarray(self.entry["intrinsics"], float)
        self.dist = np.asarray(self.entry["distortion_coeffs"], float)
        self.size = tuple(int(v) for v in self.entry["resolution"])
        if (self.params.shape != (4,) or len(self.dist) not in (4, 5)
                or not np.all(np.isfinite(np.r_[self.params, self.dist]))
                or np.any(self.params[:2] <= 0) or len(self.size) != 2 or min(self.size) <= 0):
            raise CalibrationError("内参格式/数值无效。")
        fx, fy, cx, cy = self.params
        self.k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]])

    @classmethod
    def load(cls, path, name="cam0"):
        entries = read_yaml(path)
        if name not in entries:
            raise CalibrationError(f"内参 YAML 没有 {name}。")
        return cls(entries[name], name)

    def project(self, xyz_c, *, trial_depth_floor=None):
        """Physical projection rejects nonpositive depth.

        An optimizer may request a finite continuation for trial steps ONLY;
        it must separately penalize depths below this floor and reject an
        invalid final solution. This continuation is not a physical image.
        """
        xyz_c = np.asarray(xyz_c, dtype=float)
        if xyz_c.ndim != 2 or xyz_c.shape[1] != 3 or not np.all(np.isfinite(xyz_c)):
            raise CalibrationError("投影输入必须为有限 N×3 点。")
        z = xyz_c[:, 2]
        if trial_depth_floor is None:
            if np.any(z <= 0):
                raise CalibrationError("不能投影相机后方/零深度点。")
        else:
            if not np.isfinite(trial_depth_floor) or trial_depth_floor <= 0:
                raise CalibrationError("trial_depth_floor 必须为有限正数。")
            z = np.maximum(z, trial_depth_floor)
        xy = xyz_c[:, :2] / z[:, None]
        x, y = xy.T
        r2 = x*x+y*y
        k1, k2, p1, p2 = self.dist[:4]
        k3 = self.dist[4] if len(self.dist) == 5 else 0.
        scale = 1+k1*r2+k2*r2*r2+k3*r2**3
        xd = x*scale+2*p1*x*y+p2*(r2+2*x*x)
        yd = y*scale+p1*(r2+2*y*y)+2*p2*x*y
        fx, fy, cx, cy = self.params
        return np.column_stack((fx*xd+cx, fy*yd+cy))


class Target:
    def __init__(self, entry, min_tags=4, april_corner_window_px=2):
        self.entry = entry
        self.kind = entry["target_type"]
        self.min_tags = min_tags
        if self.kind in ("checkerboard", "chessboard"):
            self.cols = int(entry.get("targetCols", entry.get("cols", 0)))
            self.rows = int(entry.get("targetRows", entry.get("rows", 0)))
            sx = float(entry.get("colSpacingMeters", entry.get("square_size", 0)))
            sy = float(entry.get("rowSpacingMeters", entry.get("square_size", 0)))
            if min(self.cols, self.rows) < 3 or min(sx, sy) <= 0:
                raise CalibrationError("棋盘格需>=3x3内角点，格边长以米为单位且>0。")
            self.points = np.zeros((self.cols*self.rows, 3))
            self.points[:, :2] = np.mgrid[:self.cols, :self.rows].T.reshape(-1, 2)*[sx, sy]
        elif self.kind == "aprilgrid":
            self.cols, self.rows = int(entry["tagCols"]), int(entry["tagRows"])
            self.size, self.spacing = float(entry["tagSize"]), float(entry["tagSpacing"])
            self.first_id = int(entry.get("first_id", 0))
            if min(self.rows, self.cols) < 1 or self.size <= 0 or self.spacing < 0:
                raise CalibrationError("AprilGrid 行列/尺寸/间距无效。")
            if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "DICT_APRILTAG_36h11"):
                raise CalibrationError("AprilGrid 需要 OpenCV aruco（安装 opencv-contrib-python>=4.7）。")
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            if self.first_id < 0 or self.first_id+self.rows*self.cols > len(dictionary.bytesList):
                raise CalibrationError("AprilGrid tag ID 超出 36h11 字典。")
            params = cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, "DetectorParameters") else cv2.aruco.DetectorParameters_create()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            params.cornerRefinementWinSize = int(april_corner_window_px)  # avoid code-bit edges on small tags
            params.cornerRefinementMaxIterations = 40
            params.cornerRefinementMinAccuracy = 0.01
            self.detector = cv2.aruco.ArucoDetector(dictionary, params) if hasattr(cv2.aruco, "ArucoDetector") else None
            self.dictionary, self.detector_params = dictionary, params
        else:
            raise CalibrationError(f"不支持的标定板 {self.kind}")

    def april_points(self, tag_id):
        # Kalibr board: IDs grow left->right, bottom->top; x right, y up.
        # OpenCV detected marker corners are canonical TL,TR,BR,BL.
        idx = int(tag_id)-self.first_id
        x = (idx % self.cols)*self.size*(1+self.spacing)
        y = (idx // self.cols)*self.size*(1+self.spacing)
        s = self.size
        return np.array([[x, y+s, 0], [x+s, y+s, 0], [x+s, y, 0], [x, y, 0]], float)

    def detect(self, gray):
        if self.kind in ("checkerboard", "chessboard"):
            ok, pts = cv2.findChessboardCornersSB(gray, (self.cols, self.rows), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not ok:
                ok, pts = cv2.findChessboardCorners(gray, (self.cols, self.rows))
                if ok:
                    pts = cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1),
                                          (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4))
            return (self.points.copy(), pts.reshape(-1, 2).astype(float)) if ok else None
        if self.detector is None:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_params)
        else:
            corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        valid = [(int(i), c.reshape(4, 2)) for i, c in zip(ids.ravel(), corners)
                 if self.first_id <= int(i) < self.first_id+self.rows*self.cols]
        if len({i for i, _ in valid}) != len(valid):
            raise CalibrationError("检测到重复 tag ID；请移走额外 AprilTag。")
        if len(valid) < self.min_tags:
            return None
        valid.sort(key=lambda v: v[0])
        return np.concatenate([self.april_points(i) for i, _ in valid]), np.concatenate([c for _, c in valid]).astype(float)


@dataclass
class Observation:
    stamp: int
    xyz: np.ndarray
    uv: np.ndarray
    r_wc: np.ndarray
    p_wc: np.ndarray
    pnp_rms: float


def pnp_candidates(xyz, uv, camera):
    out = cv2.solvePnPGeneric(xyz, uv, camera.k, camera.dist, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    if out[0]:
        for rv, tv in zip(out[1], out[2]):
            r_cw = Rotation.from_rotvec(rv.ravel()).as_matrix()
            c = xyz @ r_cw.T + tv.ravel()
            if np.min(c[:, 2]) <= 0:
                continue
            rms = float(np.sqrt(np.mean(np.sum((camera.project(c)-uv)**2, axis=1))))
            candidates.append((rms, r_cw.T, -r_cw.T @ tv.ravel()))
    return candidates


def make_observation(stamp, xyz, uv, camera, previous=None, checker=False, max_points=100):
    # An unmarked checkerboard has a 180-degree index ambiguity. Evaluate BOTH
    # orders and choose the temporally continuous pose; reject large gaps later.
    candidates = []
    for ordered in ([uv, uv[::-1]] if checker and previous is not None else [uv]):
        for rms, r, p in pnp_candidates(xyz, ordered, camera):
            score = rms
            if previous is not None:
                score += 2*Rotation.from_matrix(previous.r_wc.T @ r).magnitude()
                score += np.linalg.norm(p-previous.p_wc)
            candidates.append((score, rms, r, p, ordered))
    if not candidates:
        return None
    _, rms, r, p, ordered = min(candidates, key=lambda c: c[0])
    if max_points and len(xyz) > max_points:
        idx = np.unique(np.linspace(0, len(xyz)-1, max_points).astype(int))
        xyz, ordered = xyz[idx], ordered[idx]
    return Observation(stamp, xyz.copy(), ordered.copy(), r, p, rms)


def read_imu_csv(path, unit):
    stamps, values = [], []
    with Path(path).open(encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(line for line in f if not line.startswith("#"))
        required = ["timestamp", "ax", "ay", "az", "gx", "gy", "gz"]
        if not set(required).issubset(reader.fieldnames or []):
            raise CalibrationError("IMU CSV 表头必须含 timestamp,ax,ay,az,gx,gy,gz。")
        for row in reader:
            stamps.append(stamp_ns(row["timestamp"], unit))
            values.append([float(row[k]) for k in required[1:]])
    a = np.asarray(values, float)
    if not np.all(np.isfinite(a)):
        raise CalibrationError("IMU 含 NaN/Inf。")
    return check_stamps(stamps, "IMU"), a


def image_manifest(path, unit):
    path = Path(path)
    out = []
    with path.open(encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(line for line in f if not line.startswith("#"))
        if not {"timestamp", "filename"}.issubset(reader.fieldnames or []):
            raise CalibrationError("图像清单表头必须是 timestamp,filename。")
        for row in reader:
            out.append((stamp_ns(row["timestamp"], unit), path.parent / row["filename"]))
    check_stamps([s for s, _ in out], "图像")
    return out


def decode_ros_image(msg):
    data = np.asarray(msg.data, dtype=np.uint8)
    if hasattr(msg, "format"):
        image = cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise CalibrationError("无法解码 CompressedImage。")
        return image
    channels = {"mono8": 1, "8UC1": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
    if msg.encoding not in channels:
        raise CalibrationError(f"不支持 ROS 图像编码 {msg.encoding}；需要 mono8/rgb8/bgr8。")
    n = channels[msg.encoding]
    a = data.reshape(msg.height, msg.step)[:, :msg.width*n].reshape(msg.height, msg.width, n)
    if n == 1:
        return a[:, :, 0].copy()
    code = {"rgb8": cv2.COLOR_RGB2GRAY, "bgr8": cv2.COLOR_BGR2GRAY,
            "rgba8": cv2.COLOR_RGBA2GRAY, "bgra8": cv2.COLOR_BGRA2GRAY}[msg.encoding]
    return cv2.cvtColor(a, code)


def ros_stamp(msg):
    # Header, never rosbag record time. Integer arithmetic preserves ns epoch precision.
    return int(msg.header.stamp.sec)*1000000000 + int(msg.header.stamp.nanosec)


def load_dataset(camera, target, settings, output, *, images=None, imu=None, bag=None,
                 image_unit="ns", imu_unit="ns", cam_topic="/cam0/image_raw", imu_topic="/imu0"):
    observations, extraction = [], []
    output = Path(output)
    origin, last_kept, last_raw = None, None, None

    def process(stamp, image_getter):
        nonlocal origin, last_kept, last_raw
        if last_raw is not None and stamp <= last_raw:
            raise CalibrationError("图像 header 时间戳重复/乱序。")
        last_raw = stamp
        if origin is None:
            origin = stamp
        t = (stamp-origin)*1e-9
        if t < settings.start_s or t > settings.start_s+settings.max_duration_s:
            return
        if last_kept is not None and (stamp-last_kept)*1e-9 < 0.99/settings.camera_rate_hz:
            return
        last_kept = stamp
        gray = image_getter()
        if gray is None:
            raise CalibrationError(f"图像 {stamp} 无法读取。")
        if gray.shape[::-1] != camera.size:
            raise CalibrationError(f"图像分辨率 {gray.shape[::-1]} != 内参 {camera.size}；禁止自动缩放内参。")
        result = target.detect(gray)
        obs = None if result is None else make_observation(
            stamp, *result, camera, observations[-1] if observations else None,
            checker=target.kind in ("checkerboard", "chessboard"), max_points=settings.max_points_per_frame)
        extraction.append({"timestamp_ns": stamp, "detected": result is not None,
                           "pnp_rms_px": None if obs is None else obs.pnp_rms})
        if obs is None or obs.pnp_rms > settings.max_pnp_rms_px:
            return
        observations.append(obs)
        if settings.save_detection_every and (len(observations)-1) % settings.save_detection_every == 0:
            vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            for point in obs.uv:
                cv2.circle(vis, tuple(np.rint(point).astype(int)), 3, (0, 255, 0), 1)
            folder = output / "detections"
            folder.mkdir(parents=True, exist_ok=True)
            cv2.imencode(".jpg", vis)[1].tofile(str(folder / f"{stamp}.jpg"))

    if bag:
        try:
            from rosbags.highlevel import AnyReader
        except ImportError as exc:
            raise CalibrationError("读 bag 需要 rosbags；使用 pip install rosbags，无需 ROS/roscore。") from exc
        stamps, vals, frame_ids = [], [], set()
        with AnyReader([Path(bag)]) as reader:
            topics = {c.topic for c in reader.connections}
            if not {cam_topic, imu_topic}.issubset(topics):
                raise CalibrationError(f"bag 缺少话题；实际话题 {sorted(topics)}")
            connections = [c for c in reader.connections if c.topic in (cam_topic, imu_topic)]
            for conn, _record_time, raw in reader.messages(connections=connections):
                msg = reader.deserialize(raw, conn.msgtype)
                stamp = ros_stamp(msg)
                if conn.topic == imu_topic:
                    frame_ids.add(msg.header.frame_id)
                    if len(frame_ids) > 1:
                        raise CalibrationError("IMU header.frame_id 在记录中变化；请核查坐标系/传感器实例，拒绝混用。")
                    a, w = msg.linear_acceleration, msg.angular_velocity
                    stamps.append(stamp)
                    vals.append([a.x, a.y, a.z, w.x, w.y, w.z])
                else:
                    process(stamp, lambda msg=msg: decode_ros_image(msg))
        imu_stamps, values = check_stamps(stamps, "IMU header"), np.asarray(vals)
    else:
        if not images or not imu:
            raise CalibrationError("需提供 --bag 或同时提供 --images 和 --imu。")
        imu_stamps, values = read_imu_csv(imu, imu_unit)
        for stamp, path in image_manifest(images, image_unit):
            process(stamp, lambda path=path: cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_GRAYSCALE))
    output.mkdir(parents=True, exist_ok=True)
    (output / "extraction.json").write_text(json.dumps(extraction, indent=2, ensure_ascii=False), encoding="utf-8")
    if len(observations) < settings.min_frames:
        raise CalibrationError(f"有效标定板帧只有 {len(observations)}，至少 {settings.min_frames}。"
                               "请检查标定板类型/尺寸/清晰度；普通飞行场景不可用于此目标板算法。")
    if not np.all(np.isfinite(values)):
        raise CalibrationError("IMU 含 NaN/Inf。")
    # Subtract integer epoch BEFORE float conversion, using a common origin.
    epoch = observations[0].stamp
    tc = np.array([(o.stamp-epoch)*1e-9 for o in observations])
    ti = (imu_stamps-epoch).astype(float)*1e-9
    return observations, tc, ti, values, epoch
