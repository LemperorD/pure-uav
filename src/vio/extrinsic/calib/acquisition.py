"""Operator-provided acquisition provenance, not automatic sensor certification."""
from .data import CalibrationError, read_yaml

ENUMS = {"imu_processing": ("unknown", "raw", "filtered"),
         "clock_domain": ("unknown", "shared", "different"),
         "imu_instance": ("unknown", "single", "mixed")}
TEXT_FIELDS = ("firmware", "imu_topic", "gyro_instance", "accel_instance", "evidence")


def acquisition_preflight(path=None):
    data = {} if path is None else read_yaml(path)
    if not isinstance(data, dict):
        raise CalibrationError("采集说明 YAML 顶层必须为映射。")
    unknown = set(data)-set(ENUMS)-set(TEXT_FIELDS)
    if unknown:
        raise CalibrationError(f"未知采集说明项 {sorted(unknown)}")
    normalized = {k: data.get(k, "unknown") for k in ENUMS}
    for key, choices in ENUMS.items():
        if normalized[key] not in choices:
            raise CalibrationError(f"采集说明 {key} 必须为 {choices}")
    for key in TEXT_FIELDS:
        if key in data and not isinstance(data[key], str):
            raise CalibrationError(f"采集说明 {key} 必须为字符串")
    normalized.update({k: data[k] for k in TEXT_FIELDS if k in data})
    if normalized["imu_processing"] == "filtered":
        raise CalibrationError("已声明 IMU 经额外因果滤波：当前理想 IMU/单一时移模型不支持该处理链；请采集滤波前样本。")
    if normalized["clock_domain"] == "different":
        raise CalibrationError("已声明相机/IMU 时钟域不同：先验证映射/同步，不能靠同一个 bag 或任意 ±0.1s 搜索解决。")
    if normalized["imu_instance"] == "mixed":
        raise CalibrationError("已声明混用 IMU 实例：需采集同一固定实例的陀螺和加计。")
    gyro, accel = normalized.get("gyro_instance"), normalized.get("accel_instance")
    if gyro and accel and gyro != accel:
        raise CalibrationError("采集说明陀螺和加计实例 ID 不一致。")
    warnings = []
    if normalized["imu_processing"] == "unknown":
        warnings.append("未核实 IMU 滤波链；低残差不能排除滤波引入的时移/外参偏差。")
    if normalized["clock_domain"] == "unknown":
        warnings.append("未核实采样时钟域；同一 bag + header.stamp 不保证同一时钟域。")
    if normalized["imu_instance"] == "unknown":
        warnings.append("未核实 IMU 实例；时间戳严格递增不保证没有混用/切换实例。")
    return {"source": None if path is None else str(path.resolve()),
            "operator_declarations": normalized, "warnings": warnings,
            "verification": "operator-supplied only; not inferred from residuals or timestamps"}
