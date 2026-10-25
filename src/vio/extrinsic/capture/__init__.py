"""采集链（相机 + 飞控 IMU）与采集数据的体检 / 转换工具。

同包互引统一写 `from capture import xxx`，vendor 代码写
`from capture.vendor.mavlink_min import ...`；唯一需要上 sys.path 的是
包根 `extrinsic/`（各入口脚本开头那两行）。
"""
