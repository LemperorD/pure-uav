# vendor/ —— 现成脚本的本地副本

这三个文件不是新写的，是**原样复制**的既有脚本；复制的原因只是让 `extrinsic/`
自包含，不依赖外部临时目录。

| 文件 | 来源 | 是否改过 |
|---|---|---|
| `capture_calibration.py` | 内参标定那套 V4L2 采集脚本（本仓库 `intrinsic/` 里的同名文件） | ❌ **改过一处，见下** |
| `calibration_config.py` | 同上 | ✅ 原版（与 `intrinsic/calibration_config.py` 逐位一致） |
| `mavlink_min.py` | 早期 MAVLink 最小实现 | ✅ 原版 |

## 本地修改：`capture_calibration.py:611`

```python
# 改前
if args.cols and (frames == 1 or now - last_detect_time >= 0.5):
# 改后
if args.cols and not args.no_preview and (frames == 1 or now - last_detect_time >= 0.5):
```

**为什么**：这段"每 0.5 秒做一次 1080p 棋盘检测"在预览关掉（`--no-preview`，
也就是正式录像）时毫无用处，却要 470~530 ms/次，把 29 fps 拖到 11 fps，
并在驱动层制造 330~400 ms 空洞 —— 直接触发标定侧的 `max_camera_gap_s`。
它只影响预览窗绿点、manifest 里的 `board_detected`、3×3 覆盖度提示；
**离线标定用的是 `calib/data.py` 自己的检测器，不受影响**。

> `intrinsic/capture_calibration.py` 保留原版（它开着预览用，碰不到这条）。
> 两份内容除这一行外完全一致。

## 为什么值得复制

`capture_calibration.py` 做了 687 行很扎实的事，重写没有好处：

- 校验 USB 身份 `2bc5:0511`，**拒绝误用笔记本内置摄像头**
- 锁定曝光 / 白平衡 / 增益 / 锐化（标定期间必须固定，否则角点会漂）
- V4L2 直采（`mmap` + `VIDIOC_DQBUF`），不是 `cv2.VideoCapture` 那层封装
- 断线自动重连，重连后重新锁定控件
- 文件名就是宿主墙钟纳秒时间戳，另写一份 `capture_manifest.json`
- 退出时恢复相机原始控件

`capture/` 里新写的代码只做三件事：录 IMU、把 manifest 转成 `images.csv`、把两者编排起来。

## 注意

`calibration_config.py` 是 `capture_calibration.py` 的默认参数文件，
里面的 `DEVICE` / `WIDTH` / `HEIGHT` / `SESSION_DIR` 等**不会生效** ——
`record_session.py` 会用 `--device/--output/--width/--height/...` 全部显式覆盖，
真正的参数在 `../capture_config.py`。
