# Orbbec Gemini Max

Gemini Max 使用 Orbbec Gemini 系列硬件，本项目采用 **Orbbec SDK v1.10.37**。

---

## 目录结构

```text
src/driver/orbbec/
├── include/          # Orbbec SDK v1.10.37 头文件
├── lib/              # Orbbec SDK v1.10.37 Linux x64 动态库
├── test_camera.cpp   # Gemini Max 测试程序
└── README.md
```

SDK 已包含在仓库中，不需要另外下载 Orbbec SDK。


## USB 权限

第一次使用时安装 Orbbec udev 规则。

在仓库根目录执行：

```bash
sudo ./scripts/install_udev_rules.sh
```

然后重新插拔 Gemini Max。

可以使用以下命令检查设备：

```bash
lsusb
```

---

## 编译测试程序

在仓库根目录执行：

```bash
g++ -std=c++17 \
  src/driver/orbbec/test_camera.cpp \
  -Isrc/driver/orbbec/include \
  -Lsrc/driver/orbbec/lib \
  -lOrbbecSDK \
  -o bin/orbbec_test
```

---

## 运行

运行格式：

```bash
LD_LIBRARY_PATH="$PWD/src/driver/orbbec/lib:$LD_LIBRARY_PATH" \
./bin/orbbec_test <COLOR_FPS> <DEPTH_FPS>
```

例如：

### Color 60 FPS + Depth 60 FPS

```bash
LD_LIBRARY_PATH="$PWD/src/driver/orbbec/lib:$LD_LIBRARY_PATH" \
./bin/orbbec_test 60 60
```

### Color 60 FPS + Depth 30 FPS

```bash
LD_LIBRARY_PATH="$PWD/src/driver/orbbec/lib:$LD_LIBRARY_PATH" \
./bin/orbbec_test 60 30
```

### Color 30 FPS + Depth 30 FPS

```bash
LD_LIBRARY_PATH="$PWD/src/driver/orbbec/lib:$LD_LIBRARY_PATH" \
./bin/orbbec_test 30 30
```

---

## 帧率配置


实机查询得到的可用 Profile：

| Stream | Resolution | Supported FPS |
|---|---:|---:|
| Color | 640x480 | 5 / 10 / 15 / 30 / 60 |
| Color | 1280x720 | 30 |
| Color | 1920x1080 | 30 |
| Depth | 320x200 | 5 / 10 / 15 / 30 / 60 |
| Depth | 640x400 | 5 / 10 / 15 / 30 / 60 |
| Depth | 1280x800 | 15 / 30 |
| IR | 320x200 | 5 / 10 / 15 / 30 / 60 |
| IR | 640x400 | 5 / 10 / 15 / 30 / 60 |
| IR | 1280x800 | 15 / 30 |

只能选择设备支持的分辨率和帧率组合。

例如：

```bash
./bin/orbbec_test 60 60
```

表示：

```text
Color: 640x480 @ 60 FPS
Depth: 640x400 @ 60 FPS
```

---

## 实际 FPS 统计

测试程序会同时显示：

- 配置的目标 FPS
- 实际接收到的 FPS

示例：

```text
Configured:
  Color: 640x480 @ 60 FPS
  Depth: 640x400 @ 60 FPS

Actual FPS | Color: 59.8 | Depth: 59.7
Actual FPS | Color: 60.1 | Depth: 59.9
```

---

## OrbbecViewer

OrbbecViewer 用于可视化调试：

- RGB
- Depth
- IR
- Point Cloud
- Stream Profile
- 相机参数

Viewer 不包含在本仓库中。

下载地址：

https://github.com/orbbec/OrbbecSDK/releases/tag/v1.10.37

下载 Linux x64 版本：

```text
OrbbecViewer_v1.10.37_*_linux_x64_release.zip
```

解压后运行：

```bash
./OrbbecViewer
```

如果遇到 OpenGL / GLFW 启动问题，可以先执行：

```bash
unset LIBGL_ALWAYS_INDIRECT
unset LIBGL_ALWAYS_SOFTWARE
```

然后重新运行：

```bash
./OrbbecViewer
```

---