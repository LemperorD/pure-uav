# 测试与基线

> 本文记录“怎么跑、跑什么、当前基线是多少”。
> 代码里的测试文件只保留简短说明 + 指向本文档。

---

## 1 测试目标

| 可执行文件 | 覆盖 | 当前基线 |
|---|---|---|
| `bin/test/test_guidance` | 制导算法 + adapter + 主循环 + CSV + 参数 + 打点 + 黑匣子 | **304 通过** |
| `bin/test/test_state_adapter` | 坐标/时间/姿态换算 + 组装 + 体检 | **1193 通过** |
| `bin/test/test_mavlink_units` | MAVLink 单位换算 + 回绕/哨兵 | **120 通过** |
| `bin/test/test_math` | `pure_math_tools` 冒烟 | 运行正常 |
| `bin/test/test_camera` | Orbbec 取流 | 需要相机/SDK |
| `bin/test/test_command` / `test_communicate` / `test_frequency` | 通信 | 需要串口/飞控 |

段覆盖（`test_guidance`）：

- §1–§15：算法、ω、Vc、三律、铁律、冻结、传感器墙、目标模型、不变量；
- §16：主循环外壳（像素反投影、降级、重发、统计）；
- §17：合成世界闭环回归；
- §18：CSV 录制 → 回放逐位一致 + 句柄生命周期；
- §19：参数加载/校验/打印；
- §20：端到端打点；
- §21：黑匣子（环形缓冲 + 时间窗 dump）。

---

## 2 在 WSL 里跑（推荐）

```bash
cd /path/to/pure-uav-main

# 配置（WSL 没有 Ninja 时用 Unix Makefiles）
cmake -S . -B build -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release

# 只编测试需要的 target
cmake --build build --target test_guidance test_state_adapter test_mavlink_units --parallel 4

# 运行动态库在 lib/
export LD_LIBRARY_PATH="$PWD/lib:$LD_LIBRARY_PATH"

./bin/test/test_guidance
./bin/test/test_state_adapter
./bin/test/test_mavlink_units
```

期望末尾：

```text
通过 304, 失败 0
ALL PASS
```

---

## 3 Windows / MSYS2 注意

- `communicate` 依赖 `yaml-cpp`；MSYS2 UCRT64 需要安装 `mingw-w64-ucrt-x86_64-yaml-cpp`；
- `test_camera` 链接 Orbbec SDK；如果仓库里的 `libOrbbecSDK.so` 是从 Linux 符号链接检出成文本文件，会报 `file format not recognized`；
- 因此 Windows 下建议只编/跑 `test_guidance`、`test_state_adapter`、`test_mavlink_units`，相机/通信测试在 WSL 或目标机上跑。

---

## 4 常见失败排查

| 现象 | 原因 |
|---|---|
| `libguidance_core.so: cannot open` | 没设 `LD_LIBRARY_PATH` |
| `test_guidance` 某些段失败 | 先看段标题，再对照 `assets/docs/guidance/后续注意问题.md` 的坑 |
| `test_camera` 链接失败 | Orbbec `.so` 不是有效 ELF / 缺 SDK |
| `test_command` 打不开串口 | 没有飞控/权限 |
| CMake 找不到 yaml-cpp | 安装 MSYS2 yaml-cpp 或用 WSL |

---

## 5 加测试的规则

1. 新测试放在对应 `src/test/test_*` 下，保持“纯 main + 轻量断言”风格；
2. 新增断言后更新本文档的基线数字；
3. 算法/接口改动必须跑 `test_guidance` + `test_state_adapter`；
4. 不要为了通过测试改基线；先判断是代码错还是测试预期错；
5. 测试里不写日期/emoji，场景说明写在段标题/注释里。

---

## 6 基线维护

- 当前基线：**304 / 1193 / 120**（2026-11-01，WSL GCC 13.3）；
- 改基线时在 `changelog.md` 记一笔；
- CI 建议：只跑前三个（无硬件依赖），`test_camera`/通信测试单独在目标机跑。
