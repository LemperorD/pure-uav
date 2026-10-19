#!/usr/bin/env bash
# =============================================================================
#  Windows（MSYS2 / Git Bash）侧本地回归：编译 shim + 与原版 pn_sim 对轨迹
#
#  用途：改过 guidance_libs 之后，**不必进 WSL**，先在本机验证
#        “算法行为没变、shim 链路没坏”。需要：
#          · MSYS2 ucrt64 的 g++（在 PATH 里）
#          · Windows 版 Python（自动探测；可用 PYTHON=... 覆盖）
#
#  用法：  bash build_guidance_shim_mingw.sh
#  通过时会打印：“本机回归通过 ✅（shim 与 pn_sim 轨迹逐行一致）”
# =============================================================================
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
LIBS="$HERE/../src/guidance/guidance_libs"
# ⚠ pn_sim.cpp 在 src/guidance/**sim**/ 下，**不在** guidance_libs/sim/ 里
#   （原脚本写的是 "$LIBS/sim/pn_sim.cpp"，路径不存在 —— 2026-10-16 修正）
SIM="$HERE/../src/guidance/sim"
TMP="${TMPDIR:-/tmp}/pgu_build"
mkdir -p "$TMP"
cd "$TMP"

if command -v python >/dev/null 2>&1; then
    PY=python
else
    PY="C:/Users/wangyipeng/.workbuddy-ai/binaries/python/versions/3.13.12/python.exe"
fi

# 1) shim 动态库
#    源文件列表 = guidance_core(guidance/pn_law/quat) + guidance_sim(sensor/target)
#    ⚠ 新增源文件时这里要同步（quat.cpp 于 2026-10-16 从 sensor.cpp 拆出）
g++ -O2 -std=c++17 -shared -static -static-libgcc -static-libstdc++ \
    -I "$LIBS/include" \
    "$HERE/pure_guidance_shim.cpp" \
    "$LIBS/src/guidance.cpp" "$LIBS/src/pn_law.cpp" "$LIBS/src/quat.cpp" \
    "$LIBS/src/sensor.cpp"  "$LIBS/src/target.cpp" \
    -o pure_guidance.dll

# 2) 原版 pn_sim（对照基准）
g++ -O2 -std=c++17 -static -static-libgcc -static-libstdc++ \
    -I "$LIBS/include" \
    "$SIM/pn_sim.cpp" \
    "$LIBS/src/guidance.cpp" "$LIBS/src/pn_law.cpp" "$LIBS/src/quat.cpp" \
    "$LIBS/src/sensor.cpp"  "$LIBS/src/target.cpp" \
    -o pn_sim.exe

# 3) 生成基准轨迹 + 跑等价性对照
./pn_sim.exe ref.csv 6 PPN IDEAL dart > pn_sim.log
"$PY" "$HERE/test_guidance_shim.py" pure_guidance.dll --out shim.csv --ref ref.csv

echo
echo "本机回归通过 ✅（shim 与 pn_sim 轨迹逐行一致；日志见 $TMP/pn_sim.log）"
