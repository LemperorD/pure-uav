#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立验证 `gz_scene_sync.py` 用到的 gz-msgs 字段名 / 枚举名，是否与 WSL 里实际安装的
gz-msgs10 一致。

【为什么需要它】本机（Windows）**没有 Gazebo**，跑不了 `--demo`。而这类名字写错
是**运行时 AttributeError**，只有到 WSL 里才发现。所以这里直接读 WSL 里
`_pb2.py` 内嵌的 protobuf 描述符（二进制），自己解析出来核对 —— 不依赖
google.protobuf（本机没装），手写最小 wire-format 解码器。

【验证内容】
  1. 这些消息是否有我用的字段名（sdf / name / pose / allow_renaming / …）
  2. 这些枚举值名是否存在（**重点**：Marker.Action 里只有 ADD_MODIFY，
     没有 ADD —— 曾写错，这个检查就是为了不再犯）
  3. 打印实际枚举值，便于人工确认语义没串

用法：python verify_gz_msgs.py [WSL_msgs10_目录]
"""
import ast
import glob
import os
import re
import sys

# ⚠ 默认目录**自适应**：这个脚本既可能在 WSL 里跑（路径是 /usr/lib/...），
#   也可能在 Windows 侧跑（要读 WSL 的 UNC 路径 //wsl.localhost/...）。
#   （初版把 UNC 写死成默认值 ⇒ 在 WSL 里跑必然"文件不存在"，已修。）
_CANDIDATES = [
    "/usr/lib/python3/dist-packages/gz/msgs10",                      # WSL/Linux 本机
    "//wsl.localhost/Ubuntu-24.04/usr/lib/python3/dist-packages/gz/msgs10",  # Windows→WSL
    "/usr/local/lib/python3/dist-packages/gz/msgs10",
]


def _autodetect_dir():
    for d in _CANDIDATES:
        if os.path.isdir(d) and glob.glob(os.path.join(d, "*_pb2.py")):
            return d
    # 再退一步：按 gz/msgs<N> 的模式全局找
    for pat in ("/usr/lib/python3*/dist-packages/gz/msgs*/",
                "//wsl.localhost/*/usr/lib/python3/dist-packages/gz/msgs*/"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[-1]
    return _CANDIDATES[0]


DEFAULT_DIR = _autodetect_dir()

# ---- 我在 gz_scene_sync.py 里实际用到的名字（改动那边要同步这里）----
EXPECT_ENUMS = {
    "Marker.Action": ["ADD_MODIFY", "DELETE_ALL"],
    "Marker.Type": ["LINE_STRIP", "SPHERE"],
    "Entity.Type": ["MODEL"],
}
EXPECT_FIELDS = {
    "EntityFactory": ["sdf", "name", "pose", "allow_renaming"],
    "Pose": ["name", "position", "orientation"],
    "Quaternion": ["x", "y", "z", "w"],
    "Vector3d": ["x", "y", "z"],
    "Entity": ["name", "type"],
    "Marker": ["action", "ns", "id", "type", "lifetime", "pose", "scale",
               "material", "point"],
    "Color": ["r", "g", "b", "a"],
    "Material": ["ambient", "diffuse", "emissive"],
    # ★ `/world/<world>/pose/info` 发的是 Pose_V（一条消息带所有实体）
    #   2026-10-17 新增：用来「列出世界里有哪些模型」（--list-entities）
    "Pose_V": ["pose"],
}
# 这些是**消息类型**字段，用来确认 lifetime 不是 double 之类
EXPECT_MSGTYPE = {
    ("Marker", "lifetime"): "Time",
    ("Marker", "pose"): "Pose",
    ("Marker", "scale"): "Vector3d",
    ("Marker", "material"): "Material",
    ("Pose", "position"): "Vector3d",
    ("Pose", "orientation"): "Quaternion",
    ("Pose_V", "pose"): "Pose",          # 是**repeated Pose**
}

FILES = ["entity_factory", "pose", "quaternion", "vector3d", "entity",
         "marker", "color", "material", "time", "boolean", "pose_v"]


# ==========================================================================
#  最小 protobuf wire-format 解码器
# ==========================================================================
def _varint(buf, i):
    v = 0
    shift = 0
    while True:
        b = buf[i]
        i += 1
        v |= (b & 0x7F) << shift
        if not (b & 0x80):
            return v, i
        shift += 7


def _fields(buf):
    """把一段 message 解成 [(field_number, wire_type, value_or_bytes), …]。"""
    out = []
    i = 0
    n = len(buf)
    while i < n:
        key, i = _varint(buf, i)
        fn, wt = key >> 3, key & 7
        if wt == 0:
            v, i = _varint(buf, i)
            out.append((fn, wt, v))
        elif wt == 2:
            ln, i = _varint(buf, i)
            out.append((fn, wt, buf[i:i + ln]))
            i += ln
        elif wt == 5:
            out.append((fn, wt, buf[i:i + 4]))
            i += 4
        elif wt == 1:
            out.append((fn, wt, buf[i:i + 8]))
            i += 8
        else:
            raise ValueError(f"未知 wire_type={wt}")
    return out


def _str_of(blob):
    return blob.decode("utf-8", "replace")


def parse_descriptor(buf):
    """FileDescriptorProto → {消息名: {"fields": {name: type_name_or_None},
                                        "enums": {枚举名: {值名: 值}}}}"""
    msgs = {}
    root_name = None
    for fn, wt, val in _fields(buf):
        if fn == 1 and wt == 2:
            root_name = _str_of(val)
        elif fn == 4 and wt == 2:                       # message_type
            name, m = _parse_message(val)
            msgs[name] = m
    return root_name, msgs


def _parse_message(buf):
    name = None
    fields = {}          # 字段名 → 类型名（仅 message 类型才有）
    enums = {}

    # 先收集字段本身
    for fn, wt, val in _fields(buf):
        if fn == 1 and wt == 2:
            name = _str_of(val)
        elif fn == 2 and wt == 2:                       # field
            fname, ftype_name = None, None
            for f2, w2, v2 in _fields(val):
                if f2 == 1 and w2 == 2:
                    fname = _str_of(v2)
                elif f2 == 6 and w2 == 2:               # type_name（如 .gz.msgs.Time）
                    ftype_name = _str_of(v2).split(".")[-1]
            if fname:
                fields[fname] = ftype_name
    # 再收集嵌套枚举 / 嵌套消息
    for fn, wt, val in _fields(buf):
        if fn == 3 and wt == 2:                         # nested_type
            nname, nm = _parse_message(val)
            if nname:
                fields["__nested__" + nname] = nm
        elif fn == 4 and wt == 2:                       # enum_type
            ename, vals = _parse_enum(val)
            if ename:
                enums[ename] = vals
    return name, {"fields": fields, "enums": enums}


def _parse_enum(buf):
    name = None
    vals = {}
    for fn, wt, val in _fields(buf):
        if fn == 1 and wt == 2:
            name = _str_of(val)
        elif fn == 2 and wt == 2:                       # 枚举值
            vname, vnum = None, None
            for f2, w2, v2 in _fields(val):
                if f2 == 1 and w2 == 2:
                    vname = _str_of(v2)
                elif f2 == 2 and w2 == 0:
                    vnum = v2
            if vname:
                vals[vname] = vnum
    return name, vals


def extract_blob(path):
    """从 _pb2.py 里把 AddSerializedFile(b'...') 的字节串取出来。"""
    src = open(path, encoding="utf-8").read()
    m = re.search(r"AddSerializedFile\(\s*(b['\"].*?['\"])\s*\)", src, re.S)
    if not m:
        return None
    return ast.literal_eval(m.group(1))


# ==========================================================================
def main():
    d = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DIR
    print("=" * 74)
    print("独立验证 gz-msgs 字段名 / 枚举名（对着实际安装的描述符，不依赖 protobuf 库）")
    print(f"目录: {d}")
    if not os.path.isdir(d):
        print(f"\n❌ 目录不存在：{d}")
        print("   · 在 WSL 里跑：直接 `python3 verify_gz_msgs.py`（默认自动探测）")
        print("   · 在 Windows 跑：需能访问 WSL，或显式传入，例如")
        print("       python verify_gz_msgs.py "
              "//wsl.localhost/Ubuntu-24.04/usr/lib/python3/dist-packages/gz/msgs10")
        return 2
    print("=" * 74)

    all_msgs = {}
    ok_files, bad_files = 0, []
    for f in FILES:
        p = os.path.join(d, f + "_pb2.py")
        if not os.path.exists(p):
            bad_files.append(f + "（文件不存在）")
            continue
        blob = extract_blob(p)
        if blob is None:
            bad_files.append(f + "（未找到描述符）")
            continue
        _root, msgs = parse_descriptor(blob)
        if not msgs:
            bad_files.append(f + "（解析为空）")
            continue
        ok_files += 1
        all_msgs.update(msgs)

    print(f"\n解析成功 {ok_files}/{len(FILES)} 个消息文件"
          f"{'；失败：' + ', '.join(bad_files) if bad_files else ''}")

    fails = []

    # ---- 1) 字段名 ----
    print("\n" + "-" * 74)
    print("[1] 字段名检查")
    print("-" * 74)
    for msg, need in EXPECT_FIELDS.items():
        m = all_msgs.get(msg)
        if m is None:
            fails.append(f"找不到消息 {msg}")
            print(f"  ✗ {msg}：消息本身不存在")
            continue
        have = m["fields"]
        for f in need:
            if f in have:
                t = have[f]
                print(f"  ✓ {msg}.{f}" + (f"  (消息类型 {t})" if t else ""))
            else:
                print(f"  ✗ {msg}.{f}  —— 不存在！可用：{sorted(k for k in have if not k.startswith('__'))}")
                fails.append(f"{msg}.{f} 不存在")

    # ---- 2) 消息类型（如 lifetime 必须是 Time）----
    print("\n" + "-" * 74)
    print("[2] 字段的消息类型检查（防止把 message 当 double 用）")
    print("-" * 74)
    for (msg, fld), exp in EXPECT_MSGTYPE.items():
        have = all_msgs.get(msg, {}).get("fields", {}).get(fld)
        if have == exp:
            print(f"  ✓ {msg}.{fld} 是消息 {exp}")
        else:
            print(f"  ✗ {msg}.{fld} 实际是 {have!r}，期望 {exp}")
            fails.append(f"{msg}.{fld} 类型不符（{have} != {exp}）")

    # ---- 3) 枚举值名 ----
    print("\n" + "-" * 74)
    print("[3] 枚举值名检查  ★ 重点：Marker.Action 没有 ADD")
    print("-" * 74)
    for key, need in EXPECT_ENUMS.items():
        msg, ename = key.split(".")
        m = all_msgs.get(msg)
        if m is None:
            fails.append(f"找不到消息 {msg}")
            continue
        vals = m["enums"].get(ename)
        if vals is None:
            print(f"  ✗ {key}：枚举不存在；该消息的枚举有 {list(m['enums'])}")
            fails.append(f"{key} 枚举不存在")
            continue
        print(f"  {key} 实际值 = {vals}")
        for v in need:
            if v in vals:
                print(f"      ✓ {v} = {vals[v]}")
            else:
                print(f"      ✗ {v} 不存在！")
                fails.append(f"{key}.{v} 不存在")

    # ---- 汇总 ----
    print("\n" + "=" * 74)
    if fails:
        print(f"失败 {len(fails)} 项 ❌")
        for f in fails:
            print("  -", f)
    else:
        print("全部通过 ✅  —— gz_scene_sync.py 用到的名字与实际 gz-msgs 版本一致")
    print("=" * 74)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
