#!/usr/bin/env python3
"""
ArduPilot DataFlash (.bin) log reader.

The format is self-describing, which is what makes a reader tractable without
any external spec: every file starts with FMT messages that declare, for each
message type id, its name, its binary format string, and its column names. The
reader therefore learns the layout from the file itself rather than from a
hard-coded table.

    frame: 0xA3 0x95 <type u8> <payload of Length bytes>
    FMT payload: Type u8, Length u8, Name char[4], Format char[16], Columns char[64]

Format characters follow the usual ArduPilot set:
    a int16[32]  b int8   B uint8   h int16  H uint16
    i int32      I uint32  f float  d double  n char[4]  N char[16]
    Z char[64]   c int16*100  C uint16*100  e int32*100  E uint32*100
    L int32 lat/lon*1e7       M uint8 mode    q int64   Q uint64

Usage:
    python3 logparse.py <file.bin> [--messages GYR,ACC,IMU,PARM] [--limit N]
    python3 logparse.py <file.bin> --summary
"""

from __future__ import annotations

import struct
import sys
from collections import Counter, defaultdict

HEAD1, HEAD2 = 0xA3, 0x95

# name -> (struct char, scale, offset)
FMT_MAP = {
    "a": ("h", 1.0, 0.0), "b": ("b", 1.0, 0.0), "B": ("B", 1.0, 0.0),
    "h": ("h", 1.0, 0.0), "H": ("H", 1.0, 0.0), "i": ("i", 1.0, 0.0),
    "I": ("I", 1.0, 0.0), "f": ("f", 1.0, 0.0), "d": ("d", 1.0, 0.0),
    "n": ("4s", 1.0, 0.0), "N": ("16s", 1.0, 0.0), "Z": ("64s", 1.0, 0.0),
    "c": ("h", 0.01, 0.0), "C": ("H", 0.01, 0.0),
    "e": ("i", 0.01, 0.0), "E": ("I", 0.01, 0.0),
    "L": ("i", 1e-7, 0.0), "M": ("B", 1.0, 0.0),
    "q": ("q", 1.0, 0.0), "Q": ("Q", 1.0, 0.0),
}


class Fmt:
    __slots__ = ("type", "length", "name", "format", "columns", "struct", "fields")

    def __init__(self, type_: int, length: int, name: str, fmt: str, columns: list[str]):
        self.type = type_
        self.length = length
        self.name = name
        self.format = fmt
        self.columns = columns
        chars, self.fields = [], []
        for c in fmt:
            if c not in FMT_MAP:
                raise ValueError(f"unknown format char {c!r} in {name!r}")
            sc, scale, off = FMT_MAP[c]
            chars.append(sc)
            self.fields.append((c, scale, off))
        self.struct = struct.Struct("<" + "".join(chars))

    def decode(self, payload: bytes) -> dict:
        vals = self.struct.unpack_from(payload)
        out = {}
        for (c, scale, off), v, col in zip(self.fields, vals, self.columns):
            if isinstance(v, bytes):
                out[col] = v.split(b"\x00", 1)[0].decode("ascii", "replace")
            elif c in "cCeE" or c == "L":
                out[col] = v * scale + off
            else:
                out[col] = v
        return out


def _probe_fmt_layout(d: bytes) -> int:
    """Bytes of frame overhead that the FMT `Length` field includes.

    Verified against a live 7-Nano log by hand:

        d[0:2]  = a3 95                  frame marker
        d[2]    = 0x80                   FMT message type
        d[3:89] = struct '<BB4s16s64s'   -> Type=128, Length=89,
                                            Name='FMT', Format='BBnNZ',
                                            Columns='Type,Length,Name,...'
        next a3 95 found at offset 89

    So the first FMT record's Length (89) equals the WHOLE frame (3 + 86),
    i.e. Length counts the 3-byte header. Hence payload = Length - 3.

    This is probed rather than hard-coded because a different ArduPilot
    generation uses an 8-byte suffix instead, and a wrong choice desynchronises
    the whole file. The probe validates its answer by decoding the FMT record
    and checking the result really is a FMT descriptor.
    """
    if len(d) < 100 or d[:2] != b"\xa3\x95":
        raise ValueError("not an ArduPilot DataFlash log (need 0xa3 0x95)")
    ssz = struct.calcsize("<BB4s16s64s")
    declared = d[4]
    for overhead in (3, 8, 0):
        if declared - overhead != ssz:
            continue
        payload = d[3:3 + ssz]
        try:
            t, _ln, nm, fmt, _cols = struct.unpack("<BB4s16s64s", payload)
        except struct.error:
            continue
        if t == 0x80 and nm.split(b"\x00")[0] == b"FMT" and fmt.split(b"\x00")[0]:
            return overhead
    raise ValueError(f"cannot determine FMT layout (declared Length={declared})")


def read_log(path: str):
    """Yield (msg_name, fields_dict) for every message in the file.

    Frame:  0xA3 0x95 <type u8> <payload>
    payload length = fmt.length - overhead, with `overhead` probed from the
    file (3 for this firmware) and fmt.length taken from that type's FMT record.
    """
    with open(path, "rb") as fh:
        data = fh.read()
    n = len(data)
    overhead = _probe_fmt_layout(data)

    ssz = struct.calcsize("<BB4s16s64s")
    t0, len0, nm0, fmt0, col0 = struct.unpack("<BB4s16s64s", data[3:3 + ssz])
    cols0 = col0.split(b"\x00")[0].decode("ascii", "replace")
    fmts: dict[int, Fmt] = {}
    fmts[0x80] = Fmt(0x80, len0, "FMT", fmt0.split(b"\x00")[0].decode(),
                     [c.strip() for c in cols0.split(",") if c.strip()])

    i = 0
    while i + 3 <= n:
        if data[i] != HEAD1 or data[i + 1] != HEAD2:
            i += 1
            continue
        mtype = data[i + 2]
        fmt = fmts.get(mtype)
        if fmt is None:
            i += 1
            continue
        plen = fmt.length - overhead
        if plen <= 0 or i + 3 + plen > n:
            if i + 3 + plen > n:
                break
            i += 1
            continue
        payload = data[i + 3:i + 3 + plen]
        try:
            fields = fmt.decode(payload)
        except struct.error:
            i += 1
            continue

        if fmt.name == "FMT":
            try:
                t = int(fields["Type"])
                ln = int(fields["Length"])
                nm = fields["Name"]
                fm = fields["Format"]
                cols = [c.strip() for c in fields["Columns"].split(",") if c.strip()]
                if nm and fm and nm != "FMT":
                    fmts[t] = Fmt(t, ln, nm, fm, cols)
            except (KeyError, ValueError, TypeError):
                pass
        elif fmt.name != "FMTU":
            yield fmt.name, fields
        i += 3 + plen


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    path = sys.argv[1]
    args = sys.argv[2:]
    summary = "--summary" in args
    want = None
    limit = None
    if "--messages" in args:
        want = set(args[args.index("--messages") + 1].split(","))
    if "--limit" in args:
        limit = int(args[args.index("--limit") + 1])

    counts: Counter = Counter()
    first: dict = {}
    last: dict = {}
    samples: dict = defaultdict(list)

    total = 0
    for name, fields in read_log(path):
        total += 1
        counts[name] += 1
        if name not in first:
            first[name] = fields
        last[name] = fields
        if want and name in want and (limit is None or len(samples[name]) < limit):
            samples[name].append(fields)

    print(f"══ {path} ══")
    print(f"   共 {total} 条消息, {len(counts)} 种类型\n")

    if summary or not want:
        print(f"   {'消息':<8} {'条数':>10}   字段")
        print("   " + "-" * 90)
        for name, c in counts.most_common(40):
            f0 = first[name]
            print(f"   {name:<8} {c:>10}   {','.join(list(f0)[:11])}")

    for name in (want or []):
        rows = samples.get(name, [])
        if not rows:
            print(f"\n══ {name}: 无数据 ══")
            continue
        print(f"\n══ {name}: {len(rows)} 条样本 ══")
        print(f"   字段: {','.join(rows[0].keys())}")
        print(f"   首条: {rows[0]}")
        if len(rows) > 1:
            print(f"   末条: {rows[-1]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
