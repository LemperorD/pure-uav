"""
Minimal MAVLink v1/v2 codec -- no dependencies.

Why hand-rolled instead of pymavlink: this machine has pymavlink in no
environment, and the `vins` conda env (py3.11 + numpy/scipy/yaml) is load-bearing
for OpenVINS, so it must not be mutated.  Only a handful of messages are needed
to interrogate the 7-Nano, and the wire format is ~60 lines.

Frame layout (MAVLink 2):
    offset  size  field
    0       1     0xFD magic
    1       1     payload length
    2       1     incompatibility flags
    3       1     compatibility flags
    4       1     sequence
    5       1     system id
    6       1     component id
    7       3     message id (little endian, 24-bit)
    10      N     payload (field order is wire order, NOT xml order)
    10+N    2     checksum (X.25/CRC-16-MCRF4XX over bytes[1..10+N-1] + crc_extra)

MAVLink 1 uses 0xFE, 1-byte message id, and a different CRC seed length.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field

MAGIC_V1 = 0xFE
MAGIC_V2 = 0xFD
STX = 0xFD  # the fixed start marker we scan for (v2)

# --------------------------------------------------------------------------
# X.25 / CRC-16-MCRF4XX, the MAVLink checksum
# --------------------------------------------------------------------------


def _build_crc_table() -> list[int]:
    """Reflected table for poly 0x8408 (= reflected 0x1021), CRC-16/MCRF4XX.

    Verified against the published check value:
        crc("123456789") == 0x6F91
    which src/selfcheck_mavlink.py asserts.  An earlier version of this file
    had a table that produced 0xC0C1 at index 1 -- a plausible-looking value --
    while the accumulate step disagreed with the bitwise reference, so every
    frame failed CRC and nothing decoded.  The check value is the only thing
    that catches that class of bug.
    """
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if c & 1 else c >> 1
            c &= 0xFFFF
        table.append(c)
    return table


_CRC_TABLE = _build_crc_table()


def crc_bitwise(data: bytes, crc: int = 0xFFFF) -> int:
    """Reference implementation, no table.  Slow but definitionally correct."""
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def crc_accumulate(byte: int, crc: int) -> int:
    """One byte of CRC-16/MCRF4XX, table-driven (must match crc_bitwise)."""
    return ((crc >> 8) ^ _CRC_TABLE[(crc ^ byte) & 0xFF]) & 0xFFFF


def x25_crc(data: bytes, extra: int) -> int:
    """MAVLink CRC: accumulate over the frame (minus magic and checksum) then
    over the message's crc_extra byte.

    `extra` has NO default on purpose.  It is the crc_extra constant for the
    message id, and there is no valid "unspecified" value -- every MAVLink frame
    folds one in.  An earlier version defaulted it to 0, which silently computed
    crc_bitwise(data + b"\\x00") and made callers think they were getting the
    plain data CRC.
    """
    crc = 0xFFFF
    for b in data:
        crc = crc_accumulate(b, crc)
    return crc_accumulate(extra, crc)


# --------------------------------------------------------------------------
# message field tables
#
# Hand-writing struct format strings turned out to be a bug factory: counting
# 'f' characters by eye produced both a wrong field count and a wrong payload
# size (64 bytes instead of the real 45 for HIGHRES_IMU), with no obvious
# symptom.  Formats are now GENERATED from declared (name, type) pairs and
# reordered into MAVLink wire order (widest-first, stable within a width), so
# the field count and byte size cannot disagree with the declaration.
#
# Type letters are ordinary struct letters: Q I H B q i h b f d s.
# --------------------------------------------------------------------------

# Descending rank for wire order; a stable sort then preserves XML order within
# one rank.  Rank is by TOTAL field size, EXCEPT that a char array ('s') ranks
# below the scalar byte types.
#
# That last part is the non-obvious bit and it was wrong twice. What the data
# showed on a live 7-Nano: with char[16] ranked above uint8, PARAM_VALUE decoded
# param_id one byte late, so names came out as 'NS_GYRO_FILTER' instead of
# 'INS_GYRO_FILTER' and carried a trailing control byte ('LOG_BACKEND_TYPE\x02')
# because the byte after the string was being pulled into it. A 16-character
# field cannot have a trailing byte -- that sentinel is what settled it.
#
# The corruption being INCONSISTENT (some names losing one character, some two)
# is consistent with this: ArduPilot pads short names with spaces, so the shift
# lands differently depending on name length.
#
# The CRC cannot arbitrate any of this. It is computed over the wire bytes and
# does not depend on which name we attach to which byte, so a 100% CRC pass
# says nothing about whether the wire order is right. Live decode output is the
# only oracle here.
TYPE_ORDER = ("Q", "q", "I", "i", "d", "f", "H", "h", "s", "B", "b")
_TYPE_RANK = {c: i for i, c in enumerate(TYPE_ORDER)}

MESSAGES: dict[int, tuple[str, str, list[str], int]] = {}


def _flatten_fmt(fmt: str) -> list[str]:
    """'<IBBBBB' -> ['I','B','B','B','B','B'];  '<fHH16sB' -> ['f','H','H','s','B']"""
    body = fmt.lstrip("<>=!@")
    out, i = [], 0
    while i < len(body):
        c = body[i]
        if c.isdigit():
            j = i
            while j < len(body) and body[j].isdigit():
                j += 1
            out.append(body[j])
            i = j + 1
        else:
            out.append(c)
            i += 1
    return out


def _base_char(t: str) -> str:
    """Type token -> struct char: '16s' -> 's', 'f' -> 'f', 'Q' -> 'Q'."""
    if not t:
        raise ValueError("empty type token")
    return t[-1]


def _sort_rank(t: str) -> int:
    """Wire-order rank; higher rank sorts earlier, so wider types come first.

    A char array ranks by its TOTAL size like everything else, which places
    char[16] between the uint16s and the trailing u8.

    Worth recording: this single ordering rule was guessed wrong three times in
    a row while reverse-engineering the 7-Nano, and each wrong guess produced
    *plausible* parameter names rather than an error ('NS_GYRO_FILTER' instead
    of 'INS_GYRO_FILTER', with a stray control byte on 16-character names).
    Nothing that checks checksums can catch it -- the CRC is computed over the
    wire bytes and is blind to which name we attach to which byte.  The only
    oracle is whether the decoded names look like real ArduPilot parameters, so
    test_wire_order.py decides it empirically instead of by reasoning.
    """
    return _TYPE_RANK[_base_char(t)]


def _wire_format(fields: list[tuple[str, str]]) -> tuple[str, list[str]]:
    """(name, type) pairs in XML order -> (struct format, names in WIRE order)."""
    ordered = sorted(enumerate(fields), key=lambda iv: (_sort_rank(iv[1][1]), iv[0]))
    fmt = "<" + "".join(t for _, (_, t) in ordered)
    return fmt, [n for _, (n, _) in ordered]


def _packed_size(fmt: str) -> int:
    """Byte length of the payload.

    Uses the format string DIRECTLY -- do not feed it _flatten_fmt() output,
    which has already dropped the repeat counts ("16s" -> "s", i.e. 16 bytes
    silently becoming 1).  struct.calcsize with '<' is packed for these formats,
    confirmed empirically: '<QB' -> 9, '<QffH' -> 18.
    """
    return struct.calcsize(fmt)


def _define(msgid: int, name: str, fields: list[tuple[str, str]],
            crc_extra: int, expect_len: int, defaults: dict | None = None) -> None:
    """Register a message.

    `expect_len` is the MAVLink-defined payload length.  It is asserted rather
    than inferred because inferring it from the field list is exactly how three
    messages here ended up the wrong size -- a missing field silently changes
    the length and the only symptom is that every frame of that type fails to
    decode.  Writing the number down forces a mismatch to surface at import.

    `defaults` supplies values for fields the caller leaves out.  This exists
    because "absent means zero" is wrong for routing fields: a request with
    target_system = 0 is dropped by the autopilot without any error, so a
    parameter query simply returns nothing and looks like "no such parameter".
    """
    fmt, names = _wire_format(fields)
    got = _packed_size(fmt)
    assert len(names) == len(_flatten_fmt(fmt)), f"{name}: format/name mismatch"
    assert got == expect_len, f"{name}: payload {got} bytes, expected {expect_len}"
    if defaults:
        unknown = set(defaults) - set(names)
        assert not unknown, f"{name}: defaults for unknown fields {unknown}"
    MESSAGES[msgid] = (name, fmt, names, crc_extra)
    DEFAULTS[msgid] = defaults or {}


# Values filled in when the caller omits a field.
#   target_system/target_component: the autopilot is sysid 1, compid 1.  0 means
#     "unspecified" and most command handlers drop such requests silently.
#   param_index = -1: PARAM_REQUEST_READ's "look this up by name" sentinel.
#     0 would mean "index 0", i.e. always the same parameter.
DEFAULTS: dict[int, dict] = {}


# --- liveness / identity (crc_extra confirmed against captured traffic) ---
# HEARTBEAT is additionally confirmed by live decode: real frames carry
# 'fd 09 ...' i.e. a 9-byte payload.
_define(0, "HEARTBEAT", [
    ("custom_mode", "I"), ("type", "B"), ("autopilot", "B"), ("base_mode", "B"),
    ("system_status", "B"), ("mavlink_version", "B")], 50, 9)

_define(1, "SYS_STATUS", [
    ("sensors_present", "I"), ("sensors_enabled", "I"), ("sensors_health", "I"),
    ("load", "H"), ("voltage_battery", "H"), ("current_battery", "h"),
    ("drop_rate_comm", "H"), ("errors_comm", "H"), ("errors_count1", "H"),
    ("errors_count2", "H"), ("errors_count3", "H"), ("errors_count4", "H"),
    ("battery_remaining", "b")], 124, 31)

# Wire order: param_value f32, param_count u16, param_index u16, param_id char[16],
# param_type u8.  (param_id precedes param_type on the wire because a 16-byte
# array outranks a u8.)
_define(22, "PARAM_VALUE", [
    ("param_value", "f"), ("param_count", "H"), ("param_index", "H"),
    ("param_id", "16s"), ("param_type", "B")], 220, 25)

# Wire order: id u16, severity u8, chunk_seq u8, text char[50].
# 2 + 1 + 1 + 50 = 54.  (An earlier "expected" of 51 was simply misremembered.)
_define(253, "STATUSTEXT", [
    ("severity", "B"), ("text", "50s"), ("id", "H"), ("chunk_seq", "B")], 83, 54)

_define(148, "AUTOPILOT_VERSION", [
    ("capabilities", "Q"), ("uid", "Q"), ("flight_sw_version", "I"),
    ("middleware_sw_version", "I"), ("os_sw_version", "I"), ("board_version", "I"),
    ("vendor_id", "H"), ("product_id", "H"), ("flight_custom_version", "8s"),
    ("middleware_custom_version", "8s"), ("os_custom_version", "8s")], 178, 60)

# --- time sync (msgid 111 seen on the wire; crc_extra brute-forced to 34) ---
_define(111, "TIMESYNC", [("tc1", "q"), ("ts1", "q")], 34, 16)

# --- the IMU messages, the actual point of this exercise ---
# HIGHRES_IMU: 8 (Q) + 13*4 (floats: 9 IMU + 4 pressure/temp) + 2 (H) + 1 (B) = 63.
# XML order per ArduPilot's own send_highres_imu() struct initialiser.
_define(105, "HIGHRES_IMU", [
    ("time_usec", "Q"),
    ("xacc", "f"), ("yacc", "f"), ("zacc", "f"),
    ("xgyro", "f"), ("ygyro", "f"), ("zgyro", "f"),
    ("xmag", "f"), ("ymag", "f"), ("zmag", "f"),
    ("abs_pressure", "f"), ("diff_pressure", "f"),
    ("pressure_alt", "f"), ("temperature", "f"),
    ("fields_updated", "H"), ("id", "B")], 93, 63)

# RAW_IMU: 8 (Q) + 9*2 (nine int16) + 1 (B id) + 2 (h temperature) = 29.
# Units are awkward on the wire: xacc in mg, gyro in mrad/s.
_define(27, "RAW_IMU", [
    ("time_usec", "Q"),
    ("xacc", "h"), ("yacc", "h"), ("zacc", "h"),
    ("xgyro", "h"), ("ygyro", "h"), ("zgyro", "h"),
    ("xmag", "h"), ("ymag", "h"), ("zmag", "h"),
    ("id", "B"), ("temperature", "h")], 144, 29)

# SCALED_IMU (26) / 2 (116) / 3 (129): 4 (ms) + 10*2 (int16) = 24.
_SCALED = [
    ("time_boot_ms", "I"),
    ("xacc", "h"), ("yacc", "h"), ("zacc", "h"),
    ("xgyro", "h"), ("ygyro", "h"), ("zgyro", "h"),
    ("xmag", "h"), ("ymag", "h"), ("zmag", "h"), ("temperature", "h")]
_define(26, "SCALED_IMU", _SCALED, 170, 24)
_define(116, "SCALED_IMU2", _SCALED, 76, 24)
_define(129, "SCALED_IMU3", _SCALED, 46, 24)

# --- command plumbing, needed to ask the FC to stream IMU at a given rate ---
# --- parameter access ---
# PARAM_REQUEST_READ: param_index i16, target_system u8, target_component u8,
# param_id char[16] = 20 bytes.  Note the wire order: the int16 comes first,
# then the two routing bytes, then the string.
# PARAM_REQUEST_READ (msgid 20): param_index i16, target_system u8,
# target_component u8, param_id char[16] = 20 bytes.
_define(20, "PARAM_REQUEST_READ", [
    ("param_index", "h"), ("target_system", "B"), ("target_component", "B"),
    ("param_id", "16s")], 214, 20,
    defaults={"target_system": 1, "target_component": 1, "param_index": -1})

# PARAM_SET (msgid 23): param_value f32, target_system u8, target_component u8,
# param_id char[16], param_type u8 = 4 + 1 + 1 + 16 + 1 = 23 bytes.
# There is no dedicated ack; the autopilot echoes a PARAM_VALUE on success,
# which is how the write is verified.
_define(23, "PARAM_SET", [
    ("param_value", "f"), ("target_system", "B"), ("target_component", "B"),
    ("param_id", "16s"), ("param_type", "B")], 168, 23,
    defaults={"target_system": 1, "target_component": 1, "param_type": 0})

# PARAM_REQUEST_LIST (msgid 21): target_system u8, target_component u8 = 2 bytes.
# crc_extra 159 validated against the live device: sending it produces a stream
# of PARAM_VALUE frames whose param_count (1039) matches the real table.
_define(21, "PARAM_REQUEST_LIST", [
    ("target_system", "B"), ("target_component", "B")], 159, 2,
    defaults={"target_system": 1, "target_component": 1})

# COMMAND_LONG: 7*4 (floats) + 2 (H command) + 3 (B) = 33.  Only three trailing
# u8 are real; the first version declared six and produced 36.
_define(76, "COMMAND_LONG", [
    ("param1", "f"), ("param2", "f"), ("param3", "f"), ("param4", "f"),
    ("param5", "f"), ("param6", "f"), ("param7", "f"),
    ("command", "H"), ("target_system", "B"), ("target_component", "B"),
    ("confirmation", "B")], 152, 33,
    defaults={"target_system": 1, "target_component": 1})

# COMMAND_ACK: 4 (i result_param2) + 2 (H command) + 4*1 (B) = 10.
_define(77, "COMMAND_ACK", [
    ("command", "H"), ("result", "B"), ("progress", "B"),
    ("result_param2", "i"), ("target_system", "B"), ("target_component", "B")], 143, 10)

_CRC_EXTRA = {mid: v[3] for mid, v in MESSAGES.items()}
_NAME_TO_ID = {v[0]: mid for mid, v in MESSAGES.items()}


@dataclass
class Frame:
    msgid: int
    name: str
    sysid: int
    compid: int
    seq: int
    payload: bytes
    fields: dict = field(default_factory=dict)


def _decode_fields(msgid: int, payload: bytes) -> dict:
    """Decode a payload using the message's WIRE-order field names.

    MAVLink 2 truncates trailing zero bytes, so a payload may legitimately be
    shorter than the full struct.  Two earlier versions of this function got
    this wrong:

      v1 returned {} for any short payload, silently dropping whole messages;
      v2 decoded what was present but left the truncated tail absent from the
         result, so a field that was simply zero showed up as None (surfaced as
         "sensor instance id: {None}" in the smoke test, because HIGHRES_IMU's
         id byte is 0 and therefore always stripped).

    Truncation only ever removes *trailing zeros*, so absent fields are zero by
    definition -- zero-fill them.
    """
    name, fmt, names, _ = MESSAGES[msgid]
    chars = _flatten_fmt(fmt)
    size = struct.calcsize(fmt)
    avail = min(len(payload), size)
    out: dict = {}
    vals: list = []
    missing: list[str] = []

    if avail >= size:
        vals = list(struct.unpack(fmt, payload[:size]))
    else:
        off = 0
        for c in chars:
            sz = struct.calcsize("<" + c)
            if off + sz > avail:
                break
            vals.append(struct.unpack_from("<" + c, payload, off)[0])
            off += sz

    for k, v in zip(names, vals):
        if isinstance(v, bytes):
            out[k] = v.split(b"\x00", 1)[0].decode("ascii", "replace")
        else:
            out[k] = v
    missing = names[len(vals):]
    for k in missing:
        out[k] = "" if k.endswith("s") else 0
    return out


class MavlinkParser:
    """Incremental byte-stream parser.  Feed it bytes, get Frames back.

    Resynchronises by scanning for the next 0xFD after a bad checksum, which is
    what makes it usable on a noisy serial link.
    """

    def __init__(self, *, verify_crc: bool = True) -> None:
        self.buf = bytearray()
        self.verify_crc = verify_crc
        self.stats = {"frames": 0, "crc_bad": 0, "unknown_msgid": 0}

    def feed(self, data: bytes) -> list[Frame]:
        self.buf += data
        out: list[Frame] = []

        while True:
            start = self.buf.find(MAGIC_V2)
            if start < 0:
                # keep at most a byte, everything before is garbage
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break
            if start > 0:
                del self.buf[:start]
            if len(self.buf) < 12:
                break  # need a full header

            plen = self.buf[1]
            total = 10 + plen + 2
            if plen > 255 or total > 280:
                del self.buf[0]  # bogus length, resync
                continue
            if len(self.buf) < total:
                break  # wait for the rest

            frame = bytes(self.buf[:total])
            msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)

            if self.verify_crc:
                extra = _CRC_EXTRA.get(msgid)
                if extra is None:
                    self.stats["unknown_msgid"] += 1
                    del self.buf[0]
                    continue
                want = frame[total - 2] | (frame[total - 1] << 8)
                if x25_crc(frame[1 : 10 + plen], extra) != want:
                    self.stats["crc_bad"] += 1
                    del self.buf[0]
                    continue

            del self.buf[:total]
            self.stats["frames"] += 1
            name = MESSAGES[msgid][0]
            out.append(
                Frame(
                    msgid=msgid, name=name, sysid=frame[5], compid=frame[6],
                    seq=frame[4], payload=frame[10 : 10 + plen],
                    fields=_decode_fields(msgid, frame[10 : 10 + plen]),
                )
            )
        return out


# --------------------------------------------------------------------------
# outbound frames
# --------------------------------------------------------------------------


def pack(
    msgid: int,
    fields: dict,
    *,
    sysid: int = 255,
    compid: int = 190,
    seq: int = 0,
) -> bytes:
    """Build a MAVLink v2 frame for one of the messages in MESSAGES.

    Field values are coerced to the declared type and range-clamped rather than
    passed straight to struct.pack.  A MAVLink caller routinely has to put a
    negative sentinel into an unsigned field -- PARAM_REQUEST_READ uses
    param_index = -1 to mean "look the parameter up by name" -- and struct.pack
    rejecting that with "ushort format requires 0 <= number <= 65535" is a
    footgun, not a safety net.  The framing itself is unchanged.
    """
    name, fmt, names, extra = MESSAGES[msgid]
    # Build the payload by packing in wire order.  Fields absent from `fields`
    # default to zero, which is correct for the requests built below.
    ordered = []
    dflt = DEFAULTS.get(msgid, {})
    for n, c in zip(names, _flatten_fmt(fmt)):
        v = fields.get(n, dflt.get(n, 0))
        if c == "s":
            v = v if isinstance(v, bytes) else str(v).encode()
        elif c in "bhiq":
            lim = {"b": (-128, 127), "h": (-32768, 32767),
                   "i": (-2**31, 2**31 - 1), "q": (-2**63, 2**63 - 1)}[c]
            v = max(lim[0], min(lim[1], int(v)))
        elif c in "BHILQ":
            lim = {"B": 0xFF, "H": 0xFFFF, "I": 0xFFFFFFFF,
                   "L": 0xFFFFFFFF, "Q": 0xFFFFFFFFFFFFFFFF}[c]
            v = int(v) & lim if int(v) < 0 else min(int(v), lim)
        ordered.append(v)
    payload = struct.pack(fmt, *ordered)

    hdr = bytes([MAGIC_V2, len(payload), 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF,
                 msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    crc = x25_crc(hdr[1:] + payload, extra)
    return hdr + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
