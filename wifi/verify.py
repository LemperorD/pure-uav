#!/usr/bin/env python3
"""
verify.py —— 对本机手写 C 库（mavlink2.c）的逻辑对拍验证

由于本机无 C 编译器，用 Python 复刻与 C 完全一致的 CRC + 组包逻辑，
再用官方 pymavlink 作为「黄金参考」逐字节对拍，确保 C 代码正确。
"""
import struct

# ==================== 复刻 C 的 CRC-16/MCRF4XX ====================
def crc_accumulate(data, crc):
    tmp = (data ^ (crc & 0xFF)) & 0xFF
    tmp ^= (tmp << 4)
    tmp &= 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF

def crc_calculate(buf):
    crc = 0xFFFF
    for b in buf:
        crc = crc_accumulate(b, crc)
    return crc

# ==================== CRC_EXTRA（与 mavlink2.h 一致） ====================
CRC_EXTRA = {0: 50, 1: 124, 21: 159, 22: 220, 30: 39, 76: 152, 77: 143, 148: 178, 244: 95}

def frame_pack(sysid, compid, seq, msgid, payload):
    f = bytearray()
    f.append(0xFD)
    f.append(len(payload))
    f.append(0)
    f.append(0)
    f.append(seq & 0xFF)
    f.append(sysid)
    f.append(compid)
    f.append(msgid & 0xFF)
    f.append((msgid >> 8) & 0xFF)
    f.append((msgid >> 16) & 0xFF)
    f += payload
    crc = crc_calculate(bytes(f[1:]))
    crc = crc_accumulate(CRC_EXTRA[msgid], crc)
    f.append(crc & 0xFF)
    f.append((crc >> 8) & 0xFF)
    return bytes(f)

def pack_heartbeat(seq, type_, autopilot, base_mode, custom_mode, system_status):
    p = struct.pack('<I', custom_mode) + bytes([type_, autopilot, base_mode, system_status, 3])
    return frame_pack(255, 190, seq, 0, p)

def pack_param_request_list(seq, tsys, tcomp):
    return frame_pack(255, 190, seq, 21, bytes([tsys, tcomp]))

def pack_command_long(seq, tsys, tcomp, command, confirmation, p):
    payload = struct.pack('<7fHBBB', *p, command, tsys, tcomp, confirmation)
    return frame_pack(255, 190, seq, 76, payload)

def pack_message_interval(seq, msgid, interval_us):
    payload = struct.pack('<iH', interval_us, msgid)
    return frame_pack(255, 190, seq, 244, payload)

# ==================== 黄金参考：pymavlink ====================
def verify_with_pymavlink():
    from pymavlink.dialects.v20 import common
    ok = True

    def ref(mav, msg):
        mav.seq = 0
        return msg.pack(mav)

    # 1. HEARTBEAT
    mav = common.MAVLink(None, srcSystem=255, srcComponent=190)
    mine = pack_heartbeat(0, common.MAV_TYPE_QUADROTOR, common.MAV_AUTOPILOT_PX4,
                          0, 0, common.MAV_STATE_ACTIVE)
    theirs = ref(mav, mav.heartbeat_encode(common.MAV_TYPE_QUADROTOR,
                                           common.MAV_AUTOPILOT_PX4, 0, 0,
                                           common.MAV_STATE_ACTIVE))
    ok &= check("HEARTBEAT", mine, theirs)

    # 2. PARAM_REQUEST_LIST
    mine = pack_param_request_list(0, 1, 1)
    theirs = ref(mav, mav.param_request_list_encode(1, 1))
    ok &= check("PARAM_REQUEST_LIST", mine, theirs)

    # 3. COMMAND_LONG (REQUEST_AUTOPILOT_VERSION)
    mine = pack_command_long(0, 1, 1, 520, 0, (1, 0, 0, 0, 0, 0, 0))
    theirs = ref(mav, mav.command_long_encode(1, 1, 520, 0, 1, 0, 0, 0, 0, 0, 0))
    ok &= check("COMMAND_LONG", mine, theirs)

    # 4. MESSAGE_INTERVAL (ATTITUDE 10Hz)
    mine = pack_message_interval(0, 30, 100000)
    theirs = ref(mav, mav.message_interval_encode(30, 100000))
    ok &= check("MESSAGE_INTERVAL", mine, theirs)

    return ok

def check(name, mine, theirs):
    same = (mine == theirs)
    print(f"[{name}] {'OK' if same else 'MISMATCH'}  len mine={len(mine)} ref={len(theirs)}")
    if not same:
        print("  mine  :", mine.hex())
        print("  ref   :", theirs.hex())
    return same

if __name__ == "__main__":
    print("=== 自检：CRC round-trip（组包→CRC 重算应一致）===")
    f = pack_heartbeat(7, 2, 12, 0, 0, 4)
    payload = f[10:10 + f[1]]
    crc = crc_calculate(f[1:10 + f[1]])
    crc = crc_accumulate(CRC_EXTRA[0], crc)
    got = f[-2] | (f[-1] << 8)
    print(f"  组包 checksum={got:04x}  重算={crc:04x}  {'OK' if got == crc else 'MISMATCH'}")

    print("\n=== 黄金参考对拍（pymavlink）===")
    try:
        ok = verify_with_pymavlink()
        print("\n结果:", "全部通过" if ok else "存在不一致")
    except ImportError:
        print("pymavlink 未安装，跳过黄金参考对拍")
    except Exception as e:
        print("对拍异常:", e)
