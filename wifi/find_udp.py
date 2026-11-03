#!/usr/bin/env python3
"""
find_udp.py —— 在 PW-Link WiFi 链路上穷举查找飞控的 MAVLink UDP 流

背景：PW-Link(ESP8266 数传) 默认 AP 为 192.168.4.1，桥接飞控串口 TELEM，
      通常以 UDP 14550 转发 MAVLink。若在 14550 收不到，可用本脚本穷举端口。

用法：
  python3 find_udp.py                 # 先扫常见 MAVLink 端口（快）
  python3 find_udp.py --full          # 穷举 1..65535（分批，较慢）
  python3 find_udp.py --ports 14550,4000,5555

原理（无需 root）：
  1. 在候选端口上 bind 0.0.0.0（收广播 + 发给本机的单播）
  2. 向 192.168.4.1 与 192.168.4.255 各端口发一个 MAVLink 心跳作为触发
     （让 PW-Link 学习到本机 IP/端口，从而把串口 MAVLink 流转发过来）
  3. 轮询这些 socket，打印收到的数据来源与首字节（0xFD=MAVLink2, 0xFE=MAVLink1）
"""
import socket
import struct
import time
import sys
import select

DST = "192.168.4.1"
BCAST = "192.168.4.255"

# 常见 MAVLink / PX4 / ArduPilot / ESP 数传 UDP 端口
COMMON_PORTS = [
    14550, 14540, 14551, 14555, 14556, 14557, 14558, 14559,
    14560, 14561, 14562, 14563, 14564, 14565,
    18570, 2000, 2001, 4000, 5000, 5555, 5556, 5760, 5762, 5763,
    6000, 7000, 8000, 9000, 10000, 12345, 60000,
]


def crc_accumulate(data, crc):
    tmp = data ^ (crc & 0xFF)
    tmp ^= (tmp << 4)
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def crc_calc(buf):
    c = 0xFFFF
    for b in buf:
        c = crc_accumulate(b, c)
    return c


def heartbeat(seq=0):
    """构造一个最小 MAVLink2 HEARTBEAT 帧（sysid=255, compid=190）"""
    sysid, compid = 255, 190
    msgid, crc_extra = 0, 50
    # payload: custom_mode(u32)=0, type=6(GCS), autopilot=8(invalid), base=0, status=4(active), mavlink_version=3
    payload = struct.pack('<I', 0) + bytes([6, 8, 0, 4, 3])
    head = bytes([len(payload), 0, 0, seq & 0xFF, sysid, compid]) \
        + bytes([msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    crc = crc_accumulate(crc_extra, crc_calc(head + payload))
    return b'\xFD' + head + payload + bytes([crc & 0xFF, crc >> 8])


def bind_ports(ports):
    socks = {}
    for p in ports:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", p))
            s.setblocking(False)
            socks[p] = s
        except OSError as e:
            print(f"  [skip] 端口 {p} 被占用/无法绑定: {e}")
    return socks


def trigger(ports):
    hb = heartbeat(0)
    sent = 0
    for p in ports:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as t:
                t.sendto(hb, (DST, p))
                t.sendto(hb, (BCAST, p))
                sent += 2
        except OSError:
            pass
    print(f"  已向 {DST} 与 {BCAST} 的 {len(ports)} 个端口各发 1 个心跳触发")


def listen(socks, duration):
    # 用 poll() 而非 select()：select 受 FD_SETSIZE(1024) 限制，穷举大端口时会崩
    poller = select.poll()
    fd_to_sock = {}
    for s in socks.values():
        poller.register(s, select.POLLIN)
        fd_to_sock[s.fileno()] = s

    end = time.time() + duration
    hits = []
    while True:
        remaining = end - time.time()
        if remaining <= 0:
            break
        for fd, ev in poller.poll(int(remaining * 1000)):
            if not (ev & select.POLLIN):
                continue
            s = fd_to_sock.get(fd)
            if s is None:
                continue
            try:
                data, addr = s.recvfrom(4096)
            except OSError:
                continue
            local_port = s.getsockname()[1]
            magic = data[0] if data else -1
            hits.append((local_port, addr, data))
            tag = "MAVLink2" if magic == 0xFD else ("MAVLink1" if magic == 0xFE else "?")
            print(f"  [HIT] 本机端口={local_port}  来自={addr[0]}:{addr[1]}  "
                  f"len={len(data)}  首字节=0x{magic:02x} ({tag})")
            print(f"        前16字节: {data[:16].hex()}")
    return hits


def scan(ports, duration):
    print(f"\n== 扫描 {len(ports)} 个端口，监听 {duration}s ==")
    socks = bind_ports(ports)
    if not socks:
        print("  没有端口可绑定")
        return []
    trigger(list(socks.keys()))
    hits = listen(socks, duration)
    for s in socks.values():
        s.close()
    return hits


def full_sweep(batch=2048, batch_time=0.8):
    """穷举 1..65535，分批 bind+触发+监听，避免 fd 超限"""
    print(f"\n== 穷举 1..65535（批大小 {batch}，每批监听 {batch_time}s）==")
    all_hits = []
    for start in range(1, 65536, batch):
        ports = list(range(start, min(start + batch, 65536)))
        hits = scan(ports, batch_time)
        all_hits.extend(hits)
        if start % (batch * 8) < batch:
            print(f"  ... 已扫到 {min(start + batch, 65535)}")
    return all_hits


def main():
    global DST, BCAST

    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--full", action="store_true", help="穷举 1..65535")
    ap.add_argument("--ports", type=str, help="逗号分隔的端口列表")
    ap.add_argument("--time", type=float, default=6.0, help="监听秒数（默认6）")
    ap.add_argument("--target", default=DST, help="数传 IP（默认 192.168.4.1）")
    args = ap.parse_args()

    DST = args.target
    # 广播地址：把最后一段换成 255（适配 /24）
    parts = DST.split('.')
    BCAST = '.'.join(parts[:3]) + '.255'

    if args.ports:
        ports = [int(x) for x in args.ports.split(',') if x.strip()]
        hits = scan(ports, args.time)
    elif args.full:
        hits = full_sweep()
    else:
        hits = scan(COMMON_PORTS, args.time)

    print("\n== 结果 ==")
    if hits:
        print(f"  共收到 {len(hits)} 个命中。上面标 [HIT] 的本机端口就是要监听的端口。")
    else:
        print("  没有命中。可能原因：")
        print("  1) 飞控未上电 / TELEM 串口未接 / 波特率不匹配（PW-Link 默认 57600）")
        print("  2) PW-Link 配置成 TCP 而非 UDP（可浏览器打开 http://192.168.4.1 查看）")
        print("  3) PW-Link 只单播给某个旧 IP（手机/另一台设备曾连接），需要重启 PW-Link")
        print("  4) 有其它 GCS 已占用该链路")


if __name__ == "__main__":
    main()
