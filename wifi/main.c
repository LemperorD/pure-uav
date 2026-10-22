/*
 * main.c —— PWLink 联通测试：手写 MAVLink 2 握手 + 遥测
 *
 * 运行（本机连上 PWLink 的 WiFi 后）：
 *   ./pwlink_test [目标IP] [端口]
 *   默认：目标 192.168.4.1，端口 14550
 *
 * 流程：收飞控心跳 → 拉全量参数 → 请求版本 → 设遥测速率 → 进入稳态。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <windows.h>
  typedef int socklen_t;
  #define CLOSE_SOCK closesocket
  #define SOCKET_T   SOCKET
#else
  #include <sys/socket.h>
  #include <sys/select.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  #define CLOSE_SOCK close
  #define SOCKET_T   int
  #define INVALID_SOCKET (-1)
#endif

#include "mavlink2.h"

/* 上位机身份：与 QGC 一致（联通测试用 255 没问题，但别同时开 QGC） */
#define GCS_SYSID   255
#define GCS_COMPID  MAV_COMP_ID_MISSIONPLANNER   /* 190 */

static uint64_t now_ms(void) {
#ifdef _WIN32
    return (uint64_t)GetTickCount64();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
#endif
}

static SOCKET_T open_udp(int port) {
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    SOCKET_T s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s == INVALID_SOCKET) { perror("socket"); return INVALID_SOCKET; }

    int yes = 1;
    setsockopt(s, SOL_SOCKET, SO_BROADCAST, (const char*)&yes, sizeof(yes));

    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_port = htons((uint16_t)port);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (struct sockaddr*)&local, sizeof(local)) < 0) {
        perror("bind"); CLOSE_SOCK(s); return INVALID_SOCKET;
    }
    return s;
}

int main(int argc, char **argv) {
    const char *dst_ip = (argc > 1) ? argv[1] : "192.168.4.1";
    int port = (argc > 2) ? atoi(argv[2]) : 14550;

    SOCKET_T sock = open_udp(port);
    if (sock == INVALID_SOCKET) return 1;

    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons((uint16_t)port);
    dst.sin_addr.s_addr = inet_addr(dst_ip);

    printf("[init] UDP bound 0.0.0.0:%d, sending to %s:%d (sysid=%d compid=%d)\n",
           port, dst_ip, port, GCS_SYSID, GCS_COMPID);
    printf("[hb]   waiting for flight-controller heartbeat...\n");
    fflush(stdout);

    enum { ST_WAIT_HB, ST_REQ_PARAMS, ST_REQ_VERSION, ST_READY } st = ST_WAIT_HB;
    uint8_t seq = 0;
    uint64_t last_hb = 0;
    uint16_t param_count = 0, param_received = 0;
    int param_printed = 0;

    uint8_t frame[512], rxbuf[2048];

    /* 先发一次自己的心跳（声明 GCS） */
    {
        uint16_t n = mav_pack_heartbeat(frame, GCS_SYSID, GCS_COMPID, seq++,
                                        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
                                        0, 0, MAV_STATE_ACTIVE);
        sendto(sock, (const char*)frame, n, 0, (struct sockaddr*)&dst, sizeof(dst));
    }

    for (;;) {
        uint64_t now = now_ms();

        /* 1Hz 心跳（否则飞控判 GCS 失联） */
        if (now - last_hb >= 1000) {
            uint16_t n = mav_pack_heartbeat(frame, GCS_SYSID, GCS_COMPID, seq++,
                                            MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
                                            0, 0, MAV_STATE_ACTIVE);
            sendto(sock, (const char*)frame, n, 0, (struct sockaddr*)&dst, sizeof(dst));
            last_hb = now;
        }

        /* 非阻塞收一包（100ms 超时） */
        fd_set rfds; FD_ZERO(&rfds); FD_SET(sock, &rfds);
        struct timeval tv = {0, 100000};
        if (select((int)sock + 1, &rfds, NULL, NULL, &tv) <= 0) continue;

        socklen_t alen = sizeof(dst);
        int rlen = (int)recvfrom(sock, (char*)rxbuf, sizeof(rxbuf), 0,
                                 (struct sockaddr*)&dst, &alen);
        if (rlen <= 0) continue;

        /* 逐帧解析（一个 UDP 包可能含多帧，滑动找 STX） */
        for (int off = 0; off + MAVLINK2_HEADER_LEN + MAVLINK_CHECKSUM_LEN <= rlen; ) {
            if (rxbuf[off] != MAVLINK_STX) { off++; continue; }

            uint8_t sysid, compid, plen;
            uint32_t msgid;
            const uint8_t *payload;
            uint16_t total = mav_frame_parse(rxbuf + off, (uint16_t)(rlen - off),
                                             &sysid, &compid, &msgid, &payload, &plen);
            if (total == 0) { off++; continue; }   /* 校验失败，滑动 1 字节 */
            off += total;

            switch (st) {
            case ST_WAIT_HB:
                if (msgid == MAV_MSG_HEARTBEAT) {
                    uint8_t type, ap, base, ss;
                    uint32_t cm;
                    mav_parse_heartbeat(payload, &type, &ap, &base, &cm, &ss);
                    printf("[hb]   HEARTBEAT: sysid=%d compid=%d autopilot=%d type=%d state=%d\n",
                           sysid, compid, ap, type, ss);
                    if (ap == MAV_AUTOPILOT_PX4) {
                        uint16_t n = mav_pack_param_request_list(frame, GCS_SYSID, GCS_COMPID,
                                                                 seq++, sysid, MAV_COMP_ID_AUTOPILOT1);
                        sendto(sock, (const char*)frame, n, 0, (struct sockaddr*)&dst, sizeof(dst));
                        printf("[param] requested full param list from sysid=%d\n", sysid);
                        fflush(stdout);
                        st = ST_REQ_PARAMS;
                    }
                }
                break;

            case ST_REQ_PARAMS:
                if (msgid == MAV_MSG_PARAM_VALUE) {
                    char pid[17]; float val; uint8_t pt; uint16_t cnt, idx;
                    mav_parse_param_value(payload, pid, &val, &pt, &cnt, &idx);
                    param_count = cnt;
                    param_received++;
                    if (param_printed < 3) {
                        printf("[param] %3d/%d  %-16s = %.4f\n", idx + 1, cnt, pid, val);
                        param_printed++;
                    }
                    if (idx + 1 >= cnt) {
                        printf("[param] done: %d params received\n", cnt);
                        fflush(stdout);
                        /* 请求 AUTOPILOT_VERSION */
                        uint16_t n = mav_pack_command_long(frame, GCS_SYSID, GCS_COMPID, seq++,
                                                           sysid, MAV_COMP_ID_AUTOPILOT1,
                                                           MAV_CMD_REQUEST_AUTOPILOT_VERSION, 0,
                                                           1, 0, 0, 0, 0, 0, 0);
                        sendto(sock, (const char*)frame, n, 0, (struct sockaddr*)&dst, sizeof(dst));
                        printf("[ver]  requesting AUTOPILOT_VERSION...\n");
                        fflush(stdout);
                        st = ST_REQ_VERSION;
                    }
                }
                break;

            case ST_REQ_VERSION:
                if (msgid == MAV_MSG_AUTOPILOT_VERSION) {
                    uint32_t fsw, bv; uint16_t vid, pid;
                    mav_parse_autopilot_version(payload, plen, &fsw, &bv, &vid, &pid);
                    printf("[ver]  flight_sw=%u.%u.%u  board=%u  vendor=%u product=%u\n",
                           (fsw >> 24) & 0xFF, (fsw >> 16) & 0xFF, fsw & 0xFFFF, bv, vid, pid);
                    fflush(stdout);
                    /* 设遥测速率：ATTITUDE 10Hz，SYS_STATUS 1Hz */
                    uint16_t n = mav_pack_message_interval(frame, GCS_SYSID, GCS_COMPID, seq++,
                                                           MAV_MSG_ATTITUDE, 100000);
                    sendto(sock, (const char*)frame, n, 0, (struct sockaddr*)&dst, sizeof(dst));
                    n = mav_pack_message_interval(frame, GCS_SYSID, GCS_COMPID, seq++,
                                                  MAV_MSG_SYS_STATUS, 1000000);
                    sendto(sock, (const char*)frame, n, 0, (struct sockaddr*)&dst, sizeof(dst));
                    printf("[ready] link established, streaming telemetry...\n");
                    fflush(stdout);
                    st = ST_READY;
                }
                break;

            case ST_READY:
                if (msgid == MAV_MSG_SYS_STATUS) {
                    uint16_t v; int16_t c; int8_t br = 0;
                    mav_parse_sys_status(payload, plen, &v, &c, &br);
                    printf("[sys]  voltage=%.2fV  current=%.1fA  bat=%d%%\n",
                           v / 1000.0f, c / 100.0f, br);
                } else if (msgid == MAV_MSG_ATTITUDE) {
                    float r, p, y;
                    mav_parse_attitude(payload, &r, &p, &y);
                    printf("[att]  roll=%.1f pitch=%.1f yaw=%.1f deg\n",
                           r * 57.29578f, p * 57.29578f, y * 57.29578f);
                } else if (msgid == MAV_MSG_COMMAND_ACK) {
                    uint16_t cmd; uint8_t res;
                    mav_parse_command_ack(payload, &cmd, &res);
                    printf("[ack]  command=%d result=%d\n", cmd, res);
                }
                break;
            }
        }
    }
    return 0;
}
