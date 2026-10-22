/*
 * mavlink2.h —— 手写 MAVLink 2 编解码核心（路线 A）
 *
 * 目标：不依赖官方 mavlink 库，自己实现组包 / 解析 / CRC 校验。
 * 只覆盖联通测试需要的最小消息集，常量均与官方 c_library_v2 核对一致。
 *
 * 帧结构（MAVLink 2，magic = 0xFD）：
 *   [0] STX(0xFD) [1] len [2] incompat_flags [3] compat_flags
 *   [4] seq [5] sysid [6] compid [7..9] msgid(24bit 小端)
 *   [10 .. 10+len-1] payload
 *   [10+len .. 11+len] checksum(CRC-16/MCRF4XX, 低字节在前)
 * 总长 = 12 + len
 */
#ifndef MAVLINK2_H
#define MAVLINK2_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 帧常量 ==================== */
#define MAVLINK_STX              0xFD
#define MAVLINK2_HEADER_LEN      10   /* STX + len + incompat + compat + seq + sysid + compid + msgid(3) */
#define MAVLINK_CHECKSUM_LEN     2
#define MAVLINK_MAX_PAYLOAD      255

/* ==================== 消息 ID ==================== */
#define MAV_MSG_HEARTBEAT         0
#define MAV_MSG_SYS_STATUS        1
#define MAV_MSG_PARAM_REQUEST_LIST 21
#define MAV_MSG_PARAM_VALUE       22
#define MAV_MSG_ATTITUDE          30
#define MAV_MSG_COMMAND_LONG      76
#define MAV_MSG_COMMAND_ACK       77
#define MAV_MSG_AUTOPILOT_VERSION 148
#define MAV_MSG_MESSAGE_INTERVAL  244

/* ==================== CRC_EXTRA（与官方 c_library_v2 核对） ==================== */
#define CRC_HEARTBEAT           50
#define CRC_SYS_STATUS          124
#define CRC_PARAM_REQUEST_LIST  159
#define CRC_PARAM_VALUE         220
#define CRC_ATTITUDE            39
#define CRC_COMMAND_LONG        152
#define CRC_COMMAND_ACK         143
#define CRC_AUTOPILOT_VERSION   178
#define CRC_MESSAGE_INTERVAL    95

/* ==================== 枚举（common/minimal.xml） ==================== */
#define MAV_TYPE_GCS             6
#define MAV_TYPE_QUADROTOR       2
#define MAV_AUTOPILOT_INVALID    8
#define MAV_AUTOPILOT_PX4        12
#define MAV_STATE_ACTIVE         4
#define MAV_STATE_STANDBY        3
#define MAV_COMP_ID_AUTOPILOT1   1
#define MAV_COMP_ID_MISSIONPLANNER 190

/* MAV_CMD 常用命令 */
#define MAV_CMD_REQUEST_AUTOPILOT_VERSION 520
#define MAV_CMD_SET_MESSAGE_INTERVAL      511

/* ==================== 小端读写 helper ==================== */
static inline void mav_put_u8 (uint8_t *b, int off, uint8_t  v) { b[off] = v; }
static inline void mav_put_u16(uint8_t *b, int off, uint16_t v) { b[off]=v&0xFF; b[off+1]=(v>>8)&0xFF; }
static inline void mav_put_u32(uint8_t *b, int off, uint32_t v) {
    b[off]=v&0xFF; b[off+1]=(v>>8)&0xFF; b[off+2]=(v>>16)&0xFF; b[off+3]=(v>>24)&0xFF;
}
static inline void mav_put_u64(uint8_t *b, int off, uint64_t v) {
    for (int i = 0; i < 8; i++) b[off+i] = (uint8_t)(v >> (8*i));
}
static inline void mav_put_i32(uint8_t *b, int off, int32_t v)  { mav_put_u32(b, off, (uint32_t)v); }
static inline void mav_put_f32(uint8_t *b, int off, float v) {
    uint32_t u; memcpy(&u, &v, 4); mav_put_u32(b, off, u);
}

static inline uint8_t  mav_get_u8 (const uint8_t *b, int off) { return b[off]; }
static inline uint16_t mav_get_u16(const uint8_t *b, int off) { return (uint16_t)(b[off] | (b[off+1]<<8)); }
static inline uint32_t mav_get_u32(const uint8_t *b, int off) {
    return (uint32_t)b[off] | ((uint32_t)b[off+1]<<8) | ((uint32_t)b[off+2]<<16) | ((uint32_t)b[off+3]<<24);
}
static inline int32_t mav_get_i32(const uint8_t *b, int off) { return (int32_t)mav_get_u32(b, off); }
static inline float    mav_get_f32(const uint8_t *b, int off) {
    uint32_t u = mav_get_u32(b, off); float f; memcpy(&f, &u, 4); return f;
}

/* ==================== CRC-16/MCRF4XX ==================== */
uint16_t mav_crc_accumulate(uint8_t data, uint16_t crc);
uint16_t mav_crc_calculate(const uint8_t *buf, uint16_t len);
/* 根据 msgid 返回 CRC_EXTRA，未知消息返回 0 */
uint8_t  mav_crc_extra(uint32_t msgid);

/* ==================== 组包 ==================== */
/*
 * 通用组包：把 payload 组装成完整帧，写入 frame（调用者保证空间 >= 12+payload_len）。
 * 返回帧总长度。
 * 注意：checksum 在 payload 之后单独计算（含 CRC_EXTRA）。
 */
uint16_t mav_frame_pack(uint8_t *frame, uint8_t sysid, uint8_t compid, uint8_t seq,
                        uint32_t msgid, const uint8_t *payload, uint8_t payload_len);

/* 便捷组包 */
uint16_t mav_pack_heartbeat(uint8_t *frame, uint8_t sysid, uint8_t compid, uint8_t seq,
                            uint8_t type, uint8_t autopilot, uint8_t base_mode,
                            uint32_t custom_mode, uint8_t system_status);
uint16_t mav_pack_param_request_list(uint8_t *frame, uint8_t sysid, uint8_t compid, uint8_t seq,
                                     uint8_t target_system, uint8_t target_component);
uint16_t mav_pack_command_long(uint8_t *frame, uint8_t sysid, uint8_t compid, uint8_t seq,
                               uint8_t target_system, uint8_t target_component, uint16_t command,
                               uint8_t confirmation, float p1, float p2, float p3, float p4,
                               float p5, float p6, float p7);
uint16_t mav_pack_message_interval(uint8_t *frame, uint8_t sysid, uint8_t compid, uint8_t seq,
                                   uint16_t message_id, int32_t interval_us);

/* ==================== 解析 ==================== */
/*
 * 从对齐到 STX 的完整帧解析头部，返回帧总长度（= 12+len），校验 checksum。
 * 返回 0 表示解析失败（长度非法或 checksum 错误）。
 * 成功时输出 sysid / compid / msgid / payload 指针 / payload 长度。
 */
uint16_t mav_frame_parse(const uint8_t *frame, uint16_t frame_len,
                         uint8_t *sysid, uint8_t *compid, uint32_t *msgid,
                         const uint8_t **payload, uint8_t *payload_len);

/* 已知消息解析（payload 已在 mav_frame_parse 中校验过） */
void mav_parse_heartbeat(const uint8_t *p, uint8_t *type, uint8_t *autopilot,
                         uint8_t *base_mode, uint32_t *custom_mode, uint8_t *system_status);
void mav_parse_param_value(const uint8_t *p, char *param_id, float *value,
                           uint8_t *type, uint16_t *count, uint16_t *index);
void mav_parse_sys_status(const uint8_t *p, uint8_t len, uint16_t *voltage_mv,
                          int16_t *current_ca, int8_t *battery_remaining);
void mav_parse_attitude(const uint8_t *p, float *roll, float *pitch, float *yaw);
void mav_parse_autopilot_version(const uint8_t *p, uint8_t len,
                                 uint32_t *flight_sw, uint32_t *board_ver,
                                 uint16_t *vendor_id, uint16_t *product_id);
void mav_parse_command_ack(const uint8_t *p, uint16_t *command, uint8_t *result);

#ifdef __cplusplus
}
#endif
#endif /* MAVLINK2_H */
