/*
 * mavlink2.c —— 手写 MAVLink 2 编解码实现
 */
#include "mavlink2.h"

/* ==================== CRC-16/MCRF4XX ==================== */
uint16_t mav_crc_accumulate(uint8_t data, uint16_t crc) {
    uint8_t tmp = data ^ (uint8_t)(crc & 0xFF);
    tmp ^= (tmp << 4);
    return (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

uint16_t mav_crc_calculate(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) crc = mav_crc_accumulate(*buf++, crc);
    return crc;
}

uint8_t mav_crc_extra(uint32_t msgid) {
    switch (msgid) {
    case MAV_MSG_HEARTBEAT:         return CRC_HEARTBEAT;
    case MAV_MSG_SYS_STATUS:        return CRC_SYS_STATUS;
    case MAV_MSG_PARAM_REQUEST_LIST:return CRC_PARAM_REQUEST_LIST;
    case MAV_MSG_PARAM_VALUE:       return CRC_PARAM_VALUE;
    case MAV_MSG_ATTITUDE:          return CRC_ATTITUDE;
    case MAV_MSG_COMMAND_LONG:      return CRC_COMMAND_LONG;
    case MAV_MSG_COMMAND_ACK:       return CRC_COMMAND_ACK;
    case MAV_MSG_AUTOPILOT_VERSION: return CRC_AUTOPILOT_VERSION;
    case MAV_MSG_MESSAGE_INTERVAL:  return CRC_MESSAGE_INTERVAL;
    default:                        return 0;
    }
}

/* ==================== 组包 ==================== */
uint16_t mav_frame_pack(uint8_t *frame, uint8_t sysid, uint8_t compid, uint8_t seq,
                        uint32_t msgid, const uint8_t *payload, uint8_t payload_len) {
    frame[0] = MAVLINK_STX;
    frame[1] = payload_len;
    frame[2] = 0;                    /* incompat_flags：0 表示无签名 */
    frame[3] = 0;                    /* compat_flags */
    frame[4] = seq;
    frame[5] = sysid;
    frame[6] = compid;
    frame[7] = (uint8_t)(msgid & 0xFF);
    frame[8] = (uint8_t)((msgid >> 8) & 0xFF);
    frame[9] = (uint8_t)((msgid >> 16) & 0xFF);

    if (payload_len) memcpy(frame + MAVLINK2_HEADER_LEN, payload, payload_len);

    /* checksum 覆盖范围：从 len 字节(offset 1)到 payload 结束，再加 CRC_EXTRA */
    uint16_t crc = mav_crc_calculate(frame + 1, (uint16_t)(MAVLINK2_HEADER_LEN - 1 + payload_len));
    crc = mav_crc_accumulate(mav_crc_extra(msgid), crc);

    uint16_t ck_off = MAVLINK2_HEADER_LEN + payload_len;
    frame[ck_off]     = (uint8_t)(crc & 0xFF);
    frame[ck_off + 1] = (uint8_t)(crc >> 8);

    return (uint16_t)(ck_off + MAVLINK_CHECKSUM_LEN);
}

uint16_t mav_pack_heartbeat(uint8_t *frame, uint8_t sysid, uint8_t compid, uint8_t seq,
                            uint8_t type, uint8_t autopilot, uint8_t base_mode,
                            uint32_t custom_mode, uint8_t system_status) {
    uint8_t p[9];
    mav_put_u32(p, 0, custom_mode);
    mav_put_u8 (p, 4, type);
    mav_put_u8 (p, 5, autopilot);
    mav_put_u8 (p, 6, base_mode);
    mav_put_u8 (p, 7, system_status);
    mav_put_u8 (p, 8, 3);            /* mavlink_version = 3（MAVLink 2） */
    return mav_frame_pack(frame, sysid, compid, seq, MAV_MSG_HEARTBEAT, p, 9);
}

uint16_t mav_pack_param_request_list(uint8_t *frame, uint8_t sysid, uint8_t compid, uint8_t seq,
                                     uint8_t target_system, uint8_t target_component) {
    uint8_t p[2];
    mav_put_u8(p, 0, target_system);
    mav_put_u8(p, 1, target_component);
    return mav_frame_pack(frame, sysid, compid, seq, MAV_MSG_PARAM_REQUEST_LIST, p, 2);
}

uint16_t mav_pack_command_long(uint8_t *frame, uint8_t sysid, uint8_t compid, uint8_t seq,
                               uint8_t target_system, uint8_t target_component, uint16_t command,
                               uint8_t confirmation, float p1, float p2, float p3, float p4,
                               float p5, float p6, float p7) {
    uint8_t p[33];
    mav_put_f32(p, 0, p1);
    mav_put_f32(p, 4, p2);
    mav_put_f32(p, 8, p3);
    mav_put_f32(p, 12, p4);
    mav_put_f32(p, 16, p5);
    mav_put_f32(p, 20, p6);
    mav_put_f32(p, 24, p7);
    mav_put_u16(p, 28, command);
    mav_put_u8 (p, 30, target_system);
    mav_put_u8 (p, 31, target_component);
    mav_put_u8 (p, 32, confirmation);
    return mav_frame_pack(frame, sysid, compid, seq, MAV_MSG_COMMAND_LONG, p, 33);
}

uint16_t mav_pack_message_interval(uint8_t *frame, uint8_t sysid, uint8_t compid, uint8_t seq,
                                   uint16_t message_id, int32_t interval_us) {
    uint8_t p[6];
    mav_put_i32(p, 0, interval_us);
    mav_put_u16(p, 4, message_id);
    return mav_frame_pack(frame, sysid, compid, seq, MAV_MSG_MESSAGE_INTERVAL, p, 6);
}

/* ==================== 解析 ==================== */
uint16_t mav_frame_parse(const uint8_t *frame, uint16_t frame_len,
                         uint8_t *sysid, uint8_t *compid, uint32_t *msgid,
                         const uint8_t **payload, uint8_t *payload_len) {
    if (frame_len < MAVLINK2_HEADER_LEN + MAVLINK_CHECKSUM_LEN) return 0;
    if (frame[0] != MAVLINK_STX) return 0;

    uint8_t len = frame[1];
    uint16_t total = (uint16_t)(MAVLINK2_HEADER_LEN + len + MAVLINK_CHECKSUM_LEN);
    if (frame_len < total || len > MAVLINK_MAX_PAYLOAD) return 0;

    uint32_t mid = (uint32_t)frame[7] | ((uint32_t)frame[8] << 8) | ((uint32_t)frame[9] << 16);

    /* 校验 checksum（含 CRC_EXTRA） */
    uint16_t crc = mav_crc_calculate(frame + 1, (uint16_t)(MAVLINK2_HEADER_LEN - 1 + len));
    crc = mav_crc_accumulate(mav_crc_extra(mid), crc);
    uint16_t recv = (uint16_t)(frame[MAVLINK2_HEADER_LEN + len] |
                              (frame[MAVLINK2_HEADER_LEN + len + 1] << 8));
    if (recv != crc) return 0;       /* checksum 不匹配 */

    if (sysid)      *sysid = frame[5];
    if (compid)     *compid = frame[6];
    if (msgid)      *msgid = mid;
    if (payload)    *payload = frame + MAVLINK2_HEADER_LEN;
    if (payload_len)*payload_len = len;
    return total;
}

void mav_parse_heartbeat(const uint8_t *p, uint8_t *type, uint8_t *autopilot,
                         uint8_t *base_mode, uint32_t *custom_mode, uint8_t *system_status) {
    if (custom_mode)   *custom_mode = mav_get_u32(p, 0);
    if (type)          *type = mav_get_u8(p, 4);
    if (autopilot)     *autopilot = mav_get_u8(p, 5);
    if (base_mode)     *base_mode = mav_get_u8(p, 6);
    if (system_status) *system_status = mav_get_u8(p, 7);
}

void mav_parse_param_value(const uint8_t *p, char *param_id, float *value,
                           uint8_t *type, uint16_t *count, uint16_t *index) {
    if (value) *value = mav_get_f32(p, 0);
    if (count) *count = mav_get_u16(p, 4);
    if (index) *index = mav_get_u16(p, 6);
    if (param_id) {
        memcpy(param_id, p + 8, 16);
        param_id[16] = '\0';
    }
    if (type) *type = mav_get_u8(p, 24);
}

void mav_parse_sys_status(const uint8_t *p, uint8_t len, uint16_t *voltage_mv,
                          int16_t *current_ca, int8_t *battery_remaining) {
    /* 关键字段都在 MIN_LEN(31) 内，len 不足则跳过 */
    if (len < 18) return;
    if (voltage_mv)       *voltage_mv = mav_get_u16(p, 14);
    if (current_ca)       *current_ca = (int16_t)mav_get_u16(p, 16);
    if (battery_remaining && len >= 31) *battery_remaining = (int8_t)mav_get_u8(p, 30);
}

void mav_parse_attitude(const uint8_t *p, float *roll, float *pitch, float *yaw) {
    if (roll)  *roll  = mav_get_f32(p, 4);
    if (pitch) *pitch = mav_get_f32(p, 8);
    if (yaw)   *yaw   = mav_get_f32(p, 12);
}

void mav_parse_autopilot_version(const uint8_t *p, uint8_t len,
                                 uint32_t *flight_sw, uint32_t *board_ver,
                                 uint16_t *vendor_id, uint16_t *product_id) {
    if (len < 36) return;
    if (flight_sw) *flight_sw = mav_get_u32(p, 16);
    if (board_ver) *board_ver = mav_get_u32(p, 28);
    if (vendor_id) *vendor_id = mav_get_u16(p, 32);
    if (product_id)*product_id = mav_get_u16(p, 34);
}

void mav_parse_command_ack(const uint8_t *p, uint16_t *command, uint8_t *result) {
    if (command) *command = mav_get_u16(p, 0);
    if (result)  *result = mav_get_u8(p, 2);
}
