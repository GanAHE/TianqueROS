#pragma once
// MESSAGE GIMBAL_MANAGER_STATUS PACKING

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS 281

MAVPACKED(
typedef struct __mavlink_gimbal_manager_status_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint32_t flags; /*<  High level gimbal manager flags currently applied.*/
}) mavlink_gimbal_manager_status_t;

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN 8
#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN 8
#define MAVLINK_MSG_ID_281_LEN 8
#define MAVLINK_MSG_ID_281_MIN_LEN 8

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC 252
#define MAVLINK_MSG_ID_281_CRC 252



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_STATUS { \
    281, \
    "GIMBAL_MANAGER_STATUS", \
    2, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gimbal_manager_status_t, time_boot_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_gimbal_manager_status_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_STATUS { \
    "GIMBAL_MANAGER_STATUS", \
    2, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gimbal_manager_status_t, time_boot_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_gimbal_manager_status_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_manager_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  High level gimbal manager flags currently applied.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_manager_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint32_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
#else
    mavlink_gimbal_manager_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
}

/**
 * @brief Pack a gimbal_manager_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  High level gimbal manager flags currently applied.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_manager_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint32_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
#else
    mavlink_gimbal_manager_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
}

/**
 * @brief Encode a gimbal_manager_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_manager_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_manager_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_manager_status_t* gimbal_manager_status)
{
    return mavlink_msg_gimbal_manager_status_pack(system_id, component_id, msg, gimbal_manager_status->time_boot_ms, gimbal_manager_status->flags);
}

/**
 * @brief Encode a gimbal_manager_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_manager_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_manager_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_manager_status_t* gimbal_manager_status)
{
    return mavlink_msg_gimbal_manager_status_pack_chan(system_id, component_id, chan, msg, gimbal_manager_status->time_boot_ms, gimbal_manager_status->flags);
}

/**
 * @brief Send a gimbal_manager_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  High level gimbal manager flags currently applied.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_manager_status_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint32_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
#else
    mavlink_gimbal_manager_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
#endif
}

/**
 * @brief Send a gimbal_manager_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_manager_status_send_struct(mavlink_channel_t chan, const mavlink_gimbal_manager_status_t* gimbal_manager_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_manager_status_send(chan, gimbal_manager_status->time_boot_ms, gimbal_manager_status->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (const char *)gimbal_manager_status, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_manager_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint32_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
#else
    mavlink_gimbal_manager_status_t *packet = (mavlink_gimbal_manager_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_MANAGER_STATUS UNPACKING


/**
 * @brief Get field time_boot_ms from gimbal_manager_status message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_gimbal_manager_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field flags from gimbal_manager_status message
 *
 * @return  High level gimbal manager flags currently applied.
 */
static inline uint32_t mavlink_msg_gimbal_manager_status_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a gimbal_manager_status message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_manager_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_manager_status_decode(const mavlink_message_t* msg, mavlink_gimbal_manager_status_t* gimbal_manager_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_manager_status->time_boot_ms = mavlink_msg_gimbal_manager_status_get_time_boot_ms(msg);
    gimbal_manager_status->flags = mavlink_msg_gimbal_manager_status_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN;
        memset(gimbal_manager_status, 0, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
    memcpy(gimbal_manager_status, _MAV_PAYLOAD(msg), len);
#endif
}
