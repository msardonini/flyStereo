#pragma once
// MESSAGE RESET_COUNTERS PACKING

#define MAVLINK_MSG_ID_RESET_COUNTERS 1


typedef struct __mavlink_reset_counters_t {
 uint64_t timestamp_us; /*< [us] timestamp since linux epoch*/
} mavlink_reset_counters_t;

#define MAVLINK_MSG_ID_RESET_COUNTERS_LEN 8
#define MAVLINK_MSG_ID_RESET_COUNTERS_MIN_LEN 8
#define MAVLINK_MSG_ID_1_LEN 8
#define MAVLINK_MSG_ID_1_MIN_LEN 8

#define MAVLINK_MSG_ID_RESET_COUNTERS_CRC 68
#define MAVLINK_MSG_ID_1_CRC 68



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RESET_COUNTERS { \
    1, \
    "RESET_COUNTERS", \
    1, \
    {  { "timestamp_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_reset_counters_t, timestamp_us) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RESET_COUNTERS { \
    "RESET_COUNTERS", \
    1, \
    {  { "timestamp_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_reset_counters_t, timestamp_us) }, \
         } \
}
#endif

/**
 * @brief Pack a reset_counters message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_us [us] timestamp since linux epoch
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_reset_counters_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESET_COUNTERS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp_us);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESET_COUNTERS_LEN);
#else
    mavlink_reset_counters_t packet;
    packet.timestamp_us = timestamp_us;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESET_COUNTERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RESET_COUNTERS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RESET_COUNTERS_MIN_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_CRC);
}

/**
 * @brief Pack a reset_counters message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_us [us] timestamp since linux epoch
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_reset_counters_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESET_COUNTERS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp_us);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESET_COUNTERS_LEN);
#else
    mavlink_reset_counters_t packet;
    packet.timestamp_us = timestamp_us;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESET_COUNTERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RESET_COUNTERS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RESET_COUNTERS_MIN_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_CRC);
}

/**
 * @brief Encode a reset_counters struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param reset_counters C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_reset_counters_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_reset_counters_t* reset_counters)
{
    return mavlink_msg_reset_counters_pack(system_id, component_id, msg, reset_counters->timestamp_us);
}

/**
 * @brief Encode a reset_counters struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param reset_counters C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_reset_counters_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_reset_counters_t* reset_counters)
{
    return mavlink_msg_reset_counters_pack_chan(system_id, component_id, chan, msg, reset_counters->timestamp_us);
}

/**
 * @brief Send a reset_counters message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_us [us] timestamp since linux epoch
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_reset_counters_send(mavlink_channel_t chan, uint64_t timestamp_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESET_COUNTERS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp_us);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_COUNTERS, buf, MAVLINK_MSG_ID_RESET_COUNTERS_MIN_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_CRC);
#else
    mavlink_reset_counters_t packet;
    packet.timestamp_us = timestamp_us;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_COUNTERS, (const char *)&packet, MAVLINK_MSG_ID_RESET_COUNTERS_MIN_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_CRC);
#endif
}

/**
 * @brief Send a reset_counters message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_reset_counters_send_struct(mavlink_channel_t chan, const mavlink_reset_counters_t* reset_counters)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_reset_counters_send(chan, reset_counters->timestamp_us);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_COUNTERS, (const char *)reset_counters, MAVLINK_MSG_ID_RESET_COUNTERS_MIN_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_CRC);
#endif
}

#if MAVLINK_MSG_ID_RESET_COUNTERS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_reset_counters_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp_us);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_COUNTERS, buf, MAVLINK_MSG_ID_RESET_COUNTERS_MIN_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_CRC);
#else
    mavlink_reset_counters_t *packet = (mavlink_reset_counters_t *)msgbuf;
    packet->timestamp_us = timestamp_us;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_COUNTERS, (const char *)packet, MAVLINK_MSG_ID_RESET_COUNTERS_MIN_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_LEN, MAVLINK_MSG_ID_RESET_COUNTERS_CRC);
#endif
}
#endif

#endif

// MESSAGE RESET_COUNTERS UNPACKING


/**
 * @brief Get field timestamp_us from reset_counters message
 *
 * @return [us] timestamp since linux epoch
 */
static inline uint64_t mavlink_msg_reset_counters_get_timestamp_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Decode a reset_counters message into a struct
 *
 * @param msg The message to decode
 * @param reset_counters C-struct to decode the message contents into
 */
static inline void mavlink_msg_reset_counters_decode(const mavlink_message_t* msg, mavlink_reset_counters_t* reset_counters)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    reset_counters->timestamp_us = mavlink_msg_reset_counters_get_timestamp_us(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RESET_COUNTERS_LEN? msg->len : MAVLINK_MSG_ID_RESET_COUNTERS_LEN;
        memset(reset_counters, 0, MAVLINK_MSG_ID_RESET_COUNTERS_LEN);
    memcpy(reset_counters, _MAV_PAYLOAD(msg), len);
#endif
}
