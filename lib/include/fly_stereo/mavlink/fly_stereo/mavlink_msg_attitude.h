#pragma once
// MESSAGE ATTITUDE PACKING

#define MAVLINK_MSG_ID_ATTITUDE 0


typedef struct __mavlink_attitude_t {
 uint64_t timestamp_us; /*< [us] timestamp since linux epoch*/
 uint32_t trigger_count; /*<  counter of trigger pulses*/
 float roll; /*< [rad] Roll angle (-pi..+pi)*/
 float pitch; /*< [rad] Pitch angle (-pi..+pi)*/
 float yaw; /*< [rad] Yaw angle (-pi..+pi)*/
 float rollspeed; /*< [rad/s] Roll angular speed*/
 float pitchspeed; /*< [rad/s] Pitch angular speed*/
 float yawspeed; /*< [rad/s] Yaw angular speed*/
} mavlink_attitude_t;

#define MAVLINK_MSG_ID_ATTITUDE_LEN 36
#define MAVLINK_MSG_ID_ATTITUDE_MIN_LEN 36
#define MAVLINK_MSG_ID_0_LEN 36
#define MAVLINK_MSG_ID_0_MIN_LEN 36

#define MAVLINK_MSG_ID_ATTITUDE_CRC 170
#define MAVLINK_MSG_ID_0_CRC 170



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATTITUDE { \
    0, \
    "ATTITUDE", \
    8, \
    {  { "timestamp_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_attitude_t, timestamp_us) }, \
         { "trigger_count", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_attitude_t, trigger_count) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_attitude_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_attitude_t, yawspeed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATTITUDE { \
    "ATTITUDE", \
    8, \
    {  { "timestamp_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_attitude_t, timestamp_us) }, \
         { "trigger_count", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_attitude_t, trigger_count) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_attitude_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_attitude_t, yawspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_us [us] timestamp since linux epoch
 * @param trigger_count  counter of trigger pulses
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp_us, uint32_t trigger_count, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp_us);
    _mav_put_uint32_t(buf, 8, trigger_count);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_LEN);
#else
    mavlink_attitude_t packet;
    packet.timestamp_us = timestamp_us;
    packet.trigger_count = trigger_count;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
}

/**
 * @brief Pack a attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_us [us] timestamp since linux epoch
 * @param trigger_count  counter of trigger pulses
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp_us,uint32_t trigger_count,float roll,float pitch,float yaw,float rollspeed,float pitchspeed,float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp_us);
    _mav_put_uint32_t(buf, 8, trigger_count);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_LEN);
#else
    mavlink_attitude_t packet;
    packet.timestamp_us = timestamp_us;
    packet.trigger_count = trigger_count;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
}

/**
 * @brief Encode a attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_t* attitude)
{
    return mavlink_msg_attitude_pack(system_id, component_id, msg, attitude->timestamp_us, attitude->trigger_count, attitude->roll, attitude->pitch, attitude->yaw, attitude->rollspeed, attitude->pitchspeed, attitude->yawspeed);
}

/**
 * @brief Encode a attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude_t* attitude)
{
    return mavlink_msg_attitude_pack_chan(system_id, component_id, chan, msg, attitude->timestamp_us, attitude->trigger_count, attitude->roll, attitude->pitch, attitude->yaw, attitude->rollspeed, attitude->pitchspeed, attitude->yawspeed);
}

/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_us [us] timestamp since linux epoch
 * @param trigger_count  counter of trigger pulses
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_send(mavlink_channel_t chan, uint64_t timestamp_us, uint32_t trigger_count, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp_us);
    _mav_put_uint32_t(buf, 8, trigger_count);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, buf, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#else
    mavlink_attitude_t packet;
    packet.timestamp_us = timestamp_us;
    packet.trigger_count = trigger_count;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#endif
}

/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_attitude_send_struct(mavlink_channel_t chan, const mavlink_attitude_t* attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_attitude_send(chan, attitude->timestamp_us, attitude->trigger_count, attitude->roll, attitude->pitch, attitude->yaw, attitude->rollspeed, attitude->pitchspeed, attitude->yawspeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)attitude, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_attitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp_us, uint32_t trigger_count, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp_us);
    _mav_put_uint32_t(buf, 8, trigger_count);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, buf, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#else
    mavlink_attitude_t *packet = (mavlink_attitude_t *)msgbuf;
    packet->timestamp_us = timestamp_us;
    packet->trigger_count = trigger_count;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->rollspeed = rollspeed;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE UNPACKING


/**
 * @brief Get field timestamp_us from attitude message
 *
 * @return [us] timestamp since linux epoch
 */
static inline uint64_t mavlink_msg_attitude_get_timestamp_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field trigger_count from attitude message
 *
 * @return  counter of trigger pulses
 */
static inline uint32_t mavlink_msg_attitude_get_trigger_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field roll from attitude message
 *
 * @return [rad] Roll angle (-pi..+pi)
 */
static inline float mavlink_msg_attitude_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitch from attitude message
 *
 * @return [rad] Pitch angle (-pi..+pi)
 */
static inline float mavlink_msg_attitude_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yaw from attitude message
 *
 * @return [rad] Yaw angle (-pi..+pi)
 */
static inline float mavlink_msg_attitude_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field rollspeed from attitude message
 *
 * @return [rad/s] Roll angular speed
 */
static inline float mavlink_msg_attitude_get_rollspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field pitchspeed from attitude message
 *
 * @return [rad/s] Pitch angular speed
 */
static inline float mavlink_msg_attitude_get_pitchspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field yawspeed from attitude message
 *
 * @return [rad/s] Yaw angular speed
 */
static inline float mavlink_msg_attitude_get_yawspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a attitude message into a struct
 *
 * @param msg The message to decode
 * @param attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    attitude->timestamp_us = mavlink_msg_attitude_get_timestamp_us(msg);
    attitude->trigger_count = mavlink_msg_attitude_get_trigger_count(msg);
    attitude->roll = mavlink_msg_attitude_get_roll(msg);
    attitude->pitch = mavlink_msg_attitude_get_pitch(msg);
    attitude->yaw = mavlink_msg_attitude_get_yaw(msg);
    attitude->rollspeed = mavlink_msg_attitude_get_rollspeed(msg);
    attitude->pitchspeed = mavlink_msg_attitude_get_pitchspeed(msg);
    attitude->yawspeed = mavlink_msg_attitude_get_yawspeed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATTITUDE_LEN? msg->len : MAVLINK_MSG_ID_ATTITUDE_LEN;
        memset(attitude, 0, MAVLINK_MSG_ID_ATTITUDE_LEN);
    memcpy(attitude, _MAV_PAYLOAD(msg), len);
#endif
}
