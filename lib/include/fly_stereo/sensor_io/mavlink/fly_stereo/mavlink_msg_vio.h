#pragma once
// MESSAGE VIO PACKING

#define MAVLINK_MSG_ID_VIO 3


typedef struct __mavlink_vio_t {
 uint64_t timestamp_us; /*< [us] timestamp since linux epoch*/
 float position[3]; /*< [bool] 3D position vio output in XYZ in meters*/
 float velocity[3]; /*< [bool] 3D velocity vio output in XYZ in meters / sec*/
 float quat[4]; /*< [bool] Quaternion representation of rotation about XYZ axis*/
} mavlink_vio_t;

#define MAVLINK_MSG_ID_VIO_LEN 48
#define MAVLINK_MSG_ID_VIO_MIN_LEN 48
#define MAVLINK_MSG_ID_3_LEN 48
#define MAVLINK_MSG_ID_3_MIN_LEN 48

#define MAVLINK_MSG_ID_VIO_CRC 6
#define MAVLINK_MSG_ID_3_CRC 6

#define MAVLINK_MSG_VIO_FIELD_POSITION_LEN 3
#define MAVLINK_MSG_VIO_FIELD_VELOCITY_LEN 3
#define MAVLINK_MSG_VIO_FIELD_QUAT_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VIO { \
    3, \
    "VIO", \
    4, \
    {  { "timestamp_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vio_t, timestamp_us) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_vio_t, position) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 20, offsetof(mavlink_vio_t, velocity) }, \
         { "quat", NULL, MAVLINK_TYPE_FLOAT, 4, 32, offsetof(mavlink_vio_t, quat) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VIO { \
    "VIO", \
    4, \
    {  { "timestamp_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vio_t, timestamp_us) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_vio_t, position) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 20, offsetof(mavlink_vio_t, velocity) }, \
         { "quat", NULL, MAVLINK_TYPE_FLOAT, 4, 32, offsetof(mavlink_vio_t, quat) }, \
         } \
}
#endif

/**
 * @brief Pack a vio message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_us [us] timestamp since linux epoch
 * @param position [bool] 3D position vio output in XYZ in meters
 * @param velocity [bool] 3D velocity vio output in XYZ in meters / sec
 * @param quat [bool] Quaternion representation of rotation about XYZ axis
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vio_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp_us, const float *position, const float *velocity, const float *quat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIO_LEN];
    _mav_put_uint64_t(buf, 0, timestamp_us);
    _mav_put_float_array(buf, 8, position, 3);
    _mav_put_float_array(buf, 20, velocity, 3);
    _mav_put_float_array(buf, 32, quat, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIO_LEN);
#else
    mavlink_vio_t packet;
    packet.timestamp_us = timestamp_us;
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet.quat, quat, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VIO_MIN_LEN, MAVLINK_MSG_ID_VIO_LEN, MAVLINK_MSG_ID_VIO_CRC);
}

/**
 * @brief Pack a vio message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_us [us] timestamp since linux epoch
 * @param position [bool] 3D position vio output in XYZ in meters
 * @param velocity [bool] 3D velocity vio output in XYZ in meters / sec
 * @param quat [bool] Quaternion representation of rotation about XYZ axis
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vio_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp_us,const float *position,const float *velocity,const float *quat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIO_LEN];
    _mav_put_uint64_t(buf, 0, timestamp_us);
    _mav_put_float_array(buf, 8, position, 3);
    _mav_put_float_array(buf, 20, velocity, 3);
    _mav_put_float_array(buf, 32, quat, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIO_LEN);
#else
    mavlink_vio_t packet;
    packet.timestamp_us = timestamp_us;
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet.quat, quat, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VIO_MIN_LEN, MAVLINK_MSG_ID_VIO_LEN, MAVLINK_MSG_ID_VIO_CRC);
}

/**
 * @brief Encode a vio struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vio C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vio_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vio_t* vio)
{
    return mavlink_msg_vio_pack(system_id, component_id, msg, vio->timestamp_us, vio->position, vio->velocity, vio->quat);
}

/**
 * @brief Encode a vio struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vio C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vio_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vio_t* vio)
{
    return mavlink_msg_vio_pack_chan(system_id, component_id, chan, msg, vio->timestamp_us, vio->position, vio->velocity, vio->quat);
}

/**
 * @brief Send a vio message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_us [us] timestamp since linux epoch
 * @param position [bool] 3D position vio output in XYZ in meters
 * @param velocity [bool] 3D velocity vio output in XYZ in meters / sec
 * @param quat [bool] Quaternion representation of rotation about XYZ axis
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vio_send(mavlink_channel_t chan, uint64_t timestamp_us, const float *position, const float *velocity, const float *quat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIO_LEN];
    _mav_put_uint64_t(buf, 0, timestamp_us);
    _mav_put_float_array(buf, 8, position, 3);
    _mav_put_float_array(buf, 20, velocity, 3);
    _mav_put_float_array(buf, 32, quat, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIO, buf, MAVLINK_MSG_ID_VIO_MIN_LEN, MAVLINK_MSG_ID_VIO_LEN, MAVLINK_MSG_ID_VIO_CRC);
#else
    mavlink_vio_t packet;
    packet.timestamp_us = timestamp_us;
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet.quat, quat, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIO, (const char *)&packet, MAVLINK_MSG_ID_VIO_MIN_LEN, MAVLINK_MSG_ID_VIO_LEN, MAVLINK_MSG_ID_VIO_CRC);
#endif
}

/**
 * @brief Send a vio message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vio_send_struct(mavlink_channel_t chan, const mavlink_vio_t* vio)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vio_send(chan, vio->timestamp_us, vio->position, vio->velocity, vio->quat);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIO, (const char *)vio, MAVLINK_MSG_ID_VIO_MIN_LEN, MAVLINK_MSG_ID_VIO_LEN, MAVLINK_MSG_ID_VIO_CRC);
#endif
}

#if MAVLINK_MSG_ID_VIO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vio_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp_us, const float *position, const float *velocity, const float *quat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp_us);
    _mav_put_float_array(buf, 8, position, 3);
    _mav_put_float_array(buf, 20, velocity, 3);
    _mav_put_float_array(buf, 32, quat, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIO, buf, MAVLINK_MSG_ID_VIO_MIN_LEN, MAVLINK_MSG_ID_VIO_LEN, MAVLINK_MSG_ID_VIO_CRC);
#else
    mavlink_vio_t *packet = (mavlink_vio_t *)msgbuf;
    packet->timestamp_us = timestamp_us;
    mav_array_memcpy(packet->position, position, sizeof(float)*3);
    mav_array_memcpy(packet->velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet->quat, quat, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIO, (const char *)packet, MAVLINK_MSG_ID_VIO_MIN_LEN, MAVLINK_MSG_ID_VIO_LEN, MAVLINK_MSG_ID_VIO_CRC);
#endif
}
#endif

#endif

// MESSAGE VIO UNPACKING


/**
 * @brief Get field timestamp_us from vio message
 *
 * @return [us] timestamp since linux epoch
 */
static inline uint64_t mavlink_msg_vio_get_timestamp_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field position from vio message
 *
 * @return [bool] 3D position vio output in XYZ in meters
 */
static inline uint16_t mavlink_msg_vio_get_position(const mavlink_message_t* msg, float *position)
{
    return _MAV_RETURN_float_array(msg, position, 3,  8);
}

/**
 * @brief Get field velocity from vio message
 *
 * @return [bool] 3D velocity vio output in XYZ in meters / sec
 */
static inline uint16_t mavlink_msg_vio_get_velocity(const mavlink_message_t* msg, float *velocity)
{
    return _MAV_RETURN_float_array(msg, velocity, 3,  20);
}

/**
 * @brief Get field quat from vio message
 *
 * @return [bool] Quaternion representation of rotation about XYZ axis
 */
static inline uint16_t mavlink_msg_vio_get_quat(const mavlink_message_t* msg, float *quat)
{
    return _MAV_RETURN_float_array(msg, quat, 4,  32);
}

/**
 * @brief Decode a vio message into a struct
 *
 * @param msg The message to decode
 * @param vio C-struct to decode the message contents into
 */
static inline void mavlink_msg_vio_decode(const mavlink_message_t* msg, mavlink_vio_t* vio)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    vio->timestamp_us = mavlink_msg_vio_get_timestamp_us(msg);
    mavlink_msg_vio_get_position(msg, vio->position);
    mavlink_msg_vio_get_velocity(msg, vio->velocity);
    mavlink_msg_vio_get_quat(msg, vio->quat);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VIO_LEN? msg->len : MAVLINK_MSG_ID_VIO_LEN;
        memset(vio, 0, MAVLINK_MSG_ID_VIO_LEN);
    memcpy(vio, _MAV_PAYLOAD(msg), len);
#endif
}
