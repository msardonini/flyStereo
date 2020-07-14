#pragma once
// MESSAGE IMU PACKING

#define MAVLINK_MSG_ID_IMU 0


typedef struct __mavlink_imu_t {
 uint64_t timestamp_us; /*< [us] timestamp since linux epoch*/
 uint32_t trigger_count; /*<  counter of trigger pulses*/
 float roll; /*< [rad] Roll angle (-pi..+pi)*/
 float pitch; /*< [rad] Pitch angle (-pi..+pi)*/
 float yaw; /*< [rad] Yaw angle (-pi..+pi)*/
 float gyroXYZ[3]; /*< [rad/s] Gyro output XYZ*/
 float accelXYZ[3]; /*< [rad/s] Acel output XYZ*/
} mavlink_imu_t;

#define MAVLINK_MSG_ID_IMU_LEN 48
#define MAVLINK_MSG_ID_IMU_MIN_LEN 48
#define MAVLINK_MSG_ID_0_LEN 48
#define MAVLINK_MSG_ID_0_MIN_LEN 48

#define MAVLINK_MSG_ID_IMU_CRC 32
#define MAVLINK_MSG_ID_0_CRC 32

#define MAVLINK_MSG_IMU_FIELD_GYROXYZ_LEN 3
#define MAVLINK_MSG_IMU_FIELD_ACCELXYZ_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IMU { \
    0, \
    "IMU", \
    7, \
    {  { "timestamp_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_imu_t, timestamp_us) }, \
         { "trigger_count", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_imu_t, trigger_count) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_imu_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_imu_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_imu_t, yaw) }, \
         { "gyroXYZ", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_imu_t, gyroXYZ) }, \
         { "accelXYZ", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_imu_t, accelXYZ) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IMU { \
    "IMU", \
    7, \
    {  { "timestamp_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_imu_t, timestamp_us) }, \
         { "trigger_count", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_imu_t, trigger_count) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_imu_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_imu_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_imu_t, yaw) }, \
         { "gyroXYZ", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_imu_t, gyroXYZ) }, \
         { "accelXYZ", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_imu_t, accelXYZ) }, \
         } \
}
#endif

/**
 * @brief Pack a imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_us [us] timestamp since linux epoch
 * @param trigger_count  counter of trigger pulses
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param gyroXYZ [rad/s] Gyro output XYZ
 * @param accelXYZ [rad/s] Acel output XYZ
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp_us, uint32_t trigger_count, float roll, float pitch, float yaw, const float *gyroXYZ, const float *accelXYZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_LEN];
    _mav_put_uint64_t(buf, 0, timestamp_us);
    _mav_put_uint32_t(buf, 8, trigger_count);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);
    _mav_put_float_array(buf, 24, gyroXYZ, 3);
    _mav_put_float_array(buf, 36, accelXYZ, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_LEN);
#else
    mavlink_imu_t packet;
    packet.timestamp_us = timestamp_us;
    packet.trigger_count = trigger_count;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    mav_array_memcpy(packet.gyroXYZ, gyroXYZ, sizeof(float)*3);
    mav_array_memcpy(packet.accelXYZ, accelXYZ, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
}

/**
 * @brief Pack a imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_us [us] timestamp since linux epoch
 * @param trigger_count  counter of trigger pulses
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param gyroXYZ [rad/s] Gyro output XYZ
 * @param accelXYZ [rad/s] Acel output XYZ
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp_us,uint32_t trigger_count,float roll,float pitch,float yaw,const float *gyroXYZ,const float *accelXYZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_LEN];
    _mav_put_uint64_t(buf, 0, timestamp_us);
    _mav_put_uint32_t(buf, 8, trigger_count);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);
    _mav_put_float_array(buf, 24, gyroXYZ, 3);
    _mav_put_float_array(buf, 36, accelXYZ, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_LEN);
#else
    mavlink_imu_t packet;
    packet.timestamp_us = timestamp_us;
    packet.trigger_count = trigger_count;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    mav_array_memcpy(packet.gyroXYZ, gyroXYZ, sizeof(float)*3);
    mav_array_memcpy(packet.accelXYZ, accelXYZ, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
}

/**
 * @brief Encode a imu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_imu_t* imu)
{
    return mavlink_msg_imu_pack(system_id, component_id, msg, imu->timestamp_us, imu->trigger_count, imu->roll, imu->pitch, imu->yaw, imu->gyroXYZ, imu->accelXYZ);
}

/**
 * @brief Encode a imu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_imu_t* imu)
{
    return mavlink_msg_imu_pack_chan(system_id, component_id, chan, msg, imu->timestamp_us, imu->trigger_count, imu->roll, imu->pitch, imu->yaw, imu->gyroXYZ, imu->accelXYZ);
}

/**
 * @brief Send a imu message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_us [us] timestamp since linux epoch
 * @param trigger_count  counter of trigger pulses
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param gyroXYZ [rad/s] Gyro output XYZ
 * @param accelXYZ [rad/s] Acel output XYZ
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_imu_send(mavlink_channel_t chan, uint64_t timestamp_us, uint32_t trigger_count, float roll, float pitch, float yaw, const float *gyroXYZ, const float *accelXYZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_LEN];
    _mav_put_uint64_t(buf, 0, timestamp_us);
    _mav_put_uint32_t(buf, 8, trigger_count);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);
    _mav_put_float_array(buf, 24, gyroXYZ, 3);
    _mav_put_float_array(buf, 36, accelXYZ, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, buf, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#else
    mavlink_imu_t packet;
    packet.timestamp_us = timestamp_us;
    packet.trigger_count = trigger_count;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    mav_array_memcpy(packet.gyroXYZ, gyroXYZ, sizeof(float)*3);
    mav_array_memcpy(packet.accelXYZ, accelXYZ, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, (const char *)&packet, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#endif
}

/**
 * @brief Send a imu message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_imu_send_struct(mavlink_channel_t chan, const mavlink_imu_t* imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_imu_send(chan, imu->timestamp_us, imu->trigger_count, imu->roll, imu->pitch, imu->yaw, imu->gyroXYZ, imu->accelXYZ);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, (const char *)imu, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#endif
}

#if MAVLINK_MSG_ID_IMU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_imu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp_us, uint32_t trigger_count, float roll, float pitch, float yaw, const float *gyroXYZ, const float *accelXYZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp_us);
    _mav_put_uint32_t(buf, 8, trigger_count);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);
    _mav_put_float_array(buf, 24, gyroXYZ, 3);
    _mav_put_float_array(buf, 36, accelXYZ, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, buf, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#else
    mavlink_imu_t *packet = (mavlink_imu_t *)msgbuf;
    packet->timestamp_us = timestamp_us;
    packet->trigger_count = trigger_count;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    mav_array_memcpy(packet->gyroXYZ, gyroXYZ, sizeof(float)*3);
    mav_array_memcpy(packet->accelXYZ, accelXYZ, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, (const char *)packet, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#endif
}
#endif

#endif

// MESSAGE IMU UNPACKING


/**
 * @brief Get field timestamp_us from imu message
 *
 * @return [us] timestamp since linux epoch
 */
static inline uint64_t mavlink_msg_imu_get_timestamp_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field trigger_count from imu message
 *
 * @return  counter of trigger pulses
 */
static inline uint32_t mavlink_msg_imu_get_trigger_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field roll from imu message
 *
 * @return [rad] Roll angle (-pi..+pi)
 */
static inline float mavlink_msg_imu_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitch from imu message
 *
 * @return [rad] Pitch angle (-pi..+pi)
 */
static inline float mavlink_msg_imu_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yaw from imu message
 *
 * @return [rad] Yaw angle (-pi..+pi)
 */
static inline float mavlink_msg_imu_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field gyroXYZ from imu message
 *
 * @return [rad/s] Gyro output XYZ
 */
static inline uint16_t mavlink_msg_imu_get_gyroXYZ(const mavlink_message_t* msg, float *gyroXYZ)
{
    return _MAV_RETURN_float_array(msg, gyroXYZ, 3,  24);
}

/**
 * @brief Get field accelXYZ from imu message
 *
 * @return [rad/s] Acel output XYZ
 */
static inline uint16_t mavlink_msg_imu_get_accelXYZ(const mavlink_message_t* msg, float *accelXYZ)
{
    return _MAV_RETURN_float_array(msg, accelXYZ, 3,  36);
}

/**
 * @brief Decode a imu message into a struct
 *
 * @param msg The message to decode
 * @param imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_imu_decode(const mavlink_message_t* msg, mavlink_imu_t* imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    imu->timestamp_us = mavlink_msg_imu_get_timestamp_us(msg);
    imu->trigger_count = mavlink_msg_imu_get_trigger_count(msg);
    imu->roll = mavlink_msg_imu_get_roll(msg);
    imu->pitch = mavlink_msg_imu_get_pitch(msg);
    imu->yaw = mavlink_msg_imu_get_yaw(msg);
    mavlink_msg_imu_get_gyroXYZ(msg, imu->gyroXYZ);
    mavlink_msg_imu_get_accelXYZ(msg, imu->accelXYZ);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IMU_LEN? msg->len : MAVLINK_MSG_ID_IMU_LEN;
        memset(imu, 0, MAVLINK_MSG_ID_IMU_LEN);
    memcpy(imu, _MAV_PAYLOAD(msg), len);
#endif
}
