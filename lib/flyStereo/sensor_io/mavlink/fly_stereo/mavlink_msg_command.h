#pragma once
// MESSAGE COMMAND PACKING

#define MAVLINK_MSG_ID_COMMAND 2

typedef struct __mavlink_command_t {
  uint64_t timestamp_us; /*< [us] timestamp since linux epoch*/
  uint32_t custom_cmd1;  /*< [custom] custom command*/
  uint32_t custom_cmd2;  /*< [custom] custom command*/
  uint32_t custom_cmd3;  /*< [custom] custom command*/
  uint8_t engage;        /*< [bool] flag to turn on*/
  uint8_t shutdown;      /*< [bool] flag to turn off*/
} mavlink_command_t;

#define MAVLINK_MSG_ID_COMMAND_LEN 22
#define MAVLINK_MSG_ID_COMMAND_MIN_LEN 22
#define MAVLINK_MSG_ID_2_LEN 22
#define MAVLINK_MSG_ID_2_MIN_LEN 22

#define MAVLINK_MSG_ID_COMMAND_CRC 12
#define MAVLINK_MSG_ID_2_CRC 12

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_COMMAND                                                                     \
  {                                                                                                      \
    2, "COMMAND", 6, {                                                                                   \
      {"timestamp_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_command_t, timestamp_us)},    \
          {"engage", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_command_t, engage)},            \
          {"shutdown", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_command_t, shutdown)},        \
          {"custom_cmd1", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_command_t, custom_cmd1)},  \
          {"custom_cmd2", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_command_t, custom_cmd2)}, \
          {"custom_cmd3", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_command_t, custom_cmd3)}, \
    }                                                                                                    \
  }
#else
#define MAVLINK_MESSAGE_INFO_COMMAND                                                                     \
  {                                                                                                      \
    "COMMAND", 6, {                                                                                      \
      {"timestamp_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_command_t, timestamp_us)},    \
          {"engage", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_command_t, engage)},            \
          {"shutdown", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_command_t, shutdown)},        \
          {"custom_cmd1", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_command_t, custom_cmd1)},  \
          {"custom_cmd2", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_command_t, custom_cmd2)}, \
          {"custom_cmd3", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_command_t, custom_cmd3)}, \
    }                                                                                                    \
  }
#endif

/**
 * @brief Pack a command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_us [us] timestamp since linux epoch
 * @param engage [bool] flag to turn on
 * @param shutdown [bool] flag to turn off
 * @param custom_cmd1 [custom] custom command
 * @param custom_cmd2 [custom] custom command
 * @param custom_cmd3 [custom] custom command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                                                uint64_t timestamp_us, uint8_t engage, uint8_t shutdown,
                                                uint32_t custom_cmd1, uint32_t custom_cmd2, uint32_t custom_cmd3) {
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
  char buf[MAVLINK_MSG_ID_COMMAND_LEN];
  _mav_put_uint64_t(buf, 0, timestamp_us);
  _mav_put_uint32_t(buf, 8, custom_cmd1);
  _mav_put_uint32_t(buf, 12, custom_cmd2);
  _mav_put_uint32_t(buf, 16, custom_cmd3);
  _mav_put_uint8_t(buf, 20, engage);
  _mav_put_uint8_t(buf, 21, shutdown);

  memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_LEN);
#else
  mavlink_command_t packet;
  packet.timestamp_us = timestamp_us;
  packet.custom_cmd1 = custom_cmd1;
  packet.custom_cmd2 = custom_cmd2;
  packet.custom_cmd3 = custom_cmd3;
  packet.engage = engage;
  packet.shutdown = shutdown;

  memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LEN);
#endif

  msg->msgid = MAVLINK_MSG_ID_COMMAND;
  return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMMAND_MIN_LEN,
                                  MAVLINK_MSG_ID_COMMAND_LEN, MAVLINK_MSG_ID_COMMAND_CRC);
}

/**
 * @brief Pack a command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_us [us] timestamp since linux epoch
 * @param engage [bool] flag to turn on
 * @param shutdown [bool] flag to turn off
 * @param custom_cmd1 [custom] custom command
 * @param custom_cmd2 [custom] custom command
 * @param custom_cmd3 [custom] custom command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                                                     mavlink_message_t* msg, uint64_t timestamp_us, uint8_t engage,
                                                     uint8_t shutdown, uint32_t custom_cmd1, uint32_t custom_cmd2,
                                                     uint32_t custom_cmd3) {
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
  char buf[MAVLINK_MSG_ID_COMMAND_LEN];
  _mav_put_uint64_t(buf, 0, timestamp_us);
  _mav_put_uint32_t(buf, 8, custom_cmd1);
  _mav_put_uint32_t(buf, 12, custom_cmd2);
  _mav_put_uint32_t(buf, 16, custom_cmd3);
  _mav_put_uint8_t(buf, 20, engage);
  _mav_put_uint8_t(buf, 21, shutdown);

  memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_LEN);
#else
  mavlink_command_t packet;
  packet.timestamp_us = timestamp_us;
  packet.custom_cmd1 = custom_cmd1;
  packet.custom_cmd2 = custom_cmd2;
  packet.custom_cmd3 = custom_cmd3;
  packet.engage = engage;
  packet.shutdown = shutdown;

  memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LEN);
#endif

  msg->msgid = MAVLINK_MSG_ID_COMMAND;
  return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMMAND_MIN_LEN,
                                       MAVLINK_MSG_ID_COMMAND_LEN, MAVLINK_MSG_ID_COMMAND_CRC);
}

/**
 * @brief Encode a command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                                                  const mavlink_command_t* command) {
  return mavlink_msg_command_pack(system_id, component_id, msg, command->timestamp_us, command->engage,
                                  command->shutdown, command->custom_cmd1, command->custom_cmd2, command->custom_cmd3);
}

/**
 * @brief Encode a command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                                                       mavlink_message_t* msg, const mavlink_command_t* command) {
  return mavlink_msg_command_pack_chan(system_id, component_id, chan, msg, command->timestamp_us, command->engage,
                                       command->shutdown, command->custom_cmd1, command->custom_cmd2,
                                       command->custom_cmd3);
}

/**
 * @brief Send a command message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_us [us] timestamp since linux epoch
 * @param engage [bool] flag to turn on
 * @param shutdown [bool] flag to turn off
 * @param custom_cmd1 [custom] custom command
 * @param custom_cmd2 [custom] custom command
 * @param custom_cmd3 [custom] custom command
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_send(mavlink_channel_t chan, uint64_t timestamp_us, uint8_t engage,
                                            uint8_t shutdown, uint32_t custom_cmd1, uint32_t custom_cmd2,
                                            uint32_t custom_cmd3) {
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
  char buf[MAVLINK_MSG_ID_COMMAND_LEN];
  _mav_put_uint64_t(buf, 0, timestamp_us);
  _mav_put_uint32_t(buf, 8, custom_cmd1);
  _mav_put_uint32_t(buf, 12, custom_cmd2);
  _mav_put_uint32_t(buf, 16, custom_cmd3);
  _mav_put_uint8_t(buf, 20, engage);
  _mav_put_uint8_t(buf, 21, shutdown);

  _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND, buf, MAVLINK_MSG_ID_COMMAND_MIN_LEN,
                                  MAVLINK_MSG_ID_COMMAND_LEN, MAVLINK_MSG_ID_COMMAND_CRC);
#else
  mavlink_command_t packet;
  packet.timestamp_us = timestamp_us;
  packet.custom_cmd1 = custom_cmd1;
  packet.custom_cmd2 = custom_cmd2;
  packet.custom_cmd3 = custom_cmd3;
  packet.engage = engage;
  packet.shutdown = shutdown;

  _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND, (const char*)&packet, MAVLINK_MSG_ID_COMMAND_MIN_LEN,
                                  MAVLINK_MSG_ID_COMMAND_LEN, MAVLINK_MSG_ID_COMMAND_CRC);
#endif
}

/**
 * @brief Send a command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_command_send_struct(mavlink_channel_t chan, const mavlink_command_t* command) {
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
  mavlink_msg_command_send(chan, command->timestamp_us, command->engage, command->shutdown, command->custom_cmd1,
                           command->custom_cmd2, command->custom_cmd3);
#else
  _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND, (const char*)command, MAVLINK_MSG_ID_COMMAND_MIN_LEN,
                                  MAVLINK_MSG_ID_COMMAND_LEN, MAVLINK_MSG_ID_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_command_send_buf(mavlink_message_t* msgbuf, mavlink_channel_t chan,
                                                uint64_t timestamp_us, uint8_t engage, uint8_t shutdown,
                                                uint32_t custom_cmd1, uint32_t custom_cmd2, uint32_t custom_cmd3) {
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
  char* buf = (char*)msgbuf;
  _mav_put_uint64_t(buf, 0, timestamp_us);
  _mav_put_uint32_t(buf, 8, custom_cmd1);
  _mav_put_uint32_t(buf, 12, custom_cmd2);
  _mav_put_uint32_t(buf, 16, custom_cmd3);
  _mav_put_uint8_t(buf, 20, engage);
  _mav_put_uint8_t(buf, 21, shutdown);

  _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND, buf, MAVLINK_MSG_ID_COMMAND_MIN_LEN,
                                  MAVLINK_MSG_ID_COMMAND_LEN, MAVLINK_MSG_ID_COMMAND_CRC);
#else
  mavlink_command_t* packet = (mavlink_command_t*)msgbuf;
  packet->timestamp_us = timestamp_us;
  packet->custom_cmd1 = custom_cmd1;
  packet->custom_cmd2 = custom_cmd2;
  packet->custom_cmd3 = custom_cmd3;
  packet->engage = engage;
  packet->shutdown = shutdown;

  _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND, (const char*)packet, MAVLINK_MSG_ID_COMMAND_MIN_LEN,
                                  MAVLINK_MSG_ID_COMMAND_LEN, MAVLINK_MSG_ID_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE COMMAND UNPACKING

/**
 * @brief Get field timestamp_us from command message
 *
 * @return [us] timestamp since linux epoch
 */
static inline uint64_t mavlink_msg_command_get_timestamp_us(const mavlink_message_t* msg) {
  return _MAV_RETURN_uint64_t(msg, 0);
}

/**
 * @brief Get field engage from command message
 *
 * @return [bool] flag to turn on
 */
static inline uint8_t mavlink_msg_command_get_engage(const mavlink_message_t* msg) {
  return _MAV_RETURN_uint8_t(msg, 20);
}

/**
 * @brief Get field shutdown from command message
 *
 * @return [bool] flag to turn off
 */
static inline uint8_t mavlink_msg_command_get_shutdown(const mavlink_message_t* msg) {
  return _MAV_RETURN_uint8_t(msg, 21);
}

/**
 * @brief Get field custom_cmd1 from command message
 *
 * @return [custom] custom command
 */
static inline uint32_t mavlink_msg_command_get_custom_cmd1(const mavlink_message_t* msg) {
  return _MAV_RETURN_uint32_t(msg, 8);
}

/**
 * @brief Get field custom_cmd2 from command message
 *
 * @return [custom] custom command
 */
static inline uint32_t mavlink_msg_command_get_custom_cmd2(const mavlink_message_t* msg) {
  return _MAV_RETURN_uint32_t(msg, 12);
}

/**
 * @brief Get field custom_cmd3 from command message
 *
 * @return [custom] custom command
 */
static inline uint32_t mavlink_msg_command_get_custom_cmd3(const mavlink_message_t* msg) {
  return _MAV_RETURN_uint32_t(msg, 16);
}

/**
 * @brief Decode a command message into a struct
 *
 * @param msg The message to decode
 * @param command C-struct to decode the message contents into
 */
static inline void mavlink_msg_command_decode(const mavlink_message_t* msg, mavlink_command_t* command) {
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
  command->timestamp_us = mavlink_msg_command_get_timestamp_us(msg);
  command->custom_cmd1 = mavlink_msg_command_get_custom_cmd1(msg);
  command->custom_cmd2 = mavlink_msg_command_get_custom_cmd2(msg);
  command->custom_cmd3 = mavlink_msg_command_get_custom_cmd3(msg);
  command->engage = mavlink_msg_command_get_engage(msg);
  command->shutdown = mavlink_msg_command_get_shutdown(msg);
#else
  uint8_t len = msg->len < MAVLINK_MSG_ID_COMMAND_LEN ? msg->len : MAVLINK_MSG_ID_COMMAND_LEN;
  memset(command, 0, MAVLINK_MSG_ID_COMMAND_LEN);
  memcpy(command, _MAV_PAYLOAD(msg), len);
#endif
}
