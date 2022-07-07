/** @file
 *  @brief MAVLink comm protocol generated from fly_stereo.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_FLY_STEREO_H
#define MAVLINK_FLY_STEREO_H

#ifndef MAVLINK_H
#error Wrong include order: MAVLINK_FLY_STEREO.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS \
  {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS                                                                               \
  {                                                                                                        \
    {0, 181, 52, 52, 0, 0, 0}, {1, 68, 8, 8, 0, 0, 0}, {2, 12, 22, 22, 0, 0, 0}, { 3, 6, 48, 48, 0, 0, 0 } \
  }
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_FLY_STEREO

// ENUM DEFINITIONS

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_command.h"
#include "./mavlink_msg_imu.h"
#include "./mavlink_msg_reset_counters.h"
#include "./mavlink_msg_vio.h"

// base include

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
#define MAVLINK_MESSAGE_INFO                                                                     \
  {                                                                                              \
    MAVLINK_MESSAGE_INFO_IMU, MAVLINK_MESSAGE_INFO_RESET_COUNTERS, MAVLINK_MESSAGE_INFO_COMMAND, \
        MAVLINK_MESSAGE_INFO_VIO                                                                 \
  }
#define MAVLINK_MESSAGE_NAMES                                       \
  {                                                                 \
    {"COMMAND", 2}, {"IMU", 0}, {"RESET_COUNTERS", 1}, { "VIO", 3 } \
  }
#if MAVLINK_COMMAND_24BIT
#include "../mavlink_get_info.h"
#endif
#endif

#ifdef __cplusplus
}
#endif  // __cplusplus
#endif  // MAVLINK_FLY_STEREO_H
