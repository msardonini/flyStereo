/** @file
 *	@brief MAVLink comm protocol generated from fly_stereo.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace fly_stereo {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (trought @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 1> MESSAGE_ENTRIES {{ {0, 78, 36, 36, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 3;


// ENUM DEFINITIONS




} // namespace fly_stereo
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_attitude.hpp"

// base include

