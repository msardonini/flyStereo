/** @file
 *	@brief MAVLink comm testsuite protocol generated from fly_stereo.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "fly_stereo.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(fly_stereo, attitude)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::fly_stereo::msg::attitude packet_in{};
    packet_in.timestamp_us = 93372036854775807ULL;
    packet_in.trigger_count = 963497880;
    packet_in.roll = 101.0;
    packet_in.pitch = 129.0;
    packet_in.yaw = 157.0;
    packet_in.rollspeed = 185.0;
    packet_in.pitchspeed = 213.0;
    packet_in.yawspeed = 241.0;

    mavlink::fly_stereo::msg::attitude packet1{};
    mavlink::fly_stereo::msg::attitude packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp_us, packet2.timestamp_us);
    EXPECT_EQ(packet1.trigger_count, packet2.trigger_count);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
}

#ifdef TEST_INTEROP
TEST(fly_stereo_interop, attitude)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_t packet_c {
         93372036854775807ULL, 963497880, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0
    };

    mavlink::fly_stereo::msg::attitude packet_in{};
    packet_in.timestamp_us = 93372036854775807ULL;
    packet_in.trigger_count = 963497880;
    packet_in.roll = 101.0;
    packet_in.pitch = 129.0;
    packet_in.yaw = 157.0;
    packet_in.rollspeed = 185.0;
    packet_in.pitchspeed = 213.0;
    packet_in.yawspeed = 241.0;

    mavlink::fly_stereo::msg::attitude packet2{};

    mavlink_msg_attitude_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp_us, packet2.timestamp_us);
    EXPECT_EQ(packet_in.trigger_count, packet2.trigger_count);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
