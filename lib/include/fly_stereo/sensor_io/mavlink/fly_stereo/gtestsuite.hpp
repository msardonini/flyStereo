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


TEST(fly_stereo, IMU)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::fly_stereo::msg::IMU packet_in{};
    packet_in.timestamp_us = 93372036854775807ULL;
    packet_in.time_since_trigger_us = 963497880;
    packet_in.trigger_count = 963498088;
    packet_in.roll = 129.0;
    packet_in.pitch = 157.0;
    packet_in.yaw = 185.0;
    packet_in.gyroXYZ = {{ 213.0, 214.0, 215.0 }};
    packet_in.accelXYZ = {{ 297.0, 298.0, 299.0 }};

    mavlink::fly_stereo::msg::IMU packet1{};
    mavlink::fly_stereo::msg::IMU packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp_us, packet2.timestamp_us);
    EXPECT_EQ(packet1.time_since_trigger_us, packet2.time_since_trigger_us);
    EXPECT_EQ(packet1.trigger_count, packet2.trigger_count);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.gyroXYZ, packet2.gyroXYZ);
    EXPECT_EQ(packet1.accelXYZ, packet2.accelXYZ);
}

#ifdef TEST_INTEROP
TEST(fly_stereo_interop, IMU)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_imu_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 129.0, 157.0, 185.0, { 213.0, 214.0, 215.0 }, { 297.0, 298.0, 299.0 }
    };

    mavlink::fly_stereo::msg::IMU packet_in{};
    packet_in.timestamp_us = 93372036854775807ULL;
    packet_in.time_since_trigger_us = 963497880;
    packet_in.trigger_count = 963498088;
    packet_in.roll = 129.0;
    packet_in.pitch = 157.0;
    packet_in.yaw = 185.0;
    packet_in.gyroXYZ = {{ 213.0, 214.0, 215.0 }};
    packet_in.accelXYZ = {{ 297.0, 298.0, 299.0 }};

    mavlink::fly_stereo::msg::IMU packet2{};

    mavlink_msg_imu_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp_us, packet2.timestamp_us);
    EXPECT_EQ(packet_in.time_since_trigger_us, packet2.time_since_trigger_us);
    EXPECT_EQ(packet_in.trigger_count, packet2.trigger_count);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.gyroXYZ, packet2.gyroXYZ);
    EXPECT_EQ(packet_in.accelXYZ, packet2.accelXYZ);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(fly_stereo, RESET_COUNTERS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::fly_stereo::msg::RESET_COUNTERS packet_in{};
    packet_in.timestamp_us = 93372036854775807ULL;

    mavlink::fly_stereo::msg::RESET_COUNTERS packet1{};
    mavlink::fly_stereo::msg::RESET_COUNTERS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp_us, packet2.timestamp_us);
}

#ifdef TEST_INTEROP
TEST(fly_stereo_interop, RESET_COUNTERS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_reset_counters_t packet_c {
         93372036854775807ULL
    };

    mavlink::fly_stereo::msg::RESET_COUNTERS packet_in{};
    packet_in.timestamp_us = 93372036854775807ULL;

    mavlink::fly_stereo::msg::RESET_COUNTERS packet2{};

    mavlink_msg_reset_counters_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp_us, packet2.timestamp_us);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(fly_stereo, COMMAND)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::fly_stereo::msg::COMMAND packet_in{};
    packet_in.timestamp_us = 93372036854775807ULL;
    packet_in.engage = 65;
    packet_in.shutdown = 132;
    packet_in.custom_cmd1 = 963497880;
    packet_in.custom_cmd2 = 963498088;
    packet_in.custom_cmd3 = 963498296;

    mavlink::fly_stereo::msg::COMMAND packet1{};
    mavlink::fly_stereo::msg::COMMAND packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp_us, packet2.timestamp_us);
    EXPECT_EQ(packet1.engage, packet2.engage);
    EXPECT_EQ(packet1.shutdown, packet2.shutdown);
    EXPECT_EQ(packet1.custom_cmd1, packet2.custom_cmd1);
    EXPECT_EQ(packet1.custom_cmd2, packet2.custom_cmd2);
    EXPECT_EQ(packet1.custom_cmd3, packet2.custom_cmd3);
}

#ifdef TEST_INTEROP
TEST(fly_stereo_interop, COMMAND)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_command_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, 65, 132
    };

    mavlink::fly_stereo::msg::COMMAND packet_in{};
    packet_in.timestamp_us = 93372036854775807ULL;
    packet_in.engage = 65;
    packet_in.shutdown = 132;
    packet_in.custom_cmd1 = 963497880;
    packet_in.custom_cmd2 = 963498088;
    packet_in.custom_cmd3 = 963498296;

    mavlink::fly_stereo::msg::COMMAND packet2{};

    mavlink_msg_command_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp_us, packet2.timestamp_us);
    EXPECT_EQ(packet_in.engage, packet2.engage);
    EXPECT_EQ(packet_in.shutdown, packet2.shutdown);
    EXPECT_EQ(packet_in.custom_cmd1, packet2.custom_cmd1);
    EXPECT_EQ(packet_in.custom_cmd2, packet2.custom_cmd2);
    EXPECT_EQ(packet_in.custom_cmd3, packet2.custom_cmd3);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(fly_stereo, VIO)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::fly_stereo::msg::VIO packet_in{};
    packet_in.timestamp_us = 93372036854775807ULL;
    packet_in.position = {{ 73.0, 74.0, 75.0 }};
    packet_in.velocity = {{ 157.0, 158.0, 159.0 }};
    packet_in.quat = {{ 241.0, 242.0, 243.0, 244.0 }};

    mavlink::fly_stereo::msg::VIO packet1{};
    mavlink::fly_stereo::msg::VIO packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp_us, packet2.timestamp_us);
    EXPECT_EQ(packet1.position, packet2.position);
    EXPECT_EQ(packet1.velocity, packet2.velocity);
    EXPECT_EQ(packet1.quat, packet2.quat);
}

#ifdef TEST_INTEROP
TEST(fly_stereo_interop, VIO)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vio_t packet_c {
         93372036854775807ULL, { 73.0, 74.0, 75.0 }, { 157.0, 158.0, 159.0 }, { 241.0, 242.0, 243.0, 244.0 }
    };

    mavlink::fly_stereo::msg::VIO packet_in{};
    packet_in.timestamp_us = 93372036854775807ULL;
    packet_in.position = {{ 73.0, 74.0, 75.0 }};
    packet_in.velocity = {{ 157.0, 158.0, 159.0 }};
    packet_in.quat = {{ 241.0, 242.0, 243.0, 244.0 }};

    mavlink::fly_stereo::msg::VIO packet2{};

    mavlink_msg_vio_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp_us, packet2.timestamp_us);
    EXPECT_EQ(packet_in.position, packet2.position);
    EXPECT_EQ(packet_in.velocity, packet2.velocity);
    EXPECT_EQ(packet_in.quat, packet2.quat);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
