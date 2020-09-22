/** @file
 *    @brief MAVLink comm protocol testsuite generated from fly_stereo.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef FLY_STEREO_TESTSUITE_H
#define FLY_STEREO_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_fly_stereo(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_fly_stereo(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_IMU >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_imu_t packet_in = {
        93372036854775807ULL,963497880,963498088,129.0,157.0,185.0,{ 213.0, 214.0, 215.0 },{ 297.0, 298.0, 299.0 }
    };
    mavlink_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp_us = packet_in.timestamp_us;
        packet1.time_since_trigger_us = packet_in.time_since_trigger_us;
        packet1.trigger_count = packet_in.trigger_count;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        
        mav_array_memcpy(packet1.gyroXYZ, packet_in.gyroXYZ, sizeof(float)*3);
        mav_array_memcpy(packet1.accelXYZ, packet_in.accelXYZ, sizeof(float)*3);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_IMU_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_IMU_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_pack(system_id, component_id, &msg , packet1.timestamp_us , packet1.time_since_trigger_us , packet1.trigger_count , packet1.roll , packet1.pitch , packet1.yaw , packet1.gyroXYZ , packet1.accelXYZ );
    mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp_us , packet1.time_since_trigger_us , packet1.trigger_count , packet1.roll , packet1.pitch , packet1.yaw , packet1.gyroXYZ , packet1.accelXYZ );
    mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_send(MAVLINK_COMM_1 , packet1.timestamp_us , packet1.time_since_trigger_us , packet1.trigger_count , packet1.roll , packet1.pitch , packet1.yaw , packet1.gyroXYZ , packet1.accelXYZ );
    mavlink_msg_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_reset_counters(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RESET_COUNTERS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_reset_counters_t packet_in = {
        93372036854775807ULL
    };
    mavlink_reset_counters_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp_us = packet_in.timestamp_us;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RESET_COUNTERS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RESET_COUNTERS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reset_counters_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_reset_counters_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reset_counters_pack(system_id, component_id, &msg , packet1.timestamp_us );
    mavlink_msg_reset_counters_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reset_counters_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp_us );
    mavlink_msg_reset_counters_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_reset_counters_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reset_counters_send(MAVLINK_COMM_1 , packet1.timestamp_us );
    mavlink_msg_reset_counters_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMMAND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_command_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,65,132
    };
    mavlink_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp_us = packet_in.timestamp_us;
        packet1.custom_cmd1 = packet_in.custom_cmd1;
        packet1.custom_cmd2 = packet_in.custom_cmd2;
        packet1.custom_cmd3 = packet_in.custom_cmd3;
        packet1.engage = packet_in.engage;
        packet1.shutdown = packet_in.shutdown;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COMMAND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMMAND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_pack(system_id, component_id, &msg , packet1.timestamp_us , packet1.engage , packet1.shutdown , packet1.custom_cmd1 , packet1.custom_cmd2 , packet1.custom_cmd3 );
    mavlink_msg_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp_us , packet1.engage , packet1.shutdown , packet1.custom_cmd1 , packet1.custom_cmd2 , packet1.custom_cmd3 );
    mavlink_msg_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_send(MAVLINK_COMM_1 , packet1.timestamp_us , packet1.engage , packet1.shutdown , packet1.custom_cmd1 , packet1.custom_cmd2 , packet1.custom_cmd3 );
    mavlink_msg_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_vio(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_vio_t packet_in = {
        93372036854775807ULL,{ 73.0, 74.0, 75.0 },{ 157.0, 158.0, 159.0 },{ 241.0, 242.0, 243.0, 244.0 }
    };
    mavlink_vio_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp_us = packet_in.timestamp_us;
        
        mav_array_memcpy(packet1.position, packet_in.position, sizeof(float)*3);
        mav_array_memcpy(packet1.velocity, packet_in.velocity, sizeof(float)*3);
        mav_array_memcpy(packet1.quat, packet_in.quat, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vio_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_vio_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vio_pack(system_id, component_id, &msg , packet1.timestamp_us , packet1.position , packet1.velocity , packet1.quat );
    mavlink_msg_vio_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vio_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp_us , packet1.position , packet1.velocity , packet1.quat );
    mavlink_msg_vio_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_vio_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vio_send(MAVLINK_COMM_1 , packet1.timestamp_us , packet1.position , packet1.velocity , packet1.quat );
    mavlink_msg_vio_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_fly_stereo(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_imu(system_id, component_id, last_msg);
    mavlink_test_reset_counters(system_id, component_id, last_msg);
    mavlink_test_command(system_id, component_id, last_msg);
    mavlink_test_vio(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // FLY_STEREO_TESTSUITE_H
