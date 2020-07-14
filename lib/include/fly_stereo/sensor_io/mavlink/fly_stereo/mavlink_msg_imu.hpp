// MESSAGE IMU support class

#pragma once

namespace mavlink {
namespace fly_stereo {
namespace msg {

/**
 * @brief IMU message
 *
 * Test all field types
 */
struct IMU : mavlink::Message {
    static constexpr msgid_t MSG_ID = 0;
    static constexpr size_t LENGTH = 48;
    static constexpr size_t MIN_LENGTH = 48;
    static constexpr uint8_t CRC_EXTRA = 32;
    static constexpr auto NAME = "IMU";


    uint64_t timestamp_us; /*< [us] timestamp since linux epoch */
    uint32_t trigger_count; /*<  counter of trigger pulses */
    float roll; /*< [rad] Roll angle (-pi..+pi) */
    float pitch; /*< [rad] Pitch angle (-pi..+pi) */
    float yaw; /*< [rad] Yaw angle (-pi..+pi) */
    std::array<float, 3> gyroXYZ; /*< [rad/s] Gyro output XYZ */
    std::array<float, 3> accelXYZ; /*< [rad/s] Acel output XYZ */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  timestamp_us: " << timestamp_us << std::endl;
        ss << "  trigger_count: " << trigger_count << std::endl;
        ss << "  roll: " << roll << std::endl;
        ss << "  pitch: " << pitch << std::endl;
        ss << "  yaw: " << yaw << std::endl;
        ss << "  gyroXYZ: [" << to_string(gyroXYZ) << "]" << std::endl;
        ss << "  accelXYZ: [" << to_string(accelXYZ) << "]" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << timestamp_us;                  // offset: 0
        map << trigger_count;                 // offset: 8
        map << roll;                          // offset: 12
        map << pitch;                         // offset: 16
        map << yaw;                           // offset: 20
        map << gyroXYZ;                       // offset: 24
        map << accelXYZ;                      // offset: 36
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> timestamp_us;                  // offset: 0
        map >> trigger_count;                 // offset: 8
        map >> roll;                          // offset: 12
        map >> pitch;                         // offset: 16
        map >> yaw;                           // offset: 20
        map >> gyroXYZ;                       // offset: 24
        map >> accelXYZ;                      // offset: 36
    }
};

} // namespace msg
} // namespace fly_stereo
} // namespace mavlink
