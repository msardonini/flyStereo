// MESSAGE ATTITUDE support class

#pragma once

namespace mavlink {
namespace fly_stereo {
namespace msg {

/**
 * @brief ATTITUDE message
 *
 * Test all field types
 */
struct ATTITUDE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 0;
    static constexpr size_t LENGTH = 36;
    static constexpr size_t MIN_LENGTH = 36;
    static constexpr uint8_t CRC_EXTRA = 170;
    static constexpr auto NAME = "ATTITUDE";


    uint64_t timestamp_us; /*< [us] timestamp since linux epoch */
    uint32_t trigger_count; /*<  counter of trigger pulses */
    float roll; /*< [rad] Roll angle (-pi..+pi) */
    float pitch; /*< [rad] Pitch angle (-pi..+pi) */
    float yaw; /*< [rad] Yaw angle (-pi..+pi) */
    float rollspeed; /*< [rad/s] Roll angular speed */
    float pitchspeed; /*< [rad/s] Pitch angular speed */
    float yawspeed; /*< [rad/s] Yaw angular speed */


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
        ss << "  rollspeed: " << rollspeed << std::endl;
        ss << "  pitchspeed: " << pitchspeed << std::endl;
        ss << "  yawspeed: " << yawspeed << std::endl;

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
        map << rollspeed;                     // offset: 24
        map << pitchspeed;                    // offset: 28
        map << yawspeed;                      // offset: 32
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> timestamp_us;                  // offset: 0
        map >> trigger_count;                 // offset: 8
        map >> roll;                          // offset: 12
        map >> pitch;                         // offset: 16
        map >> yaw;                           // offset: 20
        map >> rollspeed;                     // offset: 24
        map >> pitchspeed;                    // offset: 28
        map >> yawspeed;                      // offset: 32
    }
};

} // namespace msg
} // namespace fly_stereo
} // namespace mavlink
