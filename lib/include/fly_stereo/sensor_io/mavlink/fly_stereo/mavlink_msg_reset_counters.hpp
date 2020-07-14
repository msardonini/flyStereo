// MESSAGE RESET_COUNTERS support class

#pragma once

namespace mavlink {
namespace fly_stereo {
namespace msg {

/**
 * @brief RESET_COUNTERS message
 *
 * Test all field types
 */
struct RESET_COUNTERS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 1;
    static constexpr size_t LENGTH = 8;
    static constexpr size_t MIN_LENGTH = 8;
    static constexpr uint8_t CRC_EXTRA = 68;
    static constexpr auto NAME = "RESET_COUNTERS";


    uint64_t timestamp_us; /*< [us] timestamp since linux epoch */


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

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << timestamp_us;                  // offset: 0
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> timestamp_us;                  // offset: 0
    }
};

} // namespace msg
} // namespace fly_stereo
} // namespace mavlink
