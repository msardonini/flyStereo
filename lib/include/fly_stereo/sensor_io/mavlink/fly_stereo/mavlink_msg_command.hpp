// MESSAGE COMMAND support class

#pragma once

namespace mavlink {
namespace fly_stereo {
namespace msg {

/**
 * @brief COMMAND message
 *
 * Commands the Perception unit
 */
struct COMMAND : mavlink::Message {
    static constexpr msgid_t MSG_ID = 2;
    static constexpr size_t LENGTH = 22;
    static constexpr size_t MIN_LENGTH = 22;
    static constexpr uint8_t CRC_EXTRA = 12;
    static constexpr auto NAME = "COMMAND";


    uint64_t timestamp_us; /*< [us] timestamp since linux epoch */
    uint8_t engage; /*< [bool] flag to turn on */
    uint8_t shutdown; /*< [bool] flag to turn off */
    uint32_t custom_cmd1; /*< [custom] custom command */
    uint32_t custom_cmd2; /*< [custom] custom command */
    uint32_t custom_cmd3; /*< [custom] custom command */


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
        ss << "  engage: " << +engage << std::endl;
        ss << "  shutdown: " << +shutdown << std::endl;
        ss << "  custom_cmd1: " << custom_cmd1 << std::endl;
        ss << "  custom_cmd2: " << custom_cmd2 << std::endl;
        ss << "  custom_cmd3: " << custom_cmd3 << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << timestamp_us;                  // offset: 0
        map << custom_cmd1;                   // offset: 8
        map << custom_cmd2;                   // offset: 12
        map << custom_cmd3;                   // offset: 16
        map << engage;                        // offset: 20
        map << shutdown;                      // offset: 21
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> timestamp_us;                  // offset: 0
        map >> custom_cmd1;                   // offset: 8
        map >> custom_cmd2;                   // offset: 12
        map >> custom_cmd3;                   // offset: 16
        map >> engage;                        // offset: 20
        map >> shutdown;                      // offset: 21
    }
};

} // namespace msg
} // namespace fly_stereo
} // namespace mavlink
