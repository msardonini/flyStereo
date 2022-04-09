// MESSAGE VIO support class

#pragma once

namespace mavlink {
namespace fly_stereo {
namespace msg {

/**
 * @brief VIO message
 *
 * Commands the Perception unit
 */
struct VIO : mavlink::Message {
  static constexpr msgid_t MSG_ID = 3;
  static constexpr size_t LENGTH = 48;
  static constexpr size_t MIN_LENGTH = 48;
  static constexpr uint8_t CRC_EXTRA = 6;
  static constexpr auto NAME = "VIO";

  uint64_t timestamp_us;         /*< [us] timestamp since linux epoch */
  std::array<float, 3> position; /*< [bool] 3D position vio output in XYZ in meters */
  std::array<float, 3> velocity; /*< [bool] 3D velocity vio output in XYZ in meters / sec */
  std::array<float, 4> quat;     /*< [bool] Quaternion representation of rotation about XYZ axis */

  inline std::string get_name(void) const override { return NAME; }

  inline Info get_message_info(void) const override { return {MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA}; }

  inline std::string to_yaml(void) const override {
    std::stringstream ss;

    ss << NAME << ":" << std::endl;
    ss << "  timestamp_us: " << timestamp_us << std::endl;
    ss << "  position: [" << to_string(position) << "]" << std::endl;
    ss << "  velocity: [" << to_string(velocity) << "]" << std::endl;
    ss << "  quat: [" << to_string(quat) << "]" << std::endl;

    return ss.str();
  }

  inline void serialize(mavlink::MsgMap &map) const override {
    map.reset(MSG_ID, LENGTH);

    map << timestamp_us;  // offset: 0
    map << position;      // offset: 8
    map << velocity;      // offset: 20
    map << quat;          // offset: 32
  }

  inline void deserialize(mavlink::MsgMap &map) override {
    map >> timestamp_us;  // offset: 0
    map >> position;      // offset: 8
    map >> velocity;      // offset: 20
    map >> quat;          // offset: 32
  }
};

}  // namespace msg
}  // namespace fly_stereo
}  // namespace mavlink
