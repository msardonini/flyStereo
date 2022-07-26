#pragma once

#include <vector>

#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "opencv2/core.hpp"

struct LogParams {
  uint64_t timestamp_frame;
  cv::Mat_<uint8_t> frame0;
  cv::Mat_<uint8_t> frame1;
  std::vector<mavlink_imu_t> imu_msgs;
};
