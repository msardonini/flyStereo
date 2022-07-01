#pragma once

#include <vector>

#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "flyStereo/types/umat.h"

template <UMatDerivative ImageT>
struct LogParams {
  uint64_t timestamp_frame;
  ImageT frame0;
  ImageT frame1;
  std::vector<mavlink_imu_t> imu_msgs;
};
