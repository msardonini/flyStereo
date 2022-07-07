#pragma once

#include "opencv2/core.hpp"
#include "opencv2/core/matx.hpp"

namespace utility {

// Calculates rotation matrix given euler angles.
template <typename T>
cv::Matx<typename T::value_type, 3, 3> eulerAnglesToRotationMatrix(const T &theta) {
  // Calculate rotation about x axis
  cv::Matx<typename T::value_type, 3, 3> R_x(1, 0, 0, 0, cos(theta[0]), -sin(theta[0]), 0, sin(theta[0]),
                                             cos(theta[0]));

  // Calculate rotation about y axis
  cv::Matx<typename T::value_type, 3, 3> R_y(cos(theta[1]), 0, sin(theta[1]), 0, 1, 0, -sin(theta[1]), 0,
                                             cos(theta[1]));

  // Calculate rotation about z axis
  cv::Matx<typename T::value_type, 3, 3> R_z(cos(theta[2]), -sin(theta[2]), 0, sin(theta[2]), cos(theta[2]), 0, 0, 0,
                                             1);

  // Combined rotation matrix
  return R_z * R_y * R_x;
}

}  // namespace utility
