#ifndef LIB_INCLUDE_FLY_STEREO_IMAGE_POINTS_H_
#define LIB_INCLUDE_FLY_STEREO_IMAGE_POINTS_H_

#include <array>
#include <vector>

#include "Eigen/Dense"

struct ImagePoint {
  // Default Constructor
  ImagePoint() {};
  // Constructor with values
  ImagePoint(unsigned int _id, std::array<double, 2> _cam0_pt, std::array<double, 2> _cam1_pt) :
    id(_id),
    cam0_pt(_cam0_pt),
    cam1_pt(_cam1_pt) {}

  unsigned int id;  
  std::array<double, 2> cam0_pt;
  std::array<double, 2> cam1_pt;
};

struct ImagePoints {
  // Default Constructor
  ImagePoints() {};
  // Constructor with values
  ImagePoints(uint64_t _timestamp_us, std::vector<ImagePoint> _pts) :
  timestamp_us(_timestamp_us),
  pts(_pts) {}
  // Copy Constructor
  ImagePoints(const ImagePoints &src) :
    timestamp_us(src.timestamp_us),
    pts(src.pts) {}

  // Method to extract the timestamp in seconds as a double
  double TimestampToSec() const {
    return static_cast<double>(timestamp_us) / 1.0E6;
  }

  // Stuct values
  uint64_t timestamp_us;
  uint64_t timestamp_imu_us;
  std::vector<ImagePoint> pts;
};

struct VisualOdom {
  double TimestampToSec() const {
    return static_cast<double>(timestamp_us) / 1.0E6;
  }
  uint64_t timestamp_us;
  Eigen::Vector3d pos;
  Eigen::Vector4d quat;
  Eigen::Matrix<double, 6, 6> cov;
};

#endif  // LIB_INCLUDE_FLY_STEREO_IMAGE_POINTS_H_
