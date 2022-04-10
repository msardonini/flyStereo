#pragma once

#include <array>
#include <vector>

#include "Eigen/Dense"
#include "fly_stereo/sensor_io/mavlink/fly_stereo/mavlink.h"

struct ImagePoint {
  // Default Constructor
  ImagePoint(){};
  // Constructor with values
  ImagePoint(unsigned int _id, std::array<double, 2> _cam0_t0, std::array<double, 2> _cam0_t1,
             std::array<double, 2> _cam1_t0, std::array<double, 2> _cam1_t1)
      : id(_id), cam0_t0(_cam0_t0), cam0_t1(_cam0_t1), cam1_t0(_cam1_t0), cam1_t1(_cam1_t1) {}

  unsigned int id;
  std::array<double, 2> cam0_t0;
  std::array<double, 2> cam0_t1;
  std::array<double, 2> cam1_t0;
  std::array<double, 2> cam1_t1;
};

struct ImagePoints {
  // Default Constructor
  ImagePoints() = default;

  // Constructor with values
  ImagePoints(uint64_t _timestamp_us, std::vector<ImagePoint> _pts) : timestamp_us(_timestamp_us), pts(_pts) {}

  // Copy Constructor
  ImagePoints(const ImagePoints &src) : timestamp_us(src.timestamp_us), pts(src.pts) {}

  // Method to extract the timestamp in seconds as a double
  double TimestampToSec() const { return static_cast<double>(timestamp_us) / 1.0E6; }

  // Stuct values
  uint64_t timestamp_us;
  std::vector<ImagePoint> pts;

  // Associated IMU measurments to this set of features
  std::vector<mavlink_imu_t> imu_pts;
  Eigen::Matrix3d R_t0_t1_cam0;
};

struct VisualOdom {
  double TimestampToSec() const { return static_cast<double>(timestamp_us) / 1.0E6; }
  uint64_t timestamp_us;
  Eigen::Vector3d pos;
  Eigen::Vector4d quat;
  Eigen::Matrix<double, 6, 6> cov;
};
