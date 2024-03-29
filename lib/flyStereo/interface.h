#pragma once

#include <array>
#include <vector>

#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "opencv2/core.hpp"

// struct ImagePoint {
//   // Default Constructor
//   ImagePoint(){};
//   // Constructor with values
//   ImagePoint(unsigned int _id, std::array<double, 2> _cam0_t0, std::array<double, 2> _cam0_t1,
//              std::array<double, 2> _cam1_t0, std::array<double, 2> _cam1_t1)
//       : id(_id), cam0_t0(_cam0_t0), cam0_t1(_cam0_t1), cam1_t0(_cam1_t0), cam1_t1(_cam1_t1) {}

//   unsigned int id;
//   std::array<double, 2> cam0_t0;
//   std::array<double, 2> cam0_t1;
//   std::array<double, 2> cam1_t0;
//   std::array<double, 2> cam1_t1;
// };

// struct ImagePoints {
//   // Default Constructor
//   ImagePoints() = default;

//   // Constructor with values
//   ImagePoints(uint64_t _timestamp_us, std::vector<ImagePoint> _pts) : timestamp_us(_timestamp_us), pts(_pts) {}

//   // Copy Constructor
//   ImagePoints(const ImagePoints &src) : timestamp_us(src.timestamp_us), pts(src.pts) {}

//   // Method to extract the timestamp in seconds as a double
//   double TimestampToSec() const { return static_cast<double>(timestamp_us) / 1.0E6; }

//   // Stuct values
//   uint64_t timestamp_us;
//   std::vector<ImagePoint> pts;

//   // Associated IMU measurments to this set of features
//   std::vector<mavlink_imu_t> imu_pts;
//   Eigen::Matrix3d R_t0_t1_cam0;
// };

/**
 * @brief Container to hold the outputs of the stereo image processing pipeline.
 *
 */
template <typename IpBackend>
struct TrackedImagePoints {
  // Allow Default Constructor
  TrackedImagePoints() = default;

  /**
   * @brief Construct a new Tracked Image Points object using forwarding references.
   *
   * @param timestamp_us The timestamp in microseconds
   * @param ids  The ids of the points
   * @param cam0_t0 The points from cam0 in the previous timestep
   * @param cam0_t1 The points from cam0 in the current timestep
   * @param cam1_t0 The points from cam1 in the previous timestep
   * @param cam1_t1 The points from cam1 in the current timestep
   * @param imu_pts The imu measurements associated with the points
   * @param R_t0_t1_cam0 The rotation matrix between the timestamps as measured by the IMU
   */
  TrackedImagePoints(uint64_t timestamp_us, std::vector<unsigned int> &&ids, IpBackend::array_type &&cam0_t0,
                     IpBackend::array_type &&cam0_t1, IpBackend::array_type &&cam1_t0, IpBackend::array_type &&cam1_t1,
                     std::vector<mavlink_imu_t> &&imu_pts, cv::Matx33f &&R_t0_t1_cam0)
      : timestamp_us(timestamp_us),
        ids(ids),
        cam0_t0(cam0_t0),
        cam0_t1(cam0_t1),
        cam1_t0(cam1_t0),
        cam1_t1(cam1_t1),
        imu_pts(imu_pts),
        R_t0_t1_cam0(R_t0_t1_cam0) {}

  /**
   * @brief Construct a new Tracked Image Points object.
   *
   * @param timestamp_us The timestamp in microseconds
   * @param ids  The ids of the points
   * @param cam0_t0 The points from cam0 in the previous timestep
   * @param cam0_t1 The points from cam0 in the current timestep
   * @param cam1_t0 The points from cam1 in the previous timestep
   * @param cam1_t1 The points from cam1 in the current timestep
   * @param imu_pts The imu measurements associated with the points
   * @param R_t0_t1_cam0 The rotation matrix between the timestamps as measured by the IMU
   */
  TrackedImagePoints(uint64_t timestamp_us, const std::vector<unsigned int> &ids, const IpBackend::array_type &cam0_t0,
                     const IpBackend::array_type &cam0_t1, const IpBackend::array_type &cam1_t0,
                     const IpBackend::array_type &cam1_t1, const std::vector<mavlink_imu_t> &imu_pts,
                     const cv::Matx33f &R_t0_t1_cam0)
      : timestamp_us(timestamp_us),
        ids(ids),
        cam0_t0(cam0_t0),
        cam0_t1(cam0_t1),
        cam1_t0(cam1_t0),
        cam1_t1(cam1_t1),
        imu_pts(imu_pts),
        R_t0_t1_cam0(R_t0_t1_cam0) {}

  /**
   * @brief Member function to extract the timestamp in seconds as a double
   *
   * @return double
   */
  double TimestampToSec() const { return static_cast<double>(timestamp_us) / 1.0E6; }

  uint64_t timestamp_us;               //< The timestamp of the image in microseconds
  std::vector<unsigned int> ids;       //< The IDs of the tracked features
  IpBackend::array_type cam0_t0;       //< The points from cam0 in the previous frame
  IpBackend::array_type cam0_t1;       //< The points from cam0 in the current frame
  IpBackend::array_type cam1_t0;       //< The points from cam1 in the previous frame
  IpBackend::array_type cam1_t1;       //< The points from cam1 in the current frame
  std::vector<mavlink_imu_t> imu_pts;  //< The imu measurements associated with the tracked features
  cv::Matx33f R_t0_t1_cam0;            //< The rotation matrix between the timestamps as measured by the IMU
};
