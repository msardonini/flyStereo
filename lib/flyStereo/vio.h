#pragma once

// Package Includes
#include "flyStereo/interface.h"
#include "flyStereo/stereo_calibration.h"
#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"
#include "opencv2/surface_matching/icp.hpp"
#include "yaml-cpp/yaml.h"

struct vio_t {
  cv::Vec3d position;
  cv::Vec3d velocity;
  cv::Vec4d quat;
  cv::Affine3d pose_cam0_;
};

template <typename IpBackend>
class Vio {
 public:
  // Constructor with config params
  Vio(const StereoCalibration &stereo_calibration, const cv::Matx33d &R_imu_cam0,
      const cv::Vec3d &vio_calibration_rvec = cv::Vec3d(0., 0., 0.),
      const cv::Vec3d &vio_calibration_tvec = cv::Vec3d(0., 0., 0.));
  ~Vio();
  std::tuple<cv::Affine3d, std::vector<cv::Point3f>> ProcessPoints(const TrackedImagePoints<IpBackend> &pts);

 private:
  inline unsigned int Modulo(int value, unsigned m);
  // int SaveInliers(std::vector<int> inliers, std::vector<int> pt_ids, opengv::points_t pts);

  std::tuple<cv::Affine3d, std::vector<cv::Point3f>> CalculatePoseUpdate(const TrackedImagePoints<IpBackend> &pts);

  int ProcessImu(const std::vector<mavlink_imu_t> &imu_pts);
  // int ProcessVio(const cv::Affine3d &pose_body, uint64_t image_timestamp, Eigen::Matrix<double, 6, 1> &output);

  // Stereo camera calibration parameters
  StereoCalibration stereo_cal_;
  cv::Matx33d R_imu_cam0_;

  cv::Matx34f P0_;
  cv::Matx34f P1_;

  // The estimated pose after VIO is run in the cameara frame
  cv::Vec3d vio_calibration_rvec_;
  cv::Vec3d vio_calibration_tvec_;
  cv::Affine3d pose_cam0_;
  bool first_iteration_;

  // Kalman Filter object to fuse imu and visual odom measurmenets
  // KalmanFilter kf_;
  uint64_t last_timestamp_ = 0;

  // cv::ppf_match_3d::ICP icp_;

  std::unordered_map<unsigned int, cv::Point3f> global_cloud_;
};

// #include "flyStereo/vio.tpp"
