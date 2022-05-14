#pragma once

// Package Includes
#include "Eigen/Dense"
#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"
#include "yaml-cpp/yaml.h"
// #include "liblas/liblas.hpp"
#include "flyStereo/interface.h"
#include "flyStereo/kalman_filter.h"
#include "flyStereo/stereo_calibration.h"

struct vio_t {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond quat;
};

class Vio {
 public:
  // Constructor with config params
  Vio(const StereoCalibration &stereo_calibration, const cv::Matx33d &R_imu_cam0,
      const cv::Vec3d &vio_calibration = cv::Vec3d(0., 0., 0.));
  ~Vio();

  int ProcessPoints(const TrackedImagePoints &pts, vio_t &vio);

 private:
  inline unsigned int Modulo(int value, unsigned m);
  // int SaveInliers(std::vector<int> inliers, std::vector<int> pt_ids, opengv::points_t pts);

  int CalculatePoseUpdate(const TrackedImagePoints &pts, const Eigen::Matrix3d &imu_rotation, cv::Affine3d &pose_update,
                          std::vector<cv::Point3d> *inlier_pts);

  int ProcessImu(const std::vector<mavlink_imu_t> &imu_pts);
  int ProcessVio(const cv::Affine3d &pose_body, uint64_t image_timestamp, Eigen::Matrix<double, 6, 1> &output);
  int Debug_SaveOutput(const Eigen::Matrix4d &pose_update, const Eigen::Matrix3d &R_imu);

  // The calibrated offsets for vio. These numbers will be subtracted off the result of the VIO
  // calculation to lower the drift
  Eigen::Matrix<double, 3, 1> vio_calibration_;

  bool first_iteration_save_ = true;

  // Object to hold all of the binned features
  unsigned int image_width_;
  unsigned int image_height_;
  unsigned int bins_width_;
  unsigned int bins_height_;
  unsigned int max_pts_in_bin_;

  // Stereo camera calibration parameters
  StereoCalibration stereo_cal_;
  Eigen::Matrix3d R_imu_cam0_eigen_;
  cv::Matx33d R_imu_cam0_;

  cv::Matx34d P0_;
  cv::Matx34d P1_;

  // The estimated pose after VIO is run in the cameara frame
  cv::Affine3d pose_cam0_;
  bool first_iteration_;

  // Output file stream for debugging output
  std::unique_ptr<std::ofstream> ofs_;
  std::unique_ptr<std::ofstream> trajecotry_file_;
  // std::unique_ptr<liblas::Writer> writer_;
  // std::unique_ptr<liblas::Header> header_;

  // std::vector<std::map<unsigned int, opengv::point_t> > point_history_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > pose_history_;
  unsigned int point_history_index_;

  // Kalman Filter object to fuse imu and visual odom measurmenets
  KalmanFilter kf_;
  uint64_t last_timestamp_ = 0;
};
