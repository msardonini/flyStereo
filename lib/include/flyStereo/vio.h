#pragma once

// Package Includes
#include "Eigen/Dense"
#include "opencv2/core.hpp"
#include "yaml-cpp/yaml.h"
// #include "liblas/liblas.hpp"
#include "flyStereo/interface.h"
#include "flyStereo/kalman_filter.h"
#include "flyStereo/visualization/visualization.h"
#include "opengv/types.hpp"

struct vio_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond quat;
};

class Vio {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor with config params
  Vio(const YAML::Node &input_params, const YAML::Node &stereo_calibration);
  // Destructor
  ~Vio();

  int ProcessPoints(const ImagePoints &pts, vio_t &vio);

 private:
  inline unsigned int Modulo(int value, unsigned m);
  int SaveInliers(std::vector<int> inliers, std::vector<int> pt_ids, opengv::points_t pts);

  int CalculatePoseUpdate(const ImagePoints &pts, const Eigen::Matrix3d &imu_rotation, Eigen::Matrix4d &pose_update,
                          std::vector<cv::Point3d> *inlier_pts = nullptr);

  int ProcessImu(const std::vector<mavlink_imu_t> &imu_pts);
  int ProcessVio(const Eigen::Matrix4d &pose_update, uint64_t image_timestamp, Eigen::Matrix<double, 6, 1> &output);
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

  // All points are placed in bins, sections of the image. The key to this map is the row-major
  // index of the bin. The value is a vector of image points, which holds points from both cameras
  // in the current frame and the previos one
  std::map<int, std::vector<ImagePoint> > grid;

  // Stereo camera calibration parameters
  cv::Matx33d K_cam0_;
  cv::Matx33d K_cam1_;
  std::vector<double> D_cam0_;
  std::vector<double> D_cam1_;
  cv::Matx33d R_cam0_cam1_;
  // cv::Matx33d R_imu_cam0_;
  Eigen::Matrix3d R_imu_cam0_eigen_;
  cv::Vec3d T_cam0_cam1_;
  cv::Matx34d P0_;
  cv::Matx34d P1_;

  // The estimated pose after VIO is run in the cameara frame
  Eigen::MatrixXd pose_cam0_;
  bool first_iteration_;

  // Output file stream for debugging output
  std::unique_ptr<std::ofstream> ofs_;
  std::unique_ptr<std::ofstream> trajecotry_file_;
  // std::unique_ptr<liblas::Writer> writer_;
  // std::unique_ptr<liblas::Header> header_;

  std::vector<std::map<unsigned int, opengv::point_t> > point_history_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > pose_history_;
  unsigned int point_history_index_;

  // Kalman Filter object to fuse imu and visual odom measurmenets
  KalmanFilter kf_;
  uint64_t last_timestamp_ = 0;

  Visualization visualization_;
};