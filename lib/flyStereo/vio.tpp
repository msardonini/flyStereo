
#include "flyStereo/vio.h"

// System includes
#include <fstream>
#include <iostream>

// Package includes
#include "Eigen/Geometry"
#include "flyStereo/interface.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/imgproc.hpp"
#include "spdlog/spdlog.h"

constexpr unsigned int history_size = 10;
constexpr unsigned int min_num_matches = 25;
// Constructor with config params
template <typename IpBackend>
Vio<IpBackend>::Vio(const StereoCalibration &stereo_calibration, const cv::Matx33d &R_imu_cam0,
                    const cv::Vec3d &vio_calibration)
    : stereo_cal_(stereo_calibration) {
  cv::cv2eigen(vio_calibration, vio_calibration_);
  cv::cv2eigen(R_imu_cam0, R_imu_cam0_eigen_);
  R_imu_cam0_ = R_imu_cam0;

  // Create the projection matrices such that the output points will be in the coordinate system
  // of cam0
  P0_ = cv::Matx34f::eye();
  P1_ = cv::Matx34f::eye();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P1_(i, j) = stereo_cal_.R_cam0_cam1(i, j);
    }
    P1_(i, 3) = stereo_cal_.T_cam0_cam1[i];
  }

  // Set the initial pose to identity
  pose_cam0_ = cv::Affine3d::Identity();
  first_iteration_ = true;

  point_history_index_ = 0;
  pose_history_.resize(history_size);
}

template <typename IpBackend>
Vio<IpBackend>::~Vio() {}

template <typename IpBackend>
int Vio<IpBackend>::ProcessPoints(const TrackedImagePoints<IpBackend> &pts, vio_t &vio) {
  // If this is our first update, set our initial pose by the rotation of the imu

  // Calculate the updated pose
  if (first_iteration_) {
    if (pts.imu_pts.size() == 0) {
      return 0;
    }
    first_iteration_ = false;

    Eigen::Matrix3d initial_rotation = Eigen::AngleAxisd(-pts.imu_pts[0].roll, Eigen::Vector3d::UnitX()) *
                                       Eigen::AngleAxisd(-pts.imu_pts[0].pitch, Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    cv::Matx33d initial_rotation_cv;
    cv::eigen2cv(initial_rotation, initial_rotation_cv);

    pose_cam0_.rotation(R_imu_cam0_.t() * initial_rotation_cv.t() * R_imu_cam0_);

    spdlog::info("first imu point: {}, {}, {}", pts.imu_pts[0].roll, pts.imu_pts[0].pitch, pts.imu_pts[0].yaw);
  }

  cv::Affine3d pose_update_cam0;
  std::vector<cv::Point3d> inliers;
  if (CalculatePoseUpdate(pts, pose_update_cam0, &inliers)) {
    spdlog::warn("Error in CalculatePoseUpdate");
    return -1;
  }

  // Update our Pose
  pose_cam0_ = pose_cam0_ * pose_update_cam0;
  pose_cam0_.concatenate(pose_update_cam0);

  auto pose_body = pose_cam0_.rotate(R_imu_cam0_ * pose_cam0_.rotation() * R_imu_cam0_.t());

  Eigen::Matrix<double, 6, 1> kf_state;
  ProcessVio(pose_body, pts.timestamp_us, kf_state);

  // Copy the results to the output of this function
  vio.position << kf_state(0), kf_state(2), kf_state(4);
  vio.velocity << kf_state(1), kf_state(3), kf_state(5);
  vio.pose_cam0_ = pose_cam0_;

  Eigen::Matrix<double, 3, 3> pose_body_rot;
  cv::cv2eigen(pose_body.rotation(), pose_body_rot);
  cv::cv2eigen(pose_body.translation(), vio.position);

  vio.quat = Eigen::Quaterniond(pose_body_rot);

  return 0;
}

template <typename IpBackend>
int Vio<IpBackend>::ProcessVio(const cv::Affine3d &pose_body, uint64_t image_timestamp,
                               Eigen::Matrix<double, 6, 1> &output_state) {
  Eigen::Matrix<double, 3, 1> z;
  cv::cv2eigen(pose_body.translation(), z);

  if (last_timestamp_ == 0) {
    kf_.Predict();
  } else {
    kf_.Predict(static_cast<double>(image_timestamp - last_timestamp_) / 1.0E6);
  }
  last_timestamp_ = image_timestamp;
  kf_.Measure(z);

  output_state = kf_.GetState();
  return 0;
}

template <typename IpBackend>
inline unsigned int Vio<IpBackend>::Modulo(int value, unsigned m) {
  int mod = value % (int)m;
  if (value < 0) {
    mod += m;
  }
  return mod;
}

inline double get_inlier_pct(const std::vector<int> &inliers) {
  return std::accumulate(inliers.begin(), inliers.end(), 0.,
                         [](auto a, auto new_val) { return a + ((new_val) ? 1. : 0.); }) /
         inliers.size();
}

template <typename IpBackend>
int Vio<IpBackend>::CalculatePoseUpdate(const TrackedImagePoints<IpBackend> &pts, cv::Affine3d &pose_update,
                                        std::vector<cv::Point3d> *inlier_pts) {
  if (pts.ids.size() < 6ul) {
    spdlog::warn("Not enough points for algorithm!");
    return -1;
  }

  // Containers for the undistorted points
  std::vector<cv::Point2f> pts_cam0_t0_ud;
  std::vector<cv::Point2f> pts_cam0_t1_ud;
  std::vector<cv::Point2f> pts_cam1_t0_ud;
  std::vector<cv::Point2f> pts_cam1_t1_ud;

  // Undistort the points using the intrinsic calibration
  cv::undistortPoints(pts.cam0_t0.frame(), pts_cam0_t0_ud, stereo_cal_.K_cam0, stereo_cal_.D_cam0);
  cv::undistortPoints(pts.cam0_t1.frame(), pts_cam0_t1_ud, stereo_cal_.K_cam0, stereo_cal_.D_cam0);
  cv::undistortPoints(pts.cam1_t0.frame(), pts_cam1_t0_ud, stereo_cal_.K_cam1, stereo_cal_.D_cam1);
  cv::undistortPoints(pts.cam1_t1.frame(), pts_cam1_t1_ud, stereo_cal_.K_cam1, stereo_cal_.D_cam1);

  cv::Mat triangulation_output_pts_homo;
  cv::Affine3f extrinsic_cal(stereo_cal_.R_cam0_cam1, stereo_cal_.T_cam0_cam1);
  cv::triangulatePoints(cv::Matx34f::eye(), P1_, pts_cam0_t0_ud, pts_cam1_t0_ud, triangulation_output_pts_homo);

  // Convert points from homogeneous to 3D coords
  std::vector<cv::Vec3f> triangulation_output_pts;
  cv::convertPointsFromHomogeneous(triangulation_output_pts_homo.t(), triangulation_output_pts);

  std::vector<int> inliers;
  cv::Vec3d rvec, tvec;

  cv::solvePnPRansac(triangulation_output_pts, pts.cam0_t1.frame(), stereo_cal_.K_cam0, stereo_cal_.D_cam0, rvec, tvec,
                     false, 500, 0.05, 0.99, inliers, cv::SOLVEPNP_P3P);

  // std::cout << " Inlier percentage openCV " << static_cast<float>(inliers.size()) / triangulation_output_pts.size()
  //           << std::endl;

  pose_update = cv::Affine3d(rvec, tvec).inv();

  pose_cam0_ = pose_cam0_.concatenate(pose_update);

  std::cout << " output pose \n" << static_cast<cv::Matx44f>(pose_cam0_.matrix) << std::endl;

  // std::for_each(triangulation_output_pts.begin(), triangulation_output_pts.end(),
  //               [](cv::Vec3f &pt) { std::cout << pt << std::endl; });

  return 0;
}
