
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
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include "spdlog/spdlog.h"

constexpr unsigned int history_size = 10;
constexpr unsigned int min_num_matches = 25;
constexpr float min_depth_thresh = 0.0f;
constexpr float max_depth_thresh = 5.0f;

// Constructor with config params
template <typename IpBackend>
Vio<IpBackend>::Vio(const StereoCalibration &stereo_calibration, const cv::Matx33d &R_imu_cam0,
                    const cv::Vec3d &vio_calibration_rvec, const cv::Vec3d &vio_calibration_tvec)
    : stereo_cal_(stereo_calibration),
      vio_calibration_rvec_(vio_calibration_rvec),
      vio_calibration_tvec_(vio_calibration_tvec) {
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
std::tuple<cv::Affine3d, std::vector<cv::Point3f>> Vio<IpBackend>::ProcessPoints(
    const TrackedImagePoints<IpBackend> &pts) {
  // If this is our first update, set our initial pose by the rotation of the imu

  // Calculate the updated pose
  // if (first_iteration_) {
  //   if (pts.imu_pts.size() == 0) {
  //     return {};
  //   }
  //   first_iteration_ = false;

  //   Eigen::Matrix3d initial_rotation = Eigen::AngleAxisd(-pts.imu_pts[0].roll, Eigen::Vector3d::UnitX()) *
  //                                      Eigen::AngleAxisd(-pts.imu_pts[0].pitch, Eigen::Vector3d::UnitY()) *
  //                                      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  //   cv::Matx33d initial_rotation_cv;
  //   cv::eigen2cv(initial_rotation, initial_rotation_cv);

  //   pose_cam0_.rotation(R_imu_cam0_.t() * initial_rotation_cv.t() * R_imu_cam0_);

  //   spdlog::info("first imu point: {}, {}, {}", pts.imu_pts[0].roll, pts.imu_pts[0].pitch, pts.imu_pts[0].yaw);
  // }

  auto [pose_update_cam0, inliers] = CalculatePoseUpdate(pts);

  // Update our Pose

  // std::cout << " pose update " << pose_update_cam0.rvec() << std::endl;

  pose_cam0_ = pose_cam0_.concatenate(pose_update_cam0);

  return std::make_tuple(pose_cam0_, inliers);

  // auto pose_body = pose_cam0_.rotate(R_imu_cam0_ * pose_cam0_.rotation() * R_imu_cam0_.t());

  // Eigen::Matrix<double, 6, 1> kf_state;
  // ProcessVio(pose_body, pts.timestamp_us, kf_state);

  // // Copy the results to the output of this function
  // vio.position << kf_state(0), kf_state(2), kf_state(4);
  // vio.velocity << kf_state(1), kf_state(3), kf_state(5);
  // vio.pose_cam0_ = pose_cam0_;

  // Eigen::Matrix<double, 3, 3> pose_body_rot;
  // cv::cv2eigen(pose_body.rotation(), pose_body_rot);
  // cv::cv2eigen(pose_body.translation(), vio.position);

  // vio.quat = Eigen::Quaterniond(pose_body_rot);

  // return 0;
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
std::tuple<cv::Affine3d, std::vector<cv::Point3f>> Vio<IpBackend>::CalculatePoseUpdate(
    const TrackedImagePoints<IpBackend> &pts) {
  if (pts.ids.size() < 6ul) {
    spdlog::warn("Not enough points for algorithm!");
    return {};
  }
  // Containers for the undistorted points
  // std::vector<cv::Point2f> pts_cam0_t0_ud;
  std::vector<cv::Point2f> pts_cam0_t1_ud;
  // std::vector<cv::Point2f> pts_cam1_t0_ud;
  std::vector<cv::Point2f> pts_cam1_t1_ud;

  // Undistort the points using the intrinsic calibration
  // cv::undistortPoints(pts.cam0_t0.frame(), pts_cam0_t0_ud, stereo_cal_.K_cam0, stereo_cal_.D_cam0);
  cv::undistortPoints(pts.cam0_t1.frame(), pts_cam0_t1_ud, stereo_cal_.K_cam0, stereo_cal_.D_cam0);
  // cv::undistortPoints(pts.cam1_t0.frame(), pts_cam1_t0_ud, stereo_cal_.K_cam1, stereo_cal_.D_cam1);
  cv::undistortPoints(pts.cam1_t1.frame(), pts_cam1_t1_ud, stereo_cal_.K_cam1, stereo_cal_.D_cam1);

  cv::Mat triangulation_output_pts_homo_t1;
  // cv::Affine3f extrinsic_cal(stereo_cal_.R_cam0_cam1, stereo_cal_.T_cam0_cam1);
  cv::triangulatePoints(cv::Matx34f::eye(), P1_, pts_cam0_t1_ud, pts_cam1_t1_ud, triangulation_output_pts_homo_t1);
  // cv::Mat triangulation_output_pts_homo_t0;
  // cv::triangulatePoints(cv::Matx34f::eye(), P1_, pts_cam0_t0_ud, pts_cam1_t0_ud, triangulation_output_pts_homo_t0);

  // Convert points from homogeneous to 3D coords
  std::vector<cv::Point3f> triangulation_output_pts_t1;
  cv::convertPointsFromHomogeneous(triangulation_output_pts_homo_t1.t(), triangulation_output_pts_t1);
  // triangulation_output_pts_t1 = triangulation_output_pts_t1.reshape(1);

  auto thresh_lambda = [](auto &p) { return p.z < min_depth_thresh || p.z > max_depth_thresh; };

  std::vector<cv::Point2f> pts_cam0_t0;
  pts_cam0_t0.reserve(triangulation_output_pts_t1.size());
  std::vector<int> pts_idx;
  pts_idx.reserve(triangulation_output_pts_t1.size());
  for (auto i = 0ul; i < triangulation_output_pts_t1.size(); i++) {
    if (!thresh_lambda(triangulation_output_pts_t1[i])) {
      pts_cam0_t0.emplace_back(cv::Point2f(pts.cam0_t0.frame()(i)));
      pts_idx.emplace_back(pts.ids[i]);
    }
  }

  std::erase_if(triangulation_output_pts_t1, thresh_lambda);

  if (triangulation_output_pts_t1.size() <= 6ul || pts_cam0_t0.size() != triangulation_output_pts_t1.size()) {
    spdlog::warn("Not enough points for algorithm after depth filtering!");
    return {};
  }

  // cv::Mat triangulation_output_pts_t0;
  // cv::convertPointsFromHomogeneous(triangulation_output_pts_homo_t0.t(), triangulation_output_pts_t0);
  // triangulation_output_pts_t0 = triangulation_output_pts_t0.reshape(1);

  std::vector<int> inliers;
  cv::Vec3d rvec, tvec;

  cv::solvePnPRansac(triangulation_output_pts_t1, pts_cam0_t0, stereo_cal_.K_cam0, stereo_cal_.D_cam0, rvec, tvec,
                     false, 500, 2, 0.99, inliers, cv::SOLVEPNP_AP3P);

  // std::vector<cv::Point3f> tri_pts_t1_refine;
  // tri_pts_t1_refine.reserve(inliers.size());
  // std::vector<cv::Point2f> pts_cam0_t0_refine;
  // pts_cam0_t0_refine.reserve(inliers.size());
  // std::vector<int> idx_refine;
  // idx_refine.reserve(inliers.size());

  // std::for_each(inliers.begin(), inliers.end(), [&](auto i) {
  //   tri_pts_t1_refine.emplace_back(triangulation_output_pts_t1[i]);
  //   pts_cam0_t0_refine.emplace_back(pts_cam0_t0[i]);
  //   idx_refine.emplace_back(pts_idx[i]);
  // });

  // cv::solvePnPRefineLM(tri_pts_t1_refine, pts_cam0_t0_refine, stereo_cal_.K_cam0, stereo_cal_.D_cam0, rvec, tvec);

  // Apply the calibration and consolidate pose into an affine transform
  // cv::Affine3d pose_update(rvec, tvec);
  cv::Affine3d pose_update(rvec - vio_calibration_rvec_, tvec - vio_calibration_tvec_);

  // Get the points from the global cloud with an index in triangulation_output
  std::vector<cv::Point3f> inlier_pts_3d_global;
  inlier_pts_3d_global.reserve(inliers.size());
  std::vector<cv::Point3f> inlier_pts_3d_iteration;
  inlier_pts_3d_iteration.reserve(inliers.size());
  std::for_each(inliers.begin(), inliers.end(), [&](auto i) {
    if (global_cloud_.count(pts_idx[i])) {
      inlier_pts_3d_global.emplace_back(global_cloud_.find(pts_idx[i])->second);
      inlier_pts_3d_iteration.emplace_back(triangulation_output_pts_t1[i]);
    }
  });

  std::vector<cv::Point3f> inlier_pts_3d;
  inlier_pts_3d.reserve(inliers.size());

  // if (inlier_pts_3d_global.size() >= 6) {
  //   cv::Mat affine_correction_mat, inliers_correction;
  //   cv::estimateAffine3D(inlier_pts_3d_iteration, inlier_pts_3d_global, affine_correction_mat, inliers_correction,
  //                        0.005, 0.999);

  //   cv::Affine3d affine_correction(affine_correction_mat);

  //   // // print out the values of both point sets
  //   // cv::Mat mean_diff, mean_diff2;
  //   // cv::absdiff(inlier_pts_3d_global, inlier_pts_3d_iteration, mean_diff);
  //   // cv::reduce(mean_diff, mean_diff2, 1, cv::REDUCE_AVG);

  //   // std::cout << "abs diff " << mean_diff << std::endl;

  //   cv::Point3d mean(0, 0, 0);
  //   for (auto i = 0; i < inlier_pts_3d_global.size(); i++) {
  //     // std::cout << "inlier_pts_3d_global: " << inlier_pts_3d_global[i];
  //     // std::cout << " inlier_pts_3d_iteration: " << inlier_pts_3d_iteration[i] << std::endl;

  //     cv::Point3d tmp = inlier_pts_3d_global[i] - inlier_pts_3d_iteration[i];
  //     tmp.x = (tmp.x > 0) ? tmp.x : -tmp.x;
  //     tmp.y = (tmp.y > 0) ? tmp.y : -tmp.y;
  //     tmp.z = (tmp.z > 0) ? tmp.z : -tmp.z;

  //     mean += tmp;
  //   }
  //   mean /= (double)inlier_pts_3d_global.size();

  //   std::cout << "mean diff " << mean << std::endl;

  //   std::cout << affine_correction_mat << std::endl;

  //   pose_update = pose_update.concatenate(affine_correction);

  //   auto counter = 0;
  //   std::for_each(inliers_correction.begin<int>(), inliers_correction.end<int>(), [&](auto status) {
  //     if (status == 1) {
  //       inlier_pts_3d.emplace_back(inlier_pts_3d_global[counter++]);
  //     }
  //   });
  // } else {
  std::for_each(inliers.begin(), inliers.end(),
                [&](auto i) { inlier_pts_3d.push_back(triangulation_output_pts_t1[i]); });
  // }

  std::for_each(inliers.begin(), inliers.end(),
                [&](auto i) { global_cloud_.insert(std::make_pair(pts_idx[i], triangulation_output_pts_t1[i])); });

  // check no update is over 1m away from the current pose or it is likely an error
  if (cv::norm(pose_update.translation()) > 1.0) {
    spdlog::warn("Pose update is too far away from current pose!");
    return {};
  }

  return std::make_tuple(pose_update, inlier_pts_3d);

  // cv::Mat t0_normals, t1_normals;

  // cv::Vec3f viewpoint(0, 0, 0);
  // cv::ppf_match_3d::computeNormalsPC3d(triangulation_output_pts_t0, t0_normals, 10, false, viewpoint);
  // cv::ppf_match_3d::computeNormalsPC3d(triangulation_output_pts_t1, t1_normals, 10, false, viewpoint);

  // cv::Matx44d output_pose;
  // double residual = 0;

  // icp_.registerModelToScene(t0_normals, t1_normals, residual, output_pose);

  // cv::Affine3d pose_update(output_pose);
  // std::cout << pose_update.rvec() << std::endl;

  // std::vector<cv::Point3f> output_pts;
  // output_pts.reserve(triangulation_output_pts_t1.rows);
  // for (auto i = 0; i < triangulation_output_pts_t1.rows; i++) {
  //   cv::Point3f pt(triangulation_output_pts_t1.at<float>(i, 0), triangulation_output_pts_t1.at<float>(i, 1),
  //                  triangulation_output_pts_t1.at<float>(i, 2));
  //   output_pts.push_back(pt);
  // }
  // return std::make_tuple(pose_update, output_pts);
}

#include "flyStereo/image_processing/cv_backend.h"
template class Vio<CvBackend>;
#ifdef WITH_VPI
#include "flyStereo/image_processing/vpi_backend.h"
template class Vio<VpiBackend>;
#endif
