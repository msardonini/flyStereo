
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
// #include "opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp"
// #include "opengv/absolute_pose/methods.hpp"
// #include "opengv/point_cloud/PointCloudAdapter.hpp"
// #include "opengv/point_cloud/methods.hpp"
// #include "opengv/relative_pose/CentralRelativeAdapter.hpp"
// #include "opengv/sac/Ransac.hpp"
// #include "opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp"
// #include "opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp"
// #include "opengv/triangulation/methods.hpp"
// #include "spdlog/fmt/ostr.h"  // To print out Eigen objects
#include "spdlog/spdlog.h"

constexpr unsigned int history_size = 10;
constexpr unsigned int min_num_matches = 25;
// Constructor with config params
Vio::Vio(const StereoCalibration &stereo_calibration, const cv::Matx33d &R_imu_cam0, const cv::Vec3d &vio_calibration)
    : stereo_cal_(stereo_calibration) {
  // // If we have recording enabled, initialize the logging files
  // if (input_params["record_mode"] && input_params["record_mode"]["enable"].as<bool>() &&
  //     input_params["record_mode"]["outputs"]["trajectory"].as<bool>()) {
  //   std::string run_file = input_params["record_mode"]["log_dir"].as<std::string>();
  //   trajecotry_file_ = std::make_unique<std::ofstream>(run_file + "/trajectory.txt", std::ios::out);
  // }

  // YAML::Node vio_params = input_params["vio"];
  // Parse params for the VIO node
  // std::vector<double> vio_calibration_vec = vio_params["vio_calibration"].as<std::vector<double>>();
  // vio_calibration_ = Eigen::Matrix<double, 3, 1>(vio_calibration_vec.data());

  cv::cv2eigen(vio_calibration, vio_calibration_);

  cv::cv2eigen(R_imu_cam0, R_imu_cam0_eigen_);
  R_imu_cam0_ = R_imu_cam0;

  // Create the projection matrices such that the output points will be in the coordinate system
  // of cam0
  P0_ = cv::Matx34d::eye();
  P1_ = cv::Matx34d::eye();
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
  // point_history_ .resize(history_size);
  pose_history_.resize(history_size);

  // ofs_ = std::make_unique<std::ofstream> ("points.las", std::ios::out | std::ios::binary);
  // header_ = std::make_unique<liblas::Header>();
  // header_->SetDataFormatId(liblas::ePointFormat1); // Time only
  // header_->SetScale(0.0001, 0.0001, 0.0001);
  // writer_ = std::make_unique<liblas::Writer> (*ofs_.get(), *header_.get());
}

Vio::~Vio() {}

int Vio::ProcessPoints(const TrackedImagePoints &pts, vio_t &vio) {
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

    // Eigen::Vector3d imu_rot; imu_rot << pts.imu_pts[0].roll, pts.imu_pts[0].pitch, 0.0;
    // Eigen::Vector3d initial_rotation_vec = R_imu_cam0_eigen_ * imu_rot;

    // Eigen::Matrix3d initial_rotation =
    //   Eigen::AngleAxisd(initial_rotation_vec(0), Eigen::Vector3d::UnitX()) *
    //   Eigen::AngleAxisd(initial_rotation_vec(1),  Eigen::Vector3d::UnitY()) *
    //   Eigen::AngleAxisd(initial_rotation_vec(2), Eigen::Vector3d::UnitZ())
    //   .toRotationMatrix().transpose();

    // TODO figure out if we need to transpose the initial rotation
    // pose_cam0_.block<3, 3>(0, 0) = R_imu_cam0_eigen_.transpose() * initial_rotation.transpose() * R_imu_cam0_eigen_;

    pose_cam0_.rotation(R_imu_cam0_.t() * initial_rotation_cv.t() * R_imu_cam0_);

    spdlog::info("first imu point: {}, {}, {}", pts.imu_pts[0].roll, pts.imu_pts[0].pitch, pts.imu_pts[0].yaw);
    // spdlog::info("Mat1: {}", initial_rotation);
    // spdlog::info("Mat2: {}", R_imu_cam0_eigen_);
  }

  cv::Affine3d pose_update_cam0;
  std::vector<cv::Point3d> inliers;
  if (CalculatePoseUpdate(pts, pts.R_t0_t1_cam0.cast<double>(), pose_update_cam0, &inliers)) {
    spdlog::warn("Error in CalculatePoseUpdate");
    return -1;
  }

  // Prepare some data for logging only
  // Eigen::Matrix3d R_t0_t1_imu;
  // if (first_iteration_save_) {
  //   R_t0_t1_imu = R_imu_cam0_eigen_ * pose_cam0_.block<3, 3>(0, 0) * pts.R_t0_t1_cam0.cast<double>() *
  //                 R_imu_cam0_eigen_.transpose();
  //   first_iteration_save_ = false;
  // } else {
  //   R_t0_t1_imu = R_imu_cam0_eigen_ * pts.R_t0_t1_cam0.cast<double>() * R_imu_cam0_eigen_.transpose();
  // }

  // ProcessImu(pts.imu_pts);

  // Update our Pose
  pose_cam0_ = pose_cam0_ * pose_update_cam0;
  pose_cam0_.concatenate(pose_update_cam0);

  auto pose_body = pose_cam0_.rotate(R_imu_cam0_ * pose_cam0_.rotation() * R_imu_cam0_.t());

  // // Convert the pose to the body frame
  // Eigen::Matrix4d T_cam0_imu = Eigen::Matrix4d::Identity();
  // Eigen::Matrix4d T_imu_cam0 = Eigen::Matrix4d::Identity();
  // T_imu_cam0.block<3, 3>(0, 0) = R_imu_cam0_eigen_;
  // T_cam0_imu.block<3, 3>(0, 0) = R_imu_cam0_eigen_.transpose();
  // Eigen::Matrix4d pose_body = T_imu_cam0 * pose_cam0_ * T_cam0_imu;
  // // Eigen::Matrix4d pose_body = pose_cam0_;

  Eigen::Matrix<double, 6, 1> kf_state;
  ProcessVio(pose_body, pts.timestamp_us, kf_state);

  // visualization_.ReceiveData(pose_body, inliers);

  // Debug_SaveOutput(pose_body, R_t0_t1_imu);
  // Debug_SaveOutput(pose_body, pts.R_t0_t1_cam0 * R_imu_cam0_eigen_.transpose());

  // Copy the results to the output of this function
  vio.position << kf_state(0), kf_state(2), kf_state(4);
  vio.velocity << kf_state(1), kf_state(3), kf_state(5);

  Eigen::Matrix<double, 3, 3> pose_body_rot;
  cv::cv2eigen(pose_body.rotation(), pose_body_rot);
  cv::cv2eigen(pose_body.translation(), vio.position);

  vio.quat = Eigen::Quaterniond(pose_body_rot);

  return 0;
}

// int Vio::ProcessImu(const std::vector<mavlink_imu_t> &imu_pts) {
//   for (int i = 0; i < imu_pts.size(); i++) {
//     Eigen::Matrix3f m(Eigen::AngleAxisf(imu_pts[i].roll, Eigen::Vector3f::UnitX())
//       * Eigen::AngleAxisf(imu_pts[i].pitch,  Eigen::Vector3f::UnitY())
//       * Eigen::AngleAxisf(imu_pts[i].yaw, Eigen::Vector3f::UnitZ()));
//     Eigen::Vector3f gravity_vec; gravity_vec << 0.0, 0.0, -9.81;
//     Eigen::Vector3f gravity_vec_rotated = m * gravity_vec;

//     Eigen::Array<double, 1, 6> u(Eigen::Matrix<double, 1, 6>::Zero());
//     u(0) = imu_pts[i].accelXYZ[0] - gravity_vec_rotated(0);
//     u(1) = imu_pts[i].accelXYZ[0] - gravity_vec_rotated(0);
//     u(2) = imu_pts[i].accelXYZ[1] - gravity_vec_rotated(1);
//     u(3) = imu_pts[i].accelXYZ[1] - gravity_vec_rotated(1);
//     u(4) = imu_pts[i].accelXYZ[2] - gravity_vec_rotated(2);
//     u(4) = imu_pts[i].accelXYZ[2] - gravity_vec_rotated(2);

//     spdlog::info("U!!! " << u << std::endl;
//     if (last_timestamp_ != 0) {
//       kf_.Predict(u, static_cast<double>(imu_pts[i].timestamp_us - last_timestamp_) / 1.0E6);
//     }
//     last_timestamp_ = imu_pts[i].timestamp_us;
//   }
//   return 0;
// }

int Vio::ProcessVio(const cv::Affine3d &pose_body, uint64_t image_timestamp,
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

inline unsigned int Vio::Modulo(int value, unsigned m) {
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

int Vio::CalculatePoseUpdate(const TrackedImagePoints &pts, const Eigen::Matrix3d &imu_rotation,
                             cv::Affine3d &pose_update, std::vector<cv::Point3d> *inlier_pts) {
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

  // Perform Triangulation to get 3D points for time step T0
  cv::Mat pts_cam0_t0_mat(pts_cam0_t0_ud);
  pts_cam0_t0_mat.convertTo(pts_cam0_t0_mat, CV_64FC2);
  cv::Mat pts_cam1_t0_mat(pts_cam1_t0_ud);
  pts_cam1_t0_mat.convertTo(pts_cam1_t0_mat, CV_64FC2);
  // cv::Mat pts_cam0_t1_mat(pts_cam0_t1_ud);
  // pts_cam0_t1_mat.convertTo(pts_cam0_t1_mat, CV_64F);
  // cv::Mat pts_cam1_t1_mat(pts_cam1_t1_ud);
  // pts_cam1_t1_mat.convertTo(pts_cam1_t1_mat, CV_64F);

  cv::Mat triangulation_output_pts_homo;
  cv::triangulatePoints(P0_, P1_, pts_cam0_t0_mat, pts_cam1_t0_mat, triangulation_output_pts_homo);

  // Convert points from homogeneous to 3D coords
  std::vector<cv::Vec3d> triangulation_output_pts;
  cv::convertPointsFromHomogeneous(triangulation_output_pts_homo.t(), triangulation_output_pts);

  // std::for_each(triangulation_output_pts.begin(), triangulation_output_pts.end(),
  // [](auto &val) { std::cout << val << std::endl; });

  // cv::solvePnP	(	InputArray 	objectPoints,
  // InputArray 	imagePoints,
  // InputArray 	cameraMatrix,
  // InputArray 	distCoeffs,
  // OutputArray 	rvec,
  // OutputArray 	tvec,
  // bool 	useExtrinsicGuess = false,
  // int 	flags = SOLVEPNP_ITERATIVE
  // )

  std::vector<int> inliers;
  cv::Vec3d rvec, tvec;

  cv::solvePnPRansac(triangulation_output_pts, cv::Mat_<cv::Vec2d>(pts.cam0_t1.frame()), stereo_cal_.K_cam0,
                     stereo_cal_.D_cam0, rvec, tvec, false, 500, 0.05, 0.99, inliers, cv::SOLVEPNP_P3P);

  std::cout << " Inlier percentage openCV " << static_cast<float>(inliers.size()) / triangulation_output_pts.size()
            << std::endl;

  pose_update = cv::Affine3d(rvec, tvec).inv();

  std::cout << " translation " << pose_update.translation().t() << std::endl;

  // // put the points into the opengv object
  // opengv::points_t points(2 * pts.ids.size());
  // for (size_t i = 0; i < pts.ids.size(); i++) {
  //   cv::cv2eigen(triangulation_output_pts[i], points[i]);
  //   cv::cv2eigen(triangulation_output_pts[i], points[i + pts.ids.size()]);
  // }

  // // Containers for homogeneous undistorted points
  // std::vector<cv::Vec3f> xform_vec_cam0_t0;
  // std::vector<cv::Vec3f> xform_vec_cam0_t1;
  // std::vector<cv::Vec3f> xform_vec_cam1_t0;
  // std::vector<cv::Vec3f> xform_vec_cam1_t1;
  // cv::convertPointsToHomogeneous(pts_cam0_t0_ud, xform_vec_cam0_t0);
  // cv::convertPointsToHomogeneous(pts_cam0_t1_ud, xform_vec_cam0_t1);
  // cv::convertPointsToHomogeneous(pts_cam1_t0_ud, xform_vec_cam1_t0);
  // cv::convertPointsToHomogeneous(pts_cam1_t1_ud, xform_vec_cam1_t1);

  // // Containers for use with OpenGV
  // opengv::bearingVectors_t bearing_vectors_cam0_t0;
  // opengv::bearingVectors_t bearing_vectors_cam0_t1;
  // opengv::bearingVectors_t bearing_vectors_cam1_t0;
  // opengv::bearingVectors_t bearing_vectors_cam1_t1;
  // std::vector<int> cam_correspondences_cam0_t0;
  // std::vector<int> cam_correspondences_cam0_t1;
  // std::vector<int> cam_correspondences_cam1_t0;
  // std::vector<int> cam_correspondences_cam1_t1;

  // // Fill the bearing/correspondence vectors. The bearing unit vector is unit vector pointing in
  // // the same direction as the homogeneous 3D point
  // for (size_t i = 0; i < pts.cam0_t0.frame().cols; i++) {
  //   opengv::bearingVector_t bearing_vec;
  //   cv::cv2eigen(static_cast<cv::Vec3d>(xform_vec_cam0_t0[i]), bearing_vec);
  //   bearing_vec /= bearing_vec.norm();
  //   bearing_vectors_cam0_t0.push_back(bearing_vec);
  //   cam_correspondences_cam0_t0.push_back(0);

  //   cv::cv2eigen(static_cast<cv::Vec3d>(xform_vec_cam0_t1[i]), bearing_vec);
  //   bearing_vec /= bearing_vec.norm();
  //   bearing_vectors_cam0_t1.push_back(bearing_vec);
  //   cam_correspondences_cam0_t1.push_back(0);

  //   cv::cv2eigen(static_cast<cv::Vec3d>(xform_vec_cam1_t0[i]), bearing_vec);
  //   bearing_vec /= bearing_vec.norm();
  //   bearing_vectors_cam1_t0.push_back(bearing_vec);
  //   cam_correspondences_cam1_t0.push_back(1);

  //   cv::cv2eigen(static_cast<cv::Vec3d>(xform_vec_cam1_t1[i]), bearing_vec);
  //   bearing_vec /= bearing_vec.norm();
  //   bearing_vectors_cam1_t1.push_back(bearing_vec);
  //   cam_correspondences_cam1_t1.push_back(1);
  // }

  // // Construct a vector of bearing vectors in timetemp T1 that points at 'points' in T0
  // opengv::bearingVectors_t bearing_vectors_t1;
  // bearing_vectors_t1.insert(std::end(bearing_vectors_t1), std::begin(bearing_vectors_cam0_t1),
  //                           std::end(bearing_vectors_cam0_t1));
  // bearing_vectors_t1.insert(std::end(bearing_vectors_t1), std::begin(bearing_vectors_cam1_t1),
  //                           std::end(bearing_vectors_cam1_t1));
  // std::vector<int> correspondences_t1;
  // correspondences_t1.insert(std::end(correspondences_t1), std::begin(cam_correspondences_cam0_t1),
  //                           std::end(cam_correspondences_cam0_t1));
  // correspondences_t1.insert(std::end(correspondences_t1), std::begin(cam_correspondences_cam1_t1),
  //                           std::end(cam_correspondences_cam1_t1));

  // // Add the params from our camera system
  // opengv::translations_t camOffsets;
  // opengv::rotations_t camRotations;
  // camOffsets.push_back(Eigen::Vector3d::Zero());
  // camRotations.push_back(Eigen::Matrix3d::Identity());
  // Eigen::Matrix3d R;
  // cv::cv2eigen(stereo_cal_.R_cam0_cam1.t(), R);
  // Eigen::MatrixXd T;
  // cv::cv2eigen(-stereo_cal_.T_cam0_cam1, T);
  // camRotations.push_back(R);
  // camOffsets.push_back(T);

  // // Create a non-central absolute adapter
  // opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(bearing_vectors_t1, correspondences_t1, points,
  // camOffsets,
  //                                                          camRotations, imu_rotation);

  // // Create the ransac model and compute
  // opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
  // std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
  //     new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
  //         adapter, opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P));
  // ransac.sac_model_ = absposeproblem_ptr;
  // ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0) * 0.5 / 500.0));
  // ransac.max_iterations_ = 1000;
  // ransac.computeModel();

  // // If requested, copy the inlier points
  // if (inlier_pts != nullptr) {
  //   inlier_pts->clear();
  //   for (size_t i = 0; i < ransac.inliers_.size(); i++) {
  //     Eigen::Vector3d &pt = points[ransac.inliers_[i]];
  //     inlier_pts->push_back(cv::Point3d(pt(0), pt(1), pt(2)));
  //   }
  // }

  // // //print the results
  // // std::cout << "the ransac results is: " << std::endl;
  // // std::cout << ransac.model_coefficients_ << std::endl << std::endl;
  // // std::cout << "Ransac needed " << ransac.iterations_ << " iterations and ";
  // // // std::cout << ransac_time << " seconds" << std::endl << std::endl;
  // // std::cout << "the number of inliers is: " << ransac.inliers_.size();
  // // std::cout << std::endl << std::endl;
  // // std::cout << "the found inliers are: " << std::endl;
  // //   std::cout << points[ransac.inliers_[i]].transpose() << "\n";
  // // std::cout << std::endl << std::endl;

  // pose_update = Eigen::Matrix4d::Identity();
  // // pose_cam0_ = pose_cam0_ * delta_xform ;
  // // std::cout << "output transformation: \n" << pose_cam0_ << std::endl;

  // std::cout << " Inlier percentage openGV " << static_cast<double>(ransac.inliers_.size()) /
  // bearing_vectors_t1.size()
  //           << std::endl;
  // std::cout << "\n opencv output \n" << affine << std::endl << std::endl;
  // std::cout << "\n opengv output \n" << ransac.model_coefficients_ << std::endl << std::endl;

  // Put the ransc output pose update into a 4x4 Matrix
  // bool USE_OPENGV = false;
  // if (USE_OPENGV) {
  //   // pose_update.block<3, 4>(0, 0) = ransac.model_coefficients_;
  // } else {
  //   auto mat_rotation = affine.rotation();
  //   auto mat_translation = affine.translation();
  //   for (auto row = 0; row < 3; row++) {
  //     for (auto col = 0; col < 3; col++) {
  //       pose_update(row, col) = mat_rotation(row, col);
  //     }
  //     pose_update(row, 3) = mat_translation[row];
  //   }
  // }
  // pose_update(3, 3) = 1;

  // // Apply the user calibration
  // pose_update.block<3, 1>(0, 3) -= vio_calibration_;

  // if (pose_update.block<3, 1>(0, 3).norm() > 10) {
  //   pose_update = Eigen::Matrix4d::Identity();
  //   spdlog::error("Error! Single frame distance over threshold. Discarding update");
  // }

  // std::cout << "delta update: \n" << pose_update << std::endl;
  // std::cout << "output transformation: \n" << pose_cam0_ << std::endl;

  // pose_history_[Modulo(static_cast<int>(point_history_index_), history_size)] = pose_cam0_;
  // point_history_index_ = (point_history_index_ >= history_size - 1) ? 0 : point_history_index_ + 1;

  return 0;
}

int Vio::Debug_SaveOutput(const Eigen::Matrix4d &pose_update, const Eigen::Matrix3d &R_imu) {
  // if (ransac.model_coefficients_(0,3) > 5 || ransac.model_coefficients_(1,3) > 5 || ransac.model_coefficients_(2,3) >
  // 5) {
  if (1) {
    // unsigned int index_temp = Modulo(point_history_index_ - 2, history_size);
    // Eigen::Matrix4d pose_inv = Eigen::Matrix4d::Identity();
    // pose_inv.block<3, 3>(0,0) = pose_history_[index_temp].block<3, 3>(0, 0).transpose();
    // pose_inv.block<3, 1>(0,3) = -pose_history_[index_temp].block<3, 1>(0, 3);

    // for (int i = 0; i < ransac.inliers_.size(); i++) {
    //   Eigen::Vector4d first_pt;
    //   first_pt.head<3>() = points[ransac.inliers_[i]];
    //   first_pt[3] = 1;

    //   Eigen::Vector4d point_adj = pose_inv * first_pt;
    //   liblas::Point point(header_.get());
    //   // point.SetCoordinates(point_adj[0], point_adj[1], point_adj[2]);
    //   point.SetCoordinates(first_pt[0], first_pt[1], first_pt[2]);
    //   writer_->WritePoint(point);
    // }
    if (trajecotry_file_) {
      // Eigen::Matrix<double, 3, 4> writer_pose = pose_update.block<3, 4> (0, 0);
      // Eigen::Map<Eigen::RowVectorXd> v(writer_pose.data(), writer_pose.size());
      const Eigen::IOFormat csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, ",", ",");

      // Eigen::Matrix3d R_imu_temp = R_imu;
      // Eigen::Map<Eigen::RowVectorXd> imu(R_imu_temp.data(), R_imu_temp.size());
      *trajecotry_file_ << pose_update.format(csv_format) << "," << kf_.GetState().format(csv_format) << ","
                        << R_imu.format(csv_format) << std::endl;
    }

    // for (int i = 0; i < points.size(); i++) {
    //   // ofs  << vec[i](0) << "," << vec[i](1) << "," << vec[i](2) << std::endl;
    //   ofs  << points[i].transpose() << std::endl;
    // }
    // std::exit(1);
  }
  return 0;
}

// int Vio::SaveInliers(std::vector<int> inliers, std::vector<int> pt_ids, opengv::points_t pts) {
//   // Remove the duplicate points
//   unsigned int num_pts = pts.size();
//   for (unsigned int i = 0; i < inliers.size(); i++) {
//     inliers[i] = Modulo(inliers[i], (num_pts / 2));
//   }
//   // Erase duplicates caused from the modulo
//   std::sort(inliers.begin(), inliers.end());
//   inliers.erase(unique(inliers.begin(), inliers.end()), inliers.end());

//   // Take the inliers from the current interation and put them into the point history map
//   std::map<unsigned int, opengv::point_t> temp_map;
//   for (unsigned int i = 0; i < inliers.size(); i++) {
//     temp_map[pt_ids[inliers[i]]] = pts[inliers[i]];
//   }
//   point_history_[Modulo(point_history_index_, history_size)] = (temp_map);

//   // Create a vector of ids for the inlier points
//   std::vector<int> ids_inliers(inliers.size());
//   for (unsigned int i = 0; i < inliers.size(); i++) {
//     ids_inliers[i] = pt_ids[inliers[i]];
//   }

//   // Find how many of the points from the current iteration match previous iterations
//   std::array<unsigned int, history_size> num_matches;
//   num_matches[0] = 0;
//   for (unsigned int i = 1; i < history_size; i++) {
//     const auto &temp_map = point_history_[Modulo((static_cast<int>(point_history_index_) - i), history_size)];

//     unsigned int num_matches_local = 0;
//     for (unsigned int j = 0; j < ids_inliers.size(); j++) {
//       if (temp_map.count(ids_inliers[j])) {
//         num_matches_local++;
//       }
//     }
//     // std::cout << "Num Matches: " << num_matches_local <<" temp map size " << temp_map.size() << std::endl;
//     num_matches[i] = num_matches_local;
//   }

//   // Find the point furthest back in the history that has the minimum number of matches points
//   for (int i = num_matches.size() - 1; i >= 0; i--) {
//     if (num_matches[i] >= min_num_matches) {
//       // std::cout << "i: " << i << " point history index: " << point_history_index_ << " output: " <<
//       // Modulo((static_cast<int>(point_history_index_) - i), history_size) << std::endl; int temp_ind_val =
//       // Modulo((static_cast<int>(point_history_index_) - i), history_size);
//       return Modulo((static_cast<int>(point_history_index_) - i), history_size);
//     }
//   }

//   return -5;
// }
