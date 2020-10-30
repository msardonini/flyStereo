
#include "fly_stereo/vio.h"

// System includes
#include <iostream>
#include <fstream>

// Package includes
#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"  // To print out Eigen objects
#include "Eigen/Geometry"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opengv/relative_pose/CentralRelativeAdapter.hpp"
#include "opengv/triangulation/methods.hpp"
#include "opengv/absolute_pose/methods.hpp"
#include "opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp"
#include "opengv/sac/Ransac.hpp"
#include "opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp"
#include "opengv/point_cloud/methods.hpp"
#include "opengv/point_cloud/PointCloudAdapter.hpp"
#include "opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp"

constexpr unsigned int history_size = 10;
constexpr unsigned int min_num_matches = 25;
// Constructor with config params
Vio::Vio(const YAML::Node &input_params, const YAML::Node &stereo_calibration) :
  kf_(input_params["vio"]["kalman_filter"]) {

  // If we have recording enabled, initialize the logging files
  if (input_params["record_mode"] &&
      input_params["record_mode"]["enable"].as<bool>() &&
      input_params["record_mode"]["outputs"]["trajectory"].as<bool>()) {
    std::string run_file = input_params["record_mode"]["log_dir"].as<std::string>();
    trajecotry_file_ = std::make_unique<std::ofstream> (run_file + "/trajectory.txt",
      std::ios::out);
  }

  YAML::Node vio_params = input_params["vio"];
  // Parse params for the VIO node
  std::vector<double> vio_calibration_vec = vio_params["vio_calibration"].as<
    std::vector<double>>();
  vio_calibration_ = Eigen::Matrix<double, 3, 1>(vio_calibration_vec.data());


  std::vector<double> interface_vec = stereo_calibration["K0"]["data"].as<std::vector<double>>();
  K_cam0_ = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["K1"]["data"].as<std::vector<double>>();
  K_cam1_ = cv::Matx33d(interface_vec.data());

  D_cam0_ = stereo_calibration["D0"]["data"].as<std::vector<double>>();

  D_cam1_ = stereo_calibration["D1"]["data"].as<std::vector<double>>();

  interface_vec = stereo_calibration["R_imu_cam0"].as<std::vector<double>>();
  R_imu_cam0_eigen_ =  Eigen::AngleAxisd(interface_vec[0], Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(interface_vec[1],  Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(interface_vec[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();

  spdlog::info(" rotation: {}", R_imu_cam0_eigen_);

  interface_vec = stereo_calibration["R"]["data"].as<std::vector<double>>();
  R_cam0_cam1_ = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["T"]["data"].as<std::vector<double>>();
  T_cam0_cam1_ = cv::Vec3d(interface_vec.data());

  // Create the projection matrices such that the output points will be in the coordinate system
  // of cam0
  P0_ = cv::Matx34d::eye();
  P1_ = cv::Matx34d::zeros();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P1_(i, j) = R_cam0_cam1_(i, j);
    }
    P1_(i, 3) = T_cam0_cam1_[i];
  }

  // Set the initial pose to identity
  pose_cam0_ = Eigen::Matrix4d::Identity();
  first_iteration_ = true;

  point_history_index_ = 0;
  point_history_.resize(history_size);
  pose_history_.resize(history_size);

  ofs_ = std::make_unique<std::ofstream> ("points.las", std::ios::out | std::ios::binary);
  header_ = std::make_unique<liblas::Header>();
  header_->SetDataFormatId(liblas::ePointFormat1); // Time only
  header_->SetScale(0.0001, 0.0001, 0.0001);
  writer_ = std::make_unique<liblas::Writer> (*ofs_.get(), *header_.get());
}

// Destructor
Vio::~Vio() {}

int Vio::ProcessPoints(const ImagePoints &pts, vio_t &vio) {
  // If this is our first update, set our initial pose by the rotation of the imu

  // Calculate the updated pose
  if (first_iteration_) {
    if (pts.imu_pts.size() == 0) {
      return 0;
    }
    first_iteration_ = false;


    Eigen::Matrix3d initial_rotation =
      Eigen::AngleAxisd(-pts.imu_pts[0].roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(-pts.imu_pts[0].pitch,  Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
      .toRotationMatrix();

    // Eigen::Vector3d imu_rot; imu_rot << pts.imu_pts[0].roll, pts.imu_pts[0].pitch, 0.0;
    // Eigen::Vector3d initial_rotation_vec = R_imu_cam0_eigen_ * imu_rot;

    // Eigen::Matrix3d initial_rotation =
    //   Eigen::AngleAxisd(initial_rotation_vec(0), Eigen::Vector3d::UnitX()) *
    //   Eigen::AngleAxisd(initial_rotation_vec(1),  Eigen::Vector3d::UnitY()) *
    //   Eigen::AngleAxisd(initial_rotation_vec(2), Eigen::Vector3d::UnitZ())
    //   .toRotationMatrix().transpose();

    // TODO figure out if we need to transpose the initial rotation
    pose_cam0_.block<3, 3>(0, 0) = R_imu_cam0_eigen_.transpose() * initial_rotation.transpose()
      * R_imu_cam0_eigen_;
    spdlog::info("first imu point: {}, {}, {}", pts.imu_pts[0].roll, pts.imu_pts[0].pitch,
      pts.imu_pts[0].yaw);
    spdlog::info("Mat1: {}", initial_rotation);
    spdlog::info("Mat2: {}", R_imu_cam0_eigen_);
  }

  Eigen::Matrix4d pose_update_cam0;
  if (CalculatePoseUpdate(pts, pts.R_t0_t1_cam0, pose_update_cam0)) {
    spdlog::warn("Error in CalculatePoseUpdate");
    return -1;
  }

  // ProcessImu(pts.imu_pts);
  Eigen::Matrix<double, 6, 1> kf_state;

  // Update our Pose
  pose_cam0_ = pose_cam0_ * pose_update_cam0;

  // Convert the pose to the body frame
  Eigen::Matrix4d T_cam0_imu = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_imu_cam0 = Eigen::Matrix4d::Identity();
  T_imu_cam0.block<3, 3>(0, 0) = R_imu_cam0_eigen_;
  T_cam0_imu.block<3, 3>(0, 0) = R_imu_cam0_eigen_.transpose();
  Eigen::Matrix4d pose_body = T_imu_cam0 * pose_cam0_ * T_cam0_imu;
  // Eigen::Matrix4d pose_body = pose_cam0_;

  ProcessVio(pose_body, pts.timestamp_us, kf_state);

  Eigen::Matrix3d R_t0_t1_imu = R_imu_cam0_eigen_ * pts.R_t0_t1_cam0 *
    R_imu_cam0_eigen_.transpose();
  Debug_SaveOutput(pose_body, R_t0_t1_imu);
  // Debug_SaveOutput(pose_body, pts.R_t0_t1_cam0 * R_imu_cam0_eigen_.transpose());

  // Copy the results to the output of this function
  vio.position << kf_state(0), kf_state(2), kf_state(4);
  vio.velocity << kf_state(1), kf_state(3), kf_state(5);
  vio.quat = Eigen::Quaterniond(pose_body.block<3, 3>(0, 0));

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

int Vio::ProcessVio(const Eigen::Matrix4d &pose_body, uint64_t image_timestamp,
  Eigen::Matrix<double, 6, 1> &output_state) {

  Eigen::Matrix<double, 3, 1> z;
  z = pose_body.block<3, 1>(0, 3);

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

int Vio::CalculatePoseUpdate(const ImagePoints &pts, const Eigen::Matrix3d &imu_rotation,
  Eigen::Matrix4d &pose_update) {
  // Calculate the number of points in the grid
  const unsigned int num_pts = pts.pts.size();

  // Define vectors to hold the points we will use for processing
  std::vector<cv::Point2d> pts_cam0_t0(num_pts);
  std::vector<cv::Point2d> pts_cam0_t1(num_pts);
  std::vector<cv::Point2d> pts_cam1_t0(num_pts);
  std::vector<cv::Point2d> pts_cam1_t1(num_pts);
  std::vector<int> pts_ids(num_pts);

  // Take our points out of the grid and put into vectors for processing
  for (size_t i = 0; i < num_pts; i++) {
    pts_cam0_t0[i] = cv::Point2d(pts.pts[i].cam0_t0[0], pts.pts[i].cam0_t0[1]);
    pts_cam0_t1[i] = cv::Point2d(pts.pts[i].cam0_t1[0], pts.pts[i].cam0_t1[1]);
    pts_cam1_t0[i] = cv::Point2d(pts.pts[i].cam1_t0[0], pts.pts[i].cam1_t0[1]);
    pts_cam1_t1[i] = cv::Point2d(pts.pts[i].cam1_t1[0], pts.pts[i].cam1_t1[1]);
    pts_ids[i] = pts.pts[i].id;
  }

  // Check to make sure all of our vectors have the same number of points, otherwise, something
  // went wrong
  unsigned int num_pts_cam0_t0 = pts_cam0_t0.size();
  if (num_pts_cam0_t0 != pts_cam0_t1.size() ||
      num_pts_cam0_t0 != pts_cam1_t0.size() ||
      num_pts_cam0_t0 != pts_cam1_t1.size()) {
    spdlog::error("Error! [ProcessPoints] points are not the same size!");
    return -1;
  } else if (pts_cam0_t0.size() == 0) {
    return -1;
  }

  if (num_pts_cam0_t0 < 6) {
    spdlog::warn("Not enough points for algorithm!");
    return -1;
  }

  // Points that are measured to be 45 degrees off center, and 22.5 deg off center
  // pts_cam0_t1.clear();
  // pts_cam0_t1.push_back(cv::Point2d(629, 397));
  // pts_cam0_t1.push_back(cv::Point2d(1162, 347));
  // pts_cam0_t1.push_back(cv::Point2d(625, 674));

  // Containers for the undistorted points
  std::vector<cv::Point2d> pts_cam0_t0_ud;
  std::vector<cv::Point2d> pts_cam0_t1_ud;
  std::vector<cv::Point2d> pts_cam1_t0_ud;
  std::vector<cv::Point2d> pts_cam1_t1_ud;

  // Undistort the points using the intrinsic calibration
  cv::undistortPoints(pts_cam0_t0, pts_cam0_t0_ud, K_cam0_, D_cam0_);
  cv::undistortPoints(pts_cam0_t1, pts_cam0_t1_ud, K_cam0_, D_cam0_);
  cv::undistortPoints(pts_cam1_t0, pts_cam1_t0_ud, K_cam1_, D_cam1_);
  cv::undistortPoints(pts_cam1_t1, pts_cam1_t1_ud, K_cam1_, D_cam1_);

  // Perform Triangulation to get 3D points for time step T0
  cv::Mat pts_cam0_t0_mat(pts_cam0_t0_ud); pts_cam0_t0_mat.convertTo(pts_cam0_t0_mat, CV_64F);
  cv::Mat pts_cam1_t0_mat(pts_cam1_t0_ud); pts_cam1_t0_mat.convertTo(pts_cam1_t0_mat, CV_64F);
  cv::Mat triangulation_output_pts_homo;
  cv::triangulatePoints(P0_, P1_, pts_cam0_t0_mat, pts_cam1_t0_mat, triangulation_output_pts_homo);

  // Convert points from homogeneous to 3D coords
  std::vector<cv::Vec3d> triangulation_output_pts;
  cv::convertPointsFromHomogeneous(triangulation_output_pts_homo.t(), triangulation_output_pts);

  // put the points into the opengv object
  opengv::points_t points(2 * num_pts);
  for(size_t i = 0; i < num_pts; i++) {
    cv::cv2eigen(triangulation_output_pts[i], points[i]);
    cv::cv2eigen(triangulation_output_pts[i], points[i + num_pts]);
  }

  // Containers for homogeneous undistorted points
  std::vector<cv::Vec3d> xform_vec_cam0_t0;
  std::vector<cv::Vec3d> xform_vec_cam0_t1;
  std::vector<cv::Vec3d> xform_vec_cam1_t0;
  std::vector<cv::Vec3d> xform_vec_cam1_t1;
  cv::convertPointsToHomogeneous(pts_cam0_t0_ud, xform_vec_cam0_t0);
  cv::convertPointsToHomogeneous(pts_cam0_t1_ud, xform_vec_cam0_t1);
  cv::convertPointsToHomogeneous(pts_cam1_t0_ud, xform_vec_cam1_t0);
  cv::convertPointsToHomogeneous(pts_cam1_t1_ud, xform_vec_cam1_t1);

  // Containers for use with OpenGV
  opengv::bearingVectors_t bearing_vectors_cam0_t0;
  opengv::bearingVectors_t bearing_vectors_cam0_t1;
  opengv::bearingVectors_t bearing_vectors_cam1_t0;
  opengv::bearingVectors_t bearing_vectors_cam1_t1;
  std::vector<int> cam_correspondences_cam0_t0;
  std::vector<int> cam_correspondences_cam0_t1;
  std::vector<int> cam_correspondences_cam1_t0;
  std::vector<int> cam_correspondences_cam1_t1;

  // Fill the bearing/correspondence vectors. The bearing unit vector is unit vector pointing in
  // the same direction as the homogeneous 3D point
  for (size_t i = 0; i < pts_cam0_t0.size(); i++) {
    opengv::bearingVector_t bearing_vec;
    cv::cv2eigen(static_cast<cv::Vec3d>(xform_vec_cam0_t0[i]), bearing_vec);
    bearing_vec /= bearing_vec.norm();
    bearing_vectors_cam0_t0.push_back(bearing_vec);
    cam_correspondences_cam0_t0.push_back(0);

    cv::cv2eigen(static_cast<cv::Vec3d>(xform_vec_cam0_t1[i]), bearing_vec);
    bearing_vec /= bearing_vec.norm();
    bearing_vectors_cam0_t1.push_back(bearing_vec);
    cam_correspondences_cam0_t1.push_back(0);

    cv::cv2eigen(static_cast<cv::Vec3d>(xform_vec_cam1_t0[i]), bearing_vec);
    bearing_vec /= bearing_vec.norm();
    bearing_vectors_cam1_t0.push_back(bearing_vec);
    cam_correspondences_cam1_t0.push_back(1);

    cv::cv2eigen(static_cast<cv::Vec3d>(xform_vec_cam1_t1[i]), bearing_vec);
    bearing_vec /= bearing_vec.norm();
    bearing_vectors_cam1_t1.push_back(bearing_vec);
    cam_correspondences_cam1_t1.push_back(1);
  }

  // Construct a vector of bearing vectors in timetemp T1 that points at 'points' in T0
  opengv::bearingVectors_t bearing_vectors_t1;
  bearing_vectors_t1.insert(std::end(bearing_vectors_t1), std::begin(bearing_vectors_cam0_t1),
    std::end(bearing_vectors_cam0_t1));
  bearing_vectors_t1.insert(std::end(bearing_vectors_t1), std::begin(bearing_vectors_cam1_t1),
    std::end(bearing_vectors_cam1_t1));
  std::vector<int> correspondences_t1;
  correspondences_t1.insert(std::end(correspondences_t1), std::begin(cam_correspondences_cam0_t1),
    std::end(cam_correspondences_cam0_t1));
  correspondences_t1.insert(std::end(correspondences_t1), std::begin(cam_correspondences_cam1_t1),
    std::end(cam_correspondences_cam1_t1));

  // Add the params from our camera system
  opengv::translations_t camOffsets;
  opengv::rotations_t camRotations;
  camOffsets.push_back(Eigen::Vector3d::Zero());
  camRotations.push_back(Eigen::Matrix3d::Identity());
  Eigen::Matrix3d R; cv::cv2eigen(R_cam0_cam1_.t(), R);
  Eigen::MatrixXd T; cv::cv2eigen(-T_cam0_cam1_, T);
  camRotations.push_back(R);
  camOffsets.push_back(T);

  // Create a non-central absolute adapter
  opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(
    bearing_vectors_t1,
    correspondences_t1,
    points,
    camOffsets,
    camRotations,
    imu_rotation);

  // Create the ransac model and compute
  opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
  std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
    new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(adapter,
      opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P));
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/500.0));
  ransac.max_iterations_ = 200;
  ransac.computeModel();

  // //print the results
  // std::cout << "the ransac results is: " << std::endl;
  // std::cout << ransac.model_coefficients_ << std::endl << std::endl;
  // std::cout << "Ransac needed " << ransac.iterations_ << " iterations and ";
  // // std::cout << ransac_time << " seconds" << std::endl << std::endl;
  // std::cout << "the number of inliers is: " << ransac.inliers_.size();
  // std::cout << std::endl << std::endl;
  // std::cout << "the found inliers are: " << std::endl;
  // for(size_t i = 0; i < ransac.inliers_.size(); i++)
  //   std::cout << points[ransac.inliers_[i]].transpose() << "\n";
  // std::cout << std::endl << std::endl;

  pose_update = Eigen::Matrix4d::Identity();
  // pose_cam0_ = pose_cam0_ * delta_xform ;
  // std::cout << "output transformation: \n" << pose_cam0_ << std::endl;


  // Put the ransc output pose update into a 4x4 Matrix
  pose_update.block<3,4>(0, 0) = ransac.model_coefficients_;

  // Apply the user calibration
  pose_update.block<3,1>(0, 3) -= vio_calibration_;


  if (pose_update.block<3, 1>(0, 3).norm() > 1) {
    pose_update = Eigen::Matrix4d::Identity();
    spdlog::error("Error! Single frame distance over threshold. Discarding update");
  }


  // std::cout << "delta update: \n" << pose_update << std::endl;
  // std::cout << "output transformation: \n" << pose_cam0_ << std::endl;

  // pose_history_[Modulo(static_cast<int>(point_history_index_), history_size)] = pose_cam0_;
  // point_history_index_ = (point_history_index_ >= history_size - 1) ? 0 : point_history_index_ + 1;

  return 0;
}

int Vio::Debug_SaveOutput(const Eigen::Matrix4d &pose_update, const Eigen::Matrix3d &R_imu) {
  // if (ransac.model_coefficients_(0,3) > 5 || ransac.model_coefficients_(1,3) > 5 || ransac.model_coefficients_(2,3) > 5) {
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
      *trajecotry_file_ << pose_update.format(csv_format) << "," << kf_.GetState().format(csv_format) << "," << R_imu.format(csv_format) << std::endl;
    }

    // for (int i = 0; i < points.size(); i++) {
    //   // ofs  << vec[i](0) << "," << vec[i](1) << "," << vec[i](2) << std::endl;
    //   ofs  << points[i].transpose() << std::endl;
    // }
    // std::exit(1);
  }
  return 0;
}


int Vio::SaveInliers(std::vector<int> inliers, std::vector<int> pt_ids, opengv::points_t pts) {
  // Remove the duplicate points
  unsigned int num_pts = pts.size();
  for (unsigned int i = 0; i < inliers.size(); i++) {
    inliers[i] = Modulo(inliers[i], (num_pts / 2));
  }
  // Erase duplicates caused from the modulo
  std::sort(inliers.begin(), inliers.end());
  inliers.erase(unique(inliers.begin(), inliers.end()), inliers.end());

  // Take the inliers from the current interation and put them into the point history map
  std::map<unsigned int, opengv::point_t> temp_map;
  for (unsigned int i = 0; i < inliers.size(); i++) {
    temp_map[pt_ids[inliers[i]]] = pts[inliers[i]];
  }
  point_history_[Modulo(point_history_index_, history_size)] = (temp_map);

  // Create a vector of ids for the inlier points
  std::vector<int> ids_inliers(inliers.size());
  for (unsigned int i = 0; i < inliers.size(); i++) {
    ids_inliers[i] = pt_ids[inliers[i]];
  }

  // Find how many of the points from the current iteration match previous iterations
  std::array<unsigned int, history_size> num_matches; num_matches[0] = 0;
  for (unsigned int i = 1; i < history_size; i++) {
    const auto &temp_map = point_history_[Modulo((static_cast<int>(point_history_index_) - i), history_size)];

    unsigned int num_matches_local = 0;
    for (unsigned int j = 0; j < ids_inliers.size(); j++) {
      if(temp_map.count(ids_inliers[j])) {
        num_matches_local++;
      }
    }
    // std::cout << "Num Matches: " << num_matches_local <<" temp map size " << temp_map.size() << std::endl;
    num_matches[i] = num_matches_local;
  }

  // Find the point furthest back in the history that has the minimum number of matches points
  for (int i = num_matches.size() - 1; i >= 0; i--) {
    if (num_matches[i] >= min_num_matches) {
      // std::cout << "i: " << i << " point history index: " << point_history_index_ << " output: " << Modulo((static_cast<int>(point_history_index_) - i), history_size) << std::endl;
      // int temp_ind_val = Modulo((static_cast<int>(point_history_index_) - i), history_size);
      return Modulo((static_cast<int>(point_history_index_) - i), history_size);
    }
  }

  return -5;
}