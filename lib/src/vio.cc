
#include "fly_stereo/vio.h"

// System includes
#include <iostream>
#include <fstream>

// Package includes
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
  kf_(input_params["kalman_filter"]) {

  std::vector<double> vio_calibration_vec = input_params["vio_calibration"].as<
    std::vector<double>>();
  vio_calibration_ = Eigen::Matrix<double, 3, 1>(vio_calibration_vec.data());

  // Parse params for the VIO node
  image_width_ = input_params["image_width"].as<unsigned int>();
  image_height_= input_params["image_height"].as<unsigned int>();
  bins_width_ = input_params["bins_width"].as<unsigned int>();
  bins_height_ = input_params["bins_height"].as<unsigned int>();
  max_pts_in_bin_ = input_params["max_pts_in_bin"].as<unsigned int>();

  std::vector<double> interface_vec = stereo_calibration["K0"]["data"].as<
    std::vector<double>>();
  K_cam0_ = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["K1"]["data"].as<std::vector<double>>();
  K_cam1_ = cv::Matx33d(interface_vec.data());

  D_cam0_ = stereo_calibration["D0"]["data"].as<std::vector<double>>();

  D_cam1_ = stereo_calibration["D1"]["data"].as<std::vector<double>>();

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
  pose_ = Eigen::Matrix4d::Identity();

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

int Vio::ProcessPoints(const ImagePoints &pts, Eigen::Vector3d &pose) {
  // All points are placed in bins, sections of the image. The key to this map is the row-major
  // index of the bin. The value is a vector of image points, which holds points from both cameras
  // in the current frame and the previos one
  std::map<int, std::vector<ImagePoint> > grid;

  BinFeatures(pts, grid);

  Eigen::Vector3f position_vio;
  if (CalculatePoseUpdate(grid, position_vio) == 0) {
    // TODO: add imu prediction and vo measurement update
    // ProcessImu(pts.imu_pts);
    ProcessVio(position_vio, pts.timestamp_us);
    Debug_SaveOutput();
  }
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

//     std::cout << "U!!! " << u << std::endl;
//     if (last_timestamp_ != 0) {
//       kf_.Predict(u, static_cast<double>(imu_pts[i].timestamp_us - last_timestamp_) / 1.0E6);
//     }
//     last_timestamp_ = imu_pts[i].timestamp_us;
//   }
//   return 0;
// }

int Vio::ProcessVio(const Eigen::Vector3f &position_vio, uint64_t image_timestamp) {
  Eigen::Matrix<double, 1, 3> z(Eigen::Matrix<double, 1, 3>::Zero());

  z(0) = position_vio(0);
  z(1) = position_vio(1);
  z(2) = position_vio(2);

  if (last_timestamp_ == 0) {
    kf_.Predict();
  } else {
    kf_.Predict(static_cast<double>(image_timestamp - last_timestamp_) / 1.0E6);
  }
  last_timestamp_ = image_timestamp;
  kf_.Measure(z);
  return 0;
}

int Vio::BinFeatures(const ImagePoints &pts, std::map<int, std::vector<ImagePoint> > &grid) {
  // Place all of our image points in our map
  // First, calculate the row-major index of the bin, this will be the map's key
  for (int i = 0; i < pts.pts.size(); i++) {
    unsigned int bin_row = pts.pts[i].cam0_t0[1] * bins_height_ / image_height_;
    unsigned int bin_col = pts.pts[i].cam0_t0[0] * bins_width_ / image_width_;

    unsigned int index = bins_width_ * bin_row + bin_col;
    grid[index].push_back(pts.pts[i]);
  }

  // Now that the grid is full, delete points that exceed the threshold of points per bin
  for (auto it = grid.begin(); it != grid.end(); it++) {
    if (it->second.size() >= max_pts_in_bin_) {
      unsigned int num_pts_delete = it->second.size() - max_pts_in_bin_;
      it->second.erase(it->second.end() - num_pts_delete, it->second.end());
    }
  }

  return 0;
}

inline unsigned int Vio::Modulo(int value, unsigned m) {
    int mod = value % (int)m;
    if (value < 0) {
        mod += m;
    }
    return mod;
}

int Vio::CalculatePoseUpdate(const std::map<int, std::vector<ImagePoint> > &grid,
  Eigen::Vector3f &position_update) {
  // Calculate the number of points in the grid
  unsigned int num_pts = 0;
  for (auto it = grid.begin(); it != grid.end(); it++) {
    num_pts += it->second.size();
  }

  // Define vectors to hold the points we will use for processing
  std::vector<cv::Point2d> pts_cam0_t0(num_pts);
  std::vector<cv::Point2d> pts_cam0_t1(num_pts);
  std::vector<cv::Point2d> pts_cam1_t0(num_pts);
  std::vector<cv::Point2d> pts_cam1_t1(num_pts);
  std::vector<int> pts_ids(num_pts);

  // Take our points out of the grid and put into vectors for processing
  unsigned int index = 0;
  for (auto it = grid.begin(); it != grid.end(); it++) {
    for (const ImagePoint &pt : it->second) {
      pts_cam0_t0[index] = cv::Point2d(pt.cam0_t0[0], pt.cam0_t0[1]);
      pts_cam0_t1[index] = cv::Point2d(pt.cam0_t1[0], pt.cam0_t1[1]);
      pts_cam1_t0[index] = cv::Point2d(pt.cam1_t0[0], pt.cam1_t0[1]);
      pts_cam1_t1[index] = cv::Point2d(pt.cam1_t1[0], pt.cam1_t1[1]);
      pts_ids[index] = pt.id;
      index++;
    }
  }

  // Check to make sure all of our vectors have the same number of points, otherwise, something
  // went wrong
  unsigned int num_pts_cam0_t0 = pts_cam0_t0.size();
  if (num_pts_cam0_t0 != pts_cam0_t1.size() ||
      num_pts_cam0_t0 != pts_cam1_t0.size() ||
      num_pts_cam0_t0 != pts_cam1_t1.size()) {
    std::cerr << "Error! [ProcessPoints] points are not the same size!" << std::endl;
    return -1;
  } else if (pts_cam0_t0.size() == 0) {
    return -1;
  }

  if (num_pts_cam0_t0 < 6) {
    std::cout << "Not enough points for algorithm!" << std::endl;
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
  for (int i = 0; i < pts_cam0_t0.size(); i++) {
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
    camRotations);

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

  Eigen::Matrix4d delta_xform = Eigen::Matrix4d::Identity();
  // pose_ = pose_ * delta_xform ;
  // std::cout << "output transformation: \n" << pose_ << std::endl;

  int index_hist = SaveInliers(ransac.inliers_, pts_ids, points);
  std::cout << "index_hist: " << index_hist << std::endl;
  if (index_hist >= 0 && 0) {
    // // Get the points of the current timestep by triangulating 2D points in timestep t1
    // cv::Mat pts_cam0_t1_mat(pts_cam0_t1_ud); pts_cam0_t1_mat.convertTo(pts_cam0_t1_mat, CV_64F);
    // cv::Mat pts_cam1_t1_mat(pts_cam1_t1_ud); pts_cam1_t1_mat.convertTo(pts_cam1_t1_mat, CV_64F);
    // cv::Mat triangulation_output_pts_t1_homo;
    // cv::triangulatePoints(P0_, P1_, pts_cam0_t1_mat, pts_cam1_t1_mat,
    //   triangulation_output_pts_t1_homo);

    // // Convert points from homogeneous to 3D coords
    // std::vector<cv::Vec3d> triangulation_output_pts_t1;
    // cv::convertPointsFromHomogeneous(triangulation_output_pts_t1_homo.t(),
    //   triangulation_output_pts_t1);

    // // put the points into the opengv object
    // opengv::points_t pts_t1_temp(num_pts);
    // for(size_t i = 0; i < num_pts; i++) {
    //   cv::cv2eigen(triangulation_output_pts_t1[i], pts_t1_temp[i]);
    // }

    opengv::points_t pts_t0; pts_t0.reserve(2 * pts_ids.size());
    opengv::bearingVectors_t bvec_t1; bvec_t1.reserve(2 * pts_ids.size());
    std::vector<int> corr_t1; corr_t1.reserve(2 * pts_ids.size());
    for (unsigned int i = 0; i < pts_ids.size(); i++) {
      if (point_history_[index_hist].count(pts_ids[i])) {
        pts_t0.push_back(point_history_[index_hist][pts_ids[i]]);
        pts_t0.push_back(point_history_[index_hist][pts_ids[i]]);
        bvec_t1.push_back(bearing_vectors_cam0_t1[i]);
        corr_t1.push_back(0);
        bvec_t1.push_back(bearing_vectors_cam1_t1[i]);
        corr_t1.push_back(1);
      }
    }

    // Sanity check that we have enough points
    if (pts_t0.size() < min_num_matches) {
      std::cerr << "Error! Points should have at least " << min_num_matches << " matches!\n";
      std::cerr << "index: " << index_hist << "\n";
      std::exit(1);
    }

    // create the 3D-3D adapter
    // opengv::point_cloud::PointCloudAdapter adapter_pt_cloud(pts_t0, pts_t1);
    // // // run threept_arun
    // // opengv::transformation_t threept_transformation = opengv::point_cloud::optimize_nonlinear(
    // //   adapter_pt_cloud);

    // // create a RANSAC object
    // opengv::sac::Ransac<opengv::sac_problems::point_cloud::PointCloudSacProblem> ransac;
    // // create the sample consensus problem
    // std::shared_ptr<opengv::sac_problems::point_cloud::PointCloudSacProblem>
    //     relposeproblem_ptr(
    //     new opengv::sac_problems::point_cloud::PointCloudSacProblem(adapter_pt_cloud) );
    // // run ransac
    // ransac.sac_model_ = relposeproblem_ptr;
    // ransac.threshold_ = 0.1;
    // ransac.max_iterations_ = 500;
    // ransac.computeModel(0);
    // // return the result
    // opengv::transformation_t best_transformation =
    //     ransac.model_coefficients_;


    // Create a non-central absolute adapter
    opengv::absolute_pose::NoncentralAbsoluteAdapter adapter2(
      bvec_t1,
      corr_t1,
      pts_t0,
      camOffsets,
      camRotations);

    // Create the ransac model and compute
    opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac2;
    std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
      absposeproblem_ptr2(new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(adapter,
        opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P));
    ransac2.sac_model_ = absposeproblem_ptr2;
    ransac2.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/500.0));
    ransac2.max_iterations_ = 200;
    ransac2.computeModel();

    delta_xform.block<3,4>(0, 0) = ransac2.model_coefficients_;

    pose_ = pose_history_[index_hist] * delta_xform;
    std::cout << "index_hist " << index_hist << std::endl;
    std::cout << "\n " << pose_history_[index_hist] << std::endl;
  } else {
    // Put the ransc output pose update into a 4x4 Matrix
    delta_xform.block<3,4>(0, 0) = ransac.model_coefficients_;

    // Apply the user calibration
    delta_xform.block<3,1>(0, 3) -= vio_calibration_;

    // Calculate the updated pose
    pose_ = pose_ * delta_xform;
  }



  std::cout << "delta update: \n" << delta_xform << std::endl;
  std::cout << "output transformation: \n" << pose_ << std::endl;

  pose_history_[Modulo(static_cast<int>(point_history_index_), history_size)] = pose_;
  point_history_index_ = (point_history_index_ >= history_size - 1) ? 0 : point_history_index_ + 1;

  for (int i = 0; i < 3; i++) {
    position_update(i) = pose_(i, 3);
  }

  // Debug statements
  // // if (ransac.model_coefficients_(1,3) < -0.3) {
  // if (g++ == 99) {
  //   int p = 0;
  // }

  return 0;
}

int Vio::Debug_SaveOutput() {
  static int g = 0;
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

    if (!trajecotry_file_) {
      trajecotry_file_ = std::make_unique<std::ofstream> ("file.txt", std::ios::out);
    }
    Eigen::Matrix<double, 3, 4> writer_pose = pose_.block<3, 4> (0, 0);
    Eigen::Map<Eigen::RowVectorXd> v(writer_pose.data(), writer_pose.size());
    *trajecotry_file_ << v << kf_.GetState().transpose() << std::endl;

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