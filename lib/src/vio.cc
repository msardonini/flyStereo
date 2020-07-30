
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



// Constructor with config params
Vio::Vio(const YAML::Node &input_params, const YAML::Node &stereo_calibration) {
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

  pose_ = Eigen::Matrix4d::Identity();
}

// Destructor
Vio::~Vio() {}

int Vio::ProcessPoints(const ImagePoints &pts, Eigen::Vector3d &pose) {
  // All points are placed in bins, sections of the image. The key to this map is the row-major
  // index of the bin. The value is a vector of image points, which holds points from both cameras
  // in the current frame and the previos one
  std::map<int, std::vector<ImagePoint> > grid;

  BinFeatures(pts, grid);

  Eigen::Matrix4d pose_update;
  CalculatePoseUpdate(grid, pose_update);
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

int Vio::CalculatePoseUpdate(const std::map<int, std::vector<ImagePoint> > &grid,
  Eigen::Matrix4d &pose_update) {
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

  // Take our points out of the grid and put into vectors for processing
  unsigned int index = 0;
  for (auto it = grid.begin(); it != grid.end(); it++) {
    for (const ImagePoint &pt : it->second) {
      pts_cam0_t0[index] = cv::Point2d(pt.cam0_t0[0], pt.cam0_t0[1]);
      pts_cam0_t1[index] = cv::Point2d(pt.cam0_t1[0], pt.cam0_t1[1]);
      pts_cam1_t0[index] = cv::Point2d(pt.cam1_t0[0], pt.cam1_t0[1]);
      pts_cam1_t1[index] = cv::Point2d(pt.cam1_t1[0], pt.cam1_t1[1]);
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
    return 0;
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
  ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/800.0));
  ransac.max_iterations_ = 200;
  ransac.computeModel();

  //print the results
  std::cout << "the ransac results is: " << std::endl;
  std::cout << ransac.model_coefficients_ << std::endl << std::endl;
  std::cout << "Ransac needed " << ransac.iterations_ << " iterations and ";
  // std::cout << ransac_time << " seconds" << std::endl << std::endl;
  std::cout << "the number of inliers is: " << ransac.inliers_.size();
  std::cout << std::endl << std::endl;
  std::cout << "the found inliers are: " << std::endl;
  for(size_t i = 0; i < ransac.inliers_.size(); i++)
    std::cout << points[ransac.inliers_[i]].transpose() << "\n";
  std::cout << std::endl << std::endl;

  Eigen::Matrix4d delta_xform = Eigen::Matrix4d::Identity();
  delta_xform.block<3,4>(0, 0) = ransac.model_coefficients_;
  pose_ = pose_ * delta_xform ;
  std::cout << "output transformation: \n" << pose_ << std::endl;


  // Debug statements
  // static int g = 0;
  // // if (ransac.model_coefficients_(1,3) < -0.3) {
  // if (g++ == 99) {
  //   int p = 0;
  // }

  // // if (ransac.model_coefficients_(0,3) > 5 || ransac.model_coefficients_(1,3) > 5 || ransac.model_coefficients_(2,3) > 5) {
  // if (1) {

  //   if (!ofs_) {
  //     ofs_ = std::make_unique<std::ofstream> ("file.txt", std::ios::out);
  //   }
  //   Eigen::Map<Eigen::RowVectorXd> v(ransac.model_coefficients_.data(), ransac.model_coefficients_.size());
  //   *ofs_ << v << std::endl;

  //   // for (int i = 0; i < points.size(); i++) {
  //   //   // ofs  << vec[i](0) << "," << vec[i](1) << "," << vec[i](2) << std::endl;
  //   //   ofs  << points[i].transpose() << std::endl;
  //   // }
  //   // std::exit(1);
  // }
  return 0;
}

