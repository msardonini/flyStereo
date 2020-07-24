
#include "fly_stereo/vio.h"

// System includes
#include <iostream>

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
    if (it->second.size() > max_pts_in_bin_) {
      unsigned int num_pts_delete = it->second.size() - max_pts_in_bin_;
      it->second.erase(it->second.end() - num_pts_delete, it->second.end());
    }   
  }

  return 0;
}

int Vio::CalculatePoseUpdate(const std::map<int, std::vector<ImagePoint> > &grid,
  Eigen::Matrix4d &pose_update) {
  // Calculate the number of points in the map
  unsigned int num_pts = 0;
  for (auto it = grid.begin(); it != grid.end(); it++) {
    num_pts += it->second.size();
  }
  // Define vectors to hold the points we will use for processing
  std::vector<cv::Point2f> pts_cam0_t0(num_pts);
  std::vector<cv::Point2f> pts_cam0_t1(num_pts);
  std::vector<cv::Point2f> pts_cam1_t0(num_pts);
  std::vector<cv::Point2f> pts_cam1_t1(num_pts);

  // Take our points out of the grid and put into vectors for processing
  unsigned int index = 0;
  for (auto it = grid.begin(); it != grid.end(); it++) {
    for (const ImagePoint &pt : it->second) {
      pts_cam0_t0[index] = cv::Point2f(pt.cam0_t0[0], pt.cam0_t0[1]);
      pts_cam0_t1[index] = cv::Point2f(pt.cam0_t1[0], pt.cam0_t1[1]);
      pts_cam1_t0[index] = cv::Point2f(pt.cam1_t0[0], pt.cam1_t0[1]);
      pts_cam1_t1[index] = cv::Point2f(pt.cam1_t1[0], pt.cam1_t1[1]);
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

  // pts_cam0_t1.clear();
  // pts_cam0_t1.push_back(cv::Point2f(629, 397));
  // pts_cam0_t1.push_back(cv::Point2f(1162, 347));
  // pts_cam0_t1.push_back(cv::Point2f(625, 674));

  // Cast all of our paramters to floating points for this function
  cv::Matx33f K_cam0 = static_cast<cv::Matx33f>(K_cam0_);
  cv::Matx33f K_cam1 = static_cast<cv::Matx33f>(K_cam1_);
  // cv::Matx34f P_cam0 = static_cast<cv::Matx34f>(P_cam0_);
  // cv::Matx34f P_cam1 = static_cast<cv::Matx34f>(P_cam1_);
  // cv::Vec4f D_cam0 = static_cast<cv::Vec4f>(D_cam0_);
  // cv::Vec4f D_cam1 = static_cast<cv::Vec4f>(D_cam1_);

  std::vector<cv::Point2f> pts_cam0_t0_ud;
  std::vector<cv::Point2f> pts_cam0_t1_ud;
  std::vector<cv::Point2f> pts_cam1_t0_ud;
  std::vector<cv::Point2f> pts_cam1_t1_ud;

  cv::undistortPoints(pts_cam0_t0, pts_cam0_t0_ud, K_cam0, D_cam0_);
  cv::undistortPoints(pts_cam0_t1, pts_cam0_t1_ud, K_cam0, D_cam0_);
  cv::undistortPoints(pts_cam1_t0, pts_cam1_t0_ud, K_cam1, D_cam1_);
  cv::undistortPoints(pts_cam1_t1, pts_cam1_t1_ud, K_cam1, D_cam1_);

  std::vector<cv::Vec3f> xform_vec_cam0_t0;
  std::vector<cv::Vec3f> xform_vec_cam0_t1;
  std::vector<cv::Vec3f> xform_vec_cam1_t0;
  std::vector<cv::Vec3f> xform_vec_cam1_t1;

  cv::convertPointsToHomogeneous(pts_cam0_t0_ud, xform_vec_cam0_t0);
  cv::convertPointsToHomogeneous(pts_cam0_t1_ud, xform_vec_cam0_t1);
  cv::convertPointsToHomogeneous(pts_cam1_t0_ud, xform_vec_cam1_t0);
  cv::convertPointsToHomogeneous(pts_cam1_t1_ud, xform_vec_cam1_t1);

  opengv::bearingVectors_t bearing_vectors_cam0_t0;
  opengv::bearingVectors_t bearing_vectors_cam0_t1;
  opengv::bearingVectors_t bearing_vectors_cam1_t0;
  opengv::bearingVectors_t bearing_vectors_cam1_t1;
  std::vector<int> cam_correspondences_cam0_t0;
  std::vector<int> cam_correspondences_cam0_t1;
  std::vector<int> cam_correspondences_cam1_t0;
  std::vector<int> cam_correspondences_cam1_t1;

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

  //Extract the relative pose
  opengv::translation_t position = Eigen::Vector3d::Zero(); opengv::rotation_t rotation = Eigen::
    Matrix3d::Identity();

  Eigen::MatrixXd cam1_offset;
  Eigen::Matrix3d cam1_rot;
  cv::cv2eigen(T_cam0_cam1_, cam1_offset);
  cv::cv2eigen(R_cam0_cam1_, cam1_rot);

  opengv::relative_pose::CentralRelativeAdapter adapter(
    bearing_vectors_cam0_t0,
    bearing_vectors_cam1_t0,
    cam1_offset,
    cam1_rot);

  Eigen::MatrixXd triangulate_results(3, num_pts);
  std::cout << "Triangulation output! \n";
  
  opengv::points_t points(num_pts);
  for(size_t i = 0; i < num_pts; i++) {
    points[i] = opengv::triangulation::triangulate(adapter, i);
    // std::cout << triangulate_results.block<3,1>(0, i).transpose() << std::endl;
  }


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

  opengv::translations_t camOffsets;
  opengv::rotations_t camRotations;

  camOffsets.push_back(Eigen::Vector3d::Zero());
  camRotations.push_back(Eigen::Matrix3d::Identity());

  Eigen::Matrix3d R; cv::cv2eigen(R_cam0_cam1_, R);
  Eigen::MatrixXd T; cv::cv2eigen(T_cam0_cam1_, T);
  camRotations.push_back(R);
  camOffsets.push_back(T);

  //create a non-central absolute adapter
  opengv::absolute_pose::NoncentralAbsoluteAdapter adapter2(
    bearing_vectors_t1,
    correspondences_t1,
    points,
    camOffsets,
    camRotations);


  opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
  std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
    new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(adapter2,
      opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::GAO));
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/8000.0));
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
    std::cout << ransac.inliers_[i] << " ";
  std::cout << std::endl << std::endl;

  Eigen::Matrix4d delta_xform = Eigen::Matrix4d::Identity();
  delta_xform.block<3,4>(0, 0) = ransac.model_coefficients_;
  pose_ = pose_ * delta_xform ;
  std::cout << "output transformation: \n" << pose_ << std::endl;

  if (ransac.model_coefficients_(0,3) > 100 || ransac.model_coefficients_(1,3) > 100 || ransac.model_coefficients_(2,3) > 100) {
    int p = 0;
  }

 /************** perform the triangulation with openCV *******************************/

  cv::Mat tmp_cam0(pts_cam0_t0_ud);
  cv::Mat tmp_cam1(pts_cam1_t0_ud);
  cv::Mat output;

  cv::Matx34d P0 = cv::Matx34d::eye();
  cv::Matx34d P1 = cv::Matx34d::zeros(); 
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P1(i, j) = R_cam0_cam1_(i, j);
    }
    P1(i, 3) = T_cam0_cam1_[i];
  }

  cv::triangulatePoints(P0, P1, tmp_cam0, tmp_cam1, output);

  std::vector<cv::Vec3f> vec;
  cv::convertPointsFromHomogeneous(output.t(), vec);


  static int g = 0;
  if (g == 100) {
    for (int i = 0; i < points.size(); i++) {
      // ofs  << vec[i](0) << "," << vec[i](1) << "," << vec[i](2) << std::endl;
      // ofs  << points[i].transpose() << std::endl;
    }
    // std::exit(0);
  }
  else 
    g++;


  // //create non-central relative adapter
  // opengv::relative_pose::NoncentralRelativeAdapter adapter(
  //   bearing_vectors0,
  //   bearing_vectors1,
  //   cam_correspondences0,
  //   cam_correspondences1,
  //   camOffsets,
  //   camRotations);
  //   position,
  //   rotation);

  // std::vector<int> indices17;
  // for (int i = 0; i < 6; i++)
  //   indices17.push_back(i);

  // opengv::transformation_t seventeenpt_transformation_all;
  // opengv::rotations_t rotations;
  // opengv::geOutput_t output;
  // for(size_t i = 0; i < 1; i++) {
  //   // std::cout << "i: " << i << std::endl;
  //   // rotations = opengv::relative_pose::sixpt(adapter, indices17);
  //   // seventeenpt_transformation_all = opengv::relative_pose::optimize_nonlinear(adapter);
  //   seventeenpt_transformation_all = opengv::relative_pose::seventeenpt(adapter);

  //   // generalized eigensolver (over all points)
  //   // opengv::relative_pose::ge(adapter, output);

  // }

  // Eigen::Matrix4d delta_xform = Eigen::Matrix4d::Identity();
  // delta_xform.block<3,3>(0, 0) = output.rotation;
  // Eigen::Vector3d tmp = output.translation.block<3,1>(0,0);
  // delta_xform.block<3,1>(0, 3) = tmp;
  // Update our position & rotation
  // delta_xform.block<3,4>(0, 0) = seventeenpt_transformation_all;
  

  // pose_ = pose_ * delta_xform ;


  // std::cout << "output transformation: \n" << pose_ << std::endl;

  // prev_xform_ = delta_xform;


  // cv::Mat tmp_cam0(pts_cam0_t1);
  // cv::Mat tmp_cam1(pts_cam1_t1);
  // cv::Mat output;

  // cv::triangulatePoints(P_cam0, P_cam1, tmp_cam0, tmp_cam1, output);

  // std::vector<cv::Vec3f> vec;
  // cv::convertPointsFromHomogeneous(output.t(), vec);


  // // Save the output ID / 3D position in the global map
  // unsigned int prev_pts_index = curr_pts_index_ ? 0 : 1;

  // points_3d_[curr_pts_index_].clear();
  // for (int i = 0; i < vec.size(); i++) {
  //   points_3d_[curr_pts_index_][ids[i]] = vec[i];
  // }

  // // Calculate the deltas from the last set of points to the current
  // unsigned int counter = 0;
  // cv::Vec3f diffs(0, 0.0, 0.0);
  // std::vector<cv::Vec3d> diffs_vec; diffs_vec.reserve(points_3d_[curr_pts_index_].size());
  // for (std::map<unsigned int, cv::Vec3f>::iterator it = points_3d_[curr_pts_index_].begin();
  //   it != points_3d_[curr_pts_index_].end(); it++ ) {

  //   std::map<unsigned int, cv::Vec3f>::iterator it_prev = points_3d_[prev_pts_index].find(
  //     it->first);
  //   if (it_prev != points_3d_[prev_pts_index].end()) {
  //     diffs += (it->second - it_prev->second);
  //     counter++;
  //     // std::cout << "diffs " << (it->second - it_prev->second) << std::endl;
  //     diffs_vec.push_back(it->second - it_prev->second);
  //   }
  // }

  // // std::cout << "size of vec: " << vec.size() << std::endl;
  // if (vec.size() > 0) {

  //   if (diffs_vec.size() > 0) {
  //     cv::Vec3d mean, std_dev;
  //     cv::Mat mat_diffs(diffs_vec.size(), 1, CV_64FC3, diffs_vec.data());
  //     cv::meanStdDev(mat_diffs, mean, std_dev);

  //     // Delete points that are not within 2 standard deviations of the mean
  //     for (int i = diffs_vec.size() - 1; i >= 0; i--) {
  //       if (diffs_vec[i][0] < mean(0) - 2 * std_dev(0) || 
  //           diffs_vec[i][0] > mean(0) + 2 * std_dev(0) ||
  //           diffs_vec[i][1] < mean(1) - 2 * std_dev(1) ||
  //           diffs_vec[i][1] > mean(1) + 2 * std_dev(1) ||
  //           diffs_vec[i][2] < mean(2) - 2 * std_dev(2) ||
  //           diffs_vec[i][2] > mean(2) + 2 * std_dev(2)) {
  //         diffs_vec.erase(diffs_vec.begin() + i);
  //       }
  //     }

  //   }
  //   if (diffs_vec.size() > 0) {
  //     cv::Vec3d mean2, std_dev;
  //     cv::Mat mat_diffs2(diffs_vec.size(), 1, CV_64FC3, diffs_vec.data());
  //     cv::meanStdDev(mat_diffs2, mean2, std_dev);
  //     vio_sum_ += mean2;
  //   }

  //   std::cout << "vio X,Y,Z: " << vio_sum_ << std::endl;
  // }

  // // Set the current index to the previous index for the next iteration
  // curr_pts_index_ = prev_pts_index;
  return 0;
}

