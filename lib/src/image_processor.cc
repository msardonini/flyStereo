
#include <random>

#include "fly_stereo/image_processor.h"

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/core/eigen.hpp"
#include "Eigen/Dense"
#include "opengv/types.hpp"
#include "opengv/relative_pose/methods.hpp"
#include "opengv/relative_pose/NoncentralRelativeAdapter.hpp"

ImageProcessor::ImageProcessor(const YAML::Node &input_params) {
  // Params for OpenCV function goodFeaturesToTrack
  YAML::Node features_params = input_params["goodFeaturesToTrack"];
  max_corners_ = features_params["max_corners"].as<int>();
  quality_level_ = features_params["quality_level"].as<float>();
  min_dist_ = features_params["min_dist"].as<float>();

  // Params for OpenCV function calcOpticalFlowPyrLK
  YAML::Node opticall_flow_params = input_params["calcOpticalFlowPyrLK"];
  window_size_ = features_params["max_corners"].as<int>();
  max_pyramid_level_ = features_params["max_corners"].as<int>();

  max_error_counter_ = input_params["max_error_counter"].as<int>();
  stereo_threshold_ = input_params["stereo_threshold"].as<double>();
  ransac_threshold_ = input_params["ransac_threshold"].as<double>();

  // Load the stereo calibration
  YAML::Node stereo_calibration = input_params["stereo_calibration"];

  std::vector<double> interface_vec = stereo_calibration["K1"]["data"].as<
    std::vector<double>>();
  K_cam0_ = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["K2"]["data"].as<std::vector<double>>();
  K_cam1_ = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["D1"]["data"].as<std::vector<double>>();
  D_cam0_ = cv::Vec4d(interface_vec.data());

  interface_vec = stereo_calibration["D2"]["data"].as<std::vector<double>>();
  D_cam1_ = cv::Vec4d(interface_vec.data());

  interface_vec = stereo_calibration["R"]["data"].as<std::vector<double>>();
  R_cam0_cam1_ = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["T"].as<std::vector<double>>();
  T_cam0_cam1_ = cv::Vec3d(interface_vec.data());

  // Calculate the essential matrix and fundamental matrix
  const cv::Matx33d T_cross_mat(
    0.0, -T_cam0_cam1_[2], T_cam0_cam1_[1],
    T_cam0_cam1_[2], 0.0, -T_cam0_cam1_[0],
    -T_cam0_cam1_[1], T_cam0_cam1_[0], 0.0);

  E_ = T_cross_mat * R_cam0_cam1_;
  F_ = K_cam0_.inv().t() * E_ * K_cam1_.inv();

  interface_vec = stereo_calibration["R1"]["data"].as<std::vector<double>>();
  R_cam0_ = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["R2"]["data"].as<std::vector<double>>();
  R_cam1_ = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["P1"]["data"].as<std::vector<double>>();
  P_cam0_ = cv::Matx34d(interface_vec.data());

  interface_vec = stereo_calibration["P2"]["data"].as<std::vector<double>>();
  P_cam1_ = cv::Matx34d(interface_vec.data());

  interface_vec = stereo_calibration["Q"]["data"].as<std::vector<double>>();
  Q_ = cv::Matx44d(interface_vec.data());


  std::vector<float> imu_cam0_vec = stereo_calibration["debug_vec"].as<std::vector<float>>();
  cv::Vec3f angles_imu_cam0 = {imu_cam0_vec[0], imu_cam0_vec[1], imu_cam0_vec[2]};
  cv::Rodrigues(angles_imu_cam0, R_imu_cam0_);
  cv::Matx33f R_cam0_cam1_f = R_cam0_cam1_;
  R_imu_cam1_ = R_imu_cam0_ * R_cam0_cam1_f;

  std::cout << R_imu_cam0_ << std::endl;

  pose_ = Eigen::Matrix4d::Identity();

  // Make a copy of our configuration for later use
  input_params_ = input_params;
}

ImageProcessor::~ImageProcessor() {
  is_running_.store(false);

  if (image_processor_thread_.joinable()) {
    image_processor_thread_.join();
  }
}

int ImageProcessor::Init() {
  sensor_interface_ = std::make_unique<SensorInterface>();
  sensor_interface_->Init(input_params_);

  is_running_.store(true);
  image_processor_thread_ = std::thread(&ImageProcessor::ProcessThread, this);
}


int ImageProcessor::RemoveOutliers(const std::vector<uchar> &status,
  std::vector<cv::Point2f> &points) {
  if (status.size() != points.size()) {
    std::cerr << "Error! status and points are not the same size" << std::endl;
    return -1;
  }

  for (int i = status.size() - 1; i >= 0; i--) {
    if (status[i] == 0) {
      points.erase(points.begin() + i);
    }
  }
  return 0;
}

//TODO: Make this function templated
int ImageProcessor::RemoveOutliers(const std::vector<uchar> &status,
  std::vector<unsigned int> &points) {
  if (status.size() != points.size()) {
    std::cerr << "Error! status and points are not the same size" << std::endl;
    return -1;
  }

  for (int i = status.size() - 1; i >= 0; i--) {
    if (status[i] == 0) {
      points.erase(points.begin() + i);
    }
  }
  return 0;
}

int ImageProcessor::RemoveOutliers(const cv::cuda::GpuMat &d_status, cv::cuda::GpuMat &d_points) {
  std::vector<cv::Point2f> points;
  std::vector<unsigned char> status;

  if (d_points.cols != 0) {
    d_points.download(points);
  } else {
    return -1;
  }
  if (d_status.cols != 0) {
    d_status.download(status);
  } else {
    return -1;
  }

  if (RemoveOutliers(status, points)) {
    return -1;
  }

  d_points.upload(points);
  return 0;
}

int ImageProcessor::RemovePointsOutOfFrame(const cv::Size framesize,
  const std::vector<cv::Point2f> &points, std::vector<unsigned char> &status) {
  if (status.size() != points.size()) {
    std::cerr << "Error! status and points are not the same size" << std::endl;
    return -1;
  }

  // Mark those tracked points out of the image region
  // as untracked.
  for (int i = 0; i < points.size(); ++i) {
    if (status[i] == 0) {
      continue;
    }
    if (points[i].y < 0 || points[i].y > framesize.height - 1 || points[i].x < 0 ||
        points[i].x > framesize.width - 1) {
      status[i] = 0;
    }
  }
  return 0;
}

int ImageProcessor::RemovePointsOutOfFrame(const cv::Size framesize,
  const cv::cuda::GpuMat &d_points, cv::cuda::GpuMat &d_status) {
  std::vector<cv::Point2f> points;
  std::vector<unsigned char> status;

  if (d_points.cols != 0) {
    d_points.download(points);
  } else {
    return -1;
  }
  if (d_status.cols != 0) {
    d_status.download(status);
  } else {
    return -1;
  }

  if (RemovePointsOutOfFrame(framesize, points, status)) {
    return -1;
  }

  d_status.upload(status);
  return 0;
}

cv::cuda::GpuMat ImageProcessor::AppendGpuMatColwise(
  const cv::cuda::GpuMat &mat1, const cv::cuda::GpuMat &mat2) {
  // Handle edge cases where one of the input mats is empty
  if (mat1.cols == 0) {
    if (mat2.cols == 0) {
      return cv::cuda::GpuMat();
    } else {
      return mat2.clone();
    }
  } else if (mat2.cols == 0) {
    return mat1.clone();
  }

  // Verify the two mats are the same data type
  if (mat1.type() != mat2.type()) {
    std::cerr << "Error! [AppendGpuMat] Mats are not the same type" <<
      std::endl;
    return cv::cuda::GpuMat();
  } else if (mat1.rows != mat2.rows) {
    std::cerr << "Error! [AppendGpuMat] Mats do not have the same amount of"
      " rows" << std::endl;
    return cv::cuda::GpuMat();
  }

  cv::Range range_rows(0, mat1.rows);
  cv::Range range_cols1(0, mat1.cols);
  cv::Range range_cols2(mat1.cols, mat1.cols + mat2.cols);

  cv::cuda::GpuMat return_mat(mat1.rows, mat1.cols + mat2.cols, mat1.type());

  mat1.copyTo(return_mat(range_rows, range_cols1));
  mat2.copyTo(return_mat(range_rows, range_cols2));

  return std::move(return_mat);
}


int ImageProcessor::UpdatePointsViaImu(const std::vector<cv::Point2f> &current_pts,
  const cv::Matx33d &rotation,
  const cv::Matx33d &camera_matrix,
  std::vector<cv::Point2f> &updated_pts) {
  if (current_pts.size() == 0) {
    return -1;
  }

  cv::Matx33f H = camera_matrix * rotation * camera_matrix.inv();

  updated_pts.resize(current_pts.size());
  for (int i = 0; i < current_pts.size(); i++) {
    cv::Vec3f temp_current(current_pts[i].x, current_pts[i].y, 1.0f);
    cv::Vec3f temp_updated = H * temp_current;
    updated_pts[i].x = temp_updated[0] / temp_updated[2];
    updated_pts[i].y = temp_updated[1] / temp_updated[2];
  }
  return 0;
}

int ImageProcessor::UpdatePointsViaImu(const cv::cuda::GpuMat &d_current_pts,
  const cv::Matx33d &rotation,
  const cv::Matx33d &camera_matrix,
  cv::cuda::GpuMat &d_updated_pts) {
  if (d_current_pts.cols == 0) {
    return -1;
  }

  std::vector<cv::Point2f> current_pts;
  d_current_pts.download(current_pts);
  std::vector<cv::Point2f> updated_pts;

  int ret = UpdatePointsViaImu(current_pts, rotation, camera_matrix, updated_pts);
  d_updated_pts.upload(updated_pts);
  return ret;
}

int ImageProcessor::ProcessPoints(std::vector<cv::Point2f> pts_cam0_t0, std::vector<cv::Point2f>
  pts_cam0_t1, std::vector<cv::Point2f> pts_cam1_t0, std::vector<cv::Point2f> pts_cam1_t1,
  std::vector<unsigned int> ids) {

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

  if (num_pts_cam0_t0 < 17) {
    std::cout << "Not enough points for algorithm!" << std::endl;
    return 0;
  }

  // Cast all of our paramters to floating points for this function
  cv::Matx33f K_cam0 = static_cast<cv::Matx33f>(K_cam0_);
  cv::Matx33f K_cam1 = static_cast<cv::Matx33f>(K_cam1_);
  cv::Matx34f P_cam0 = static_cast<cv::Matx34f>(P_cam0_);
  cv::Matx34f P_cam1 = static_cast<cv::Matx34f>(P_cam1_);
  cv::Vec4f D_cam0 = static_cast<cv::Vec4f>(D_cam0_);
  cv::Vec4f D_cam1 = static_cast<cv::Vec4f>(D_cam1_);

  cv::fisheye::undistortPoints(pts_cam0_t0, pts_cam0_t0, K_cam0, D_cam0, R_cam0_, P_cam0);
  cv::fisheye::undistortPoints(pts_cam0_t1, pts_cam0_t1, K_cam0, D_cam0, R_cam0_, P_cam0);
  cv::fisheye::undistortPoints(pts_cam1_t0, pts_cam1_t0, K_cam1, D_cam1, R_cam1_, P_cam1);
  cv::fisheye::undistortPoints(pts_cam1_t1, pts_cam1_t1, K_cam1, D_cam1, R_cam1_, P_cam1);

  opengv::bearingVectors_t bearing_vectors0;
  opengv::bearingVectors_t bearing_vectors1;
  std::vector<int> cam_correspondences0;
  std::vector<int> cam_correspondences1;

  for (int i = 0; i < pts_cam0_t0.size(); i++) {
    opengv::bearingVector_t tmp_cam0;
    tmp_cam0 << (pts_cam0_t0[i].x -  K_cam0_(0, 2)) / K_cam0_(0, 0), (pts_cam0_t0[i].y - K_cam0_(1, 2)) / K_cam0_(1, 1), 1;
    tmp_cam0 /= tmp_cam0.norm();
    bearing_vectors0.push_back(tmp_cam0);
    cam_correspondences0.push_back(0);

    // bearingVector_t tmp_cam0;
    tmp_cam0 << (pts_cam0_t1[i].x - K_cam0_(0, 2)) / K_cam0_(0, 0), (pts_cam0_t1[i].y -
      K_cam0_(1, 2)) / K_cam0_(1, 1), 1;
    tmp_cam0 /= tmp_cam0.norm();
    bearing_vectors1.push_back(tmp_cam0);
    cam_correspondences1.push_back(0);

    opengv::bearingVector_t tmp_cam1;
    tmp_cam1 << (pts_cam1_t0[i].x - K_cam1_(0, 2)) / K_cam1_(0, 0), (pts_cam1_t0[i].y - K_cam1_(1, 2)) / K_cam1_(1, 1), 1;
    tmp_cam1 /= tmp_cam1.norm();
    bearing_vectors0.push_back(tmp_cam1);
    cam_correspondences0.push_back(1);

    tmp_cam1 << (pts_cam1_t1[i].x - K_cam1_(0, 2)) / K_cam1_(0, 0), (pts_cam1_t1[i].y - K_cam1_(1, 2)) / K_cam1_(1, 1), 1;
    tmp_cam1 /= tmp_cam1.norm();
    bearing_vectors1.push_back(tmp_cam1);
    cam_correspondences1.push_back(1);
  }

  //Extract the relative pose
  opengv::translation_t position = Eigen::Vector3d::Zero(); opengv::rotation_t rotation = Eigen::Matrix3d::Identity();

  opengv::translations_t camOffsets;
  opengv::rotations_t camRotations;

  camOffsets.push_back(Eigen::Vector3d::Zero());
  camRotations.push_back(Eigen::Matrix3d::Identity());

  Eigen::Matrix3d R; cv::cv2eigen(R_cam0_cam1_, R);
  Eigen::MatrixXf T; cv::cv2eigen(T_cam0_cam1_, T);
  camRotations.push_back(R);
  camOffsets.push_back(T.cast<double>());

  //create non-central relative adapter
  opengv::relative_pose::NoncentralRelativeAdapter adapter(
    bearing_vectors0,
    bearing_vectors1,
    cam_correspondences0,
    cam_correspondences1,
    camOffsets,
    camRotations);
    // position,
    // rotation);

  std::vector<int> indices17;
  for (int i = 0; i < 6; i++)
    indices17.push_back(i);

  opengv::transformation_t seventeenpt_transformation_all;
  opengv::rotations_t rotations;
  for(size_t i = 0; i < 1; i++) {
    // std::cout << "i: " << i << std::endl;
    // rotations = opengv::relative_pose::sixpt(adapter, indices17);
    // seventeenpt_transformation_all = opengv::relative_pose::optimize_nonlinear(adapter);
    seventeenpt_transformation_all = opengv::relative_pose::seventeenpt(adapter);
  }

  // Update our position & rotation
  Eigen::Matrix4d delta_xform = Eigen::Matrix4d::Identity();
  delta_xform.block<3,4>(0, 0) = seventeenpt_transformation_all;
  pose_ = delta_xform * pose_;

  std::cout << "output transformation: \n" << pose_ << std::endl;

  // cv::Mat tmp_cam0(pts_cam0);
  // cv::Mat tmp_cam1(pts_cam1);
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
  //     std::cout << "diffs " << (it->second - it_prev->second) << std::endl;
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

int ImageProcessor::OuputTrackedPoints(std::vector<cv::Point2f> pts_cam0,
    std::vector<cv::Point2f> pts_cam1, std::vector<unsigned int> ids) {
  // Check to make sure that our input data is sized correctly
  unsigned int num_pts = pts_cam0.size();
  if (num_pts != pts_cam1.size() || num_pts != ids.size()) {
    std::cerr << "Error! Vectors do not match in OutputTrackedPoints" << std::endl;
    return -1;
  }

  // Acquire access to shared data, then write to it
  std::lock_guard<std::mutex> lock(output_mutex_);
  output_points_.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  output_points_.pts.clear();
  output_points_.pts.reserve(num_pts);

  for (int i = 0; i < num_pts; i++) {
    ImagePoint pt(ids[i], {pts_cam0[i].x, pts_cam0[i].y}, {pts_cam1[i].x, pts_cam1[i].y});
    output_points_.pts.push_back(pt);
  }

  output_cond_var_.notify_one();
  return 0;
}

bool ImageProcessor::GetTrackedPoints(ImagePoints *usr_pts) {
  std::unique_lock<std::mutex> lock(output_mutex_);
  std::cv_status ret = output_cond_var_.wait_for(lock, std::chrono::seconds(1));

  // Return false if we have hit our timeout
  if (ret == std::cv_status::timeout) {
    return false;
  }

  *usr_pts = output_points_;
  return true;
}

int ImageProcessor::ProcessThread() {
  // Vector of imu messages
  std::vector<mavlink_imu_t> imu_msgs;

  // Frames of the left and right camera
  cv::cuda::GpuMat d_frame_cam0_t1, d_frame_cam1_t1;
  cv::cuda::GpuMat d_frame_cam0_t0, d_frame_cam1_t0;

  // Arrays of points for tracking, cpu and gpu
  cv::cuda::GpuMat d_tracked_pts_cam0_t0, d_tracked_pts_cam0_t1;
  cv::cuda::GpuMat d_tracked_pts_cam1_t0, d_tracked_pts_cam1_t1;
  std::vector<cv::Point2f> tracked_pts_cam0_t0, tracked_pts_cam0_t1;
  std::vector<cv::Point2f> tracked_pts_cam1_t0, tracked_pts_cam1_t1;

  // Arrays of detected points for potential tracking, cpu and gpu
  cv::cuda::GpuMat d_keypoints_cam0_t0, d_keypoints_cam0_t1;
  cv::cuda::GpuMat d_keypoints_cam1_t0, d_keypoints_cam1_t1;
  std::vector<cv::Point2f> keypoints_cam0_t0, keypoints_cam0_t1;
  std::vector<cv::Point2f> keypoints_cam1_t0, keypoints_cam1_t1;

  // Arrays of the point id's, each feature point will be assigned a unique ID which will remain
  // constant over time until the feature is gone
  std::vector<unsigned int> ids_detected_t0, ids_detected_t1;
  std::vector<unsigned int> ids_tracked_t0, ids_tracked_t1;
  unsigned int current_id = 0;

  // Outputs for the calcOpticalFlowPyrLK call
  std::vector<unsigned char> status;
  cv::cuda::GpuMat d_status;

  // Flag telling if we should draw points to the ouput image frame
  bool draw_points_to_frame = true;
  if (input_params_["draw_points_to_frame"]) {
    draw_points_to_frame = input_params_["draw_points_to_frame"].as<bool>();
  }

  int counter = 0;
  int error_counter = 0;
  int debug_num_pts[4];
  std::chrono::time_point<std::chrono::system_clock> time_end = std::chrono::
    system_clock::now();

  cv::Ptr<cv::cuda::CornersDetector> detector_ptr = cv::cuda::createGoodFeaturesToTrackDetector(
    CV_8U, max_corners_, quality_level_, min_dist_);

  // cv::Ptr<cv::cuda::FastFeatureDetector> detector_ptr = cv::cuda::FastFeatureDetector::create();

  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_opt_flow_cam0 = cv::cuda::
    SparsePyrLKOpticalFlow::create(cv::Size(21, 21), 3, 30, true);
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_opt_flow_cam1 = cv::cuda::
    SparsePyrLKOpticalFlow::create(cv::Size(21, 21), 3, 30, false);
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_opt_flow_cam1_temp = cv::cuda::
    SparsePyrLKOpticalFlow::create(cv::Size(21, 21), 3, 30, true);

  while (is_running_.load()) {
    // Trigger the camera to start acquiring an image

    // Read the frame and check for errors
    if (sensor_interface_->GetSynchronizedData(d_frame_cam0_t1, d_frame_cam1_t1, imu_msgs)) {
      if (++error_counter >= max_error_counter_) {
        std::cerr << "Error! Failed to read more than " << max_error_counter_
          << " frames" << std::endl;
        is_running_.store(false);
        return -1;
      } else {
        continue;
      }
    }

    uint64_t timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::
      steady_clock::now().time_since_epoch()).count();

    cv::Matx33f rotation_t0_t1_cam0;
    cv::Matx33f rotation_t0_t1_cam1;
    int retval = sensor_interface_->GenerateImuXform(imu_msgs, R_imu_cam0_, R_imu_cam1_,
      rotation_t0_t1_cam0, rotation_t0_t1_cam1);

    std::cout << rotation_t0_t1_cam0 << std::endl;
    std::cout << imu_msgs.size() << std::endl;

    // if (retval == 0) {
    //   // cv::cuda::GpuMat tmp;
    //   std::cout << "running UpdatePointsViaImu \n";
    //   if(UpdatePointsViaImu(d_debug_pts, rotation_t0_t1_cam0, K_cam0_, d_debug_pts)) {
    //     std::cout << "Error in UpdatePointsViaImu \n";
    //   }
    //   // d_debug_pts = tmp;
    // }


    std::cout << counter << std::endl;


    // Append the kepypoints from the previous iteration to the tracked points of the current
    // iteration. Points that pass through all the outlier checks will remain as tracked points
    d_tracked_pts_cam0_t0 = AppendGpuMatColwise(d_tracked_pts_cam0_t0, d_keypoints_cam0_t0);
    d_tracked_pts_cam1_t0 = AppendGpuMatColwise(d_tracked_pts_cam1_t0, d_keypoints_cam1_t0);
    ids_tracked_t0.insert(std::end(ids_tracked_t0), std::begin(ids_detected_t0), std::end(
      ids_detected_t0));
    ids_tracked_t1 = ids_tracked_t0;

    // Make a prediction of where the points will be in the current image given the IMU data
    if (retval == 0) {
      UpdatePointsViaImu(d_tracked_pts_cam0_t0, rotation_t0_t1_cam0, K_cam0_,
        d_tracked_pts_cam0_t1);
      UpdatePointsViaImu(d_tracked_pts_cam1_t0, rotation_t0_t1_cam1, K_cam1_,
        d_tracked_pts_cam1_t1);
    } else {
      std::cerr << "Error receiving IMU transform" << std::endl;
      continue;
    }

    debug_num_pts[0] = d_tracked_pts_cam0_t1.cols;
    /*********************************************************************
    * Apply the optical flow from the previous frame to the current frame
    *********************************************************************/
    if (counter++ != 0 && d_tracked_pts_cam0_t0.rows > 0) {
      d_opt_flow_cam0->calc(d_frame_cam0_t0, d_frame_cam0_t1, d_tracked_pts_cam0_t0,
        d_tracked_pts_cam0_t1, d_status);

      // Check if we have zero tracked points, this is a corner case and these variables need
      // to be reset to prevent errors in sizing, calc() does not return all zero length vectors
      if (d_tracked_pts_cam0_t1.cols == 0) {
        d_tracked_pts_cam0_t1.release();
        d_tracked_pts_cam0_t0.release();
        d_status.release();
      }
      if (RemovePointsOutOfFrame(d_frame_cam0_t1.size(), d_tracked_pts_cam0_t1, d_status)) {
        std::cout << "RemovePointsOutOfFrame failed after tracking\n" << std::endl;
        std::cout << "status size:" << d_status.size() << std::endl;
        std::cout << "points size:" << d_tracked_pts_cam0_t1.size() << std::endl;
        return -1;
      }
      // TODO: Make this function accept a vector (non gpu) for first arg and templated vector as
      // the second argument
      d_status.download(status);
      if (RemoveOutliers(d_status, d_tracked_pts_cam0_t0) ||
          RemoveOutliers(d_status, d_tracked_pts_cam0_t1) ||
          RemoveOutliers(d_status, d_tracked_pts_cam1_t0) ||
          RemoveOutliers(status, ids_tracked_t1)) {
        std::cout << "RemoveOutliers failed after Tracking\n" << std::endl;
        return -1;
      }
      debug_num_pts[1] = d_tracked_pts_cam0_t1.cols;

      // Match the points from camera 0 to camera 1
      int ret_val = StereoMatch(d_opt_flow_cam1, d_frame_cam0_t1, d_frame_cam1_t1,
         d_tracked_pts_cam0_t1, d_tracked_pts_cam1_t1, d_status);
      // Remove the outliers from the StereoMatch algorithm
      if (ret_val == 0) {
        if (RemovePointsOutOfFrame(d_frame_cam1_t1.size(), d_tracked_pts_cam1_t1, d_status)) {
          std::cout << "RemovePointsOutOfFrame failed after StereoMatch\n" << std::endl;
          return -1;
        }
        d_status.download(status);
        if (RemoveOutliers(d_status, d_tracked_pts_cam0_t0) ||
            RemoveOutliers(d_status, d_tracked_pts_cam0_t1) ||
            RemoveOutliers(d_status, d_tracked_pts_cam1_t0) ||
            RemoveOutliers(d_status, d_tracked_pts_cam1_t1) ||
            RemoveOutliers(status, ids_tracked_t1)) {
          std::cout << "RemoveOutliers failed after StereoMatch\n" << std::endl;
          std::cout << "Sizes: " << d_status.cols << ", " << d_tracked_pts_cam0_t0.cols << ", " <<
            d_tracked_pts_cam0_t1.cols << ", " << d_tracked_pts_cam1_t0.cols << ", " <<
            d_tracked_pts_cam1_t1.cols << ", " << std::endl;
          return -1;
        }
      }
      debug_num_pts[2] = d_tracked_pts_cam0_t1.cols;

      // Perform a check to make sure that all of our tracked vectors are the same length,
      // otherwise something is wrong
      int tracked_pts = d_tracked_pts_cam0_t0.cols;
      if (tracked_pts != d_tracked_pts_cam0_t1.cols ||
          tracked_pts != d_tracked_pts_cam1_t0.cols ||
          tracked_pts != d_tracked_pts_cam1_t1.cols) {
        std::cerr << "ERROR! There are differences in the numbers of tracked points " << counter
          << std::endl;
        return -1;
      }
    }

    /*********************************************************************
    * Run the Two Point RANSAC algorithm to find more outliers
    *********************************************************************/
    if (d_tracked_pts_cam0_t0.cols > 1 && d_tracked_pts_cam1_t0.cols > 1) {
      d_tracked_pts_cam0_t0.download(tracked_pts_cam0_t0);
      d_tracked_pts_cam0_t1.download(tracked_pts_cam0_t1);

      std::vector<unsigned char> cam0_ransac_inliers(0);
      int ret1 = twoPointRansac(tracked_pts_cam0_t0, tracked_pts_cam0_t1, rotation_t0_t1_cam0,
        K_cam0_, D_cam0_, ransac_threshold_, 0.99, cam0_ransac_inliers);

      d_tracked_pts_cam1_t0.download(tracked_pts_cam1_t0);
      d_tracked_pts_cam1_t1.download(tracked_pts_cam1_t1);

      std::vector<unsigned char> cam1_ransac_inliers(0);
      int ret2 = twoPointRansac(tracked_pts_cam1_t0, tracked_pts_cam1_t1, rotation_t0_t1_cam1,
        K_cam1_, D_cam1_, ransac_threshold_, 0.99, cam1_ransac_inliers);

      if (ret1 == 0 && ret2 == 0) {
        // std::cout <<" before RANSAC " << tracked_pts_cam0_t1.size() << std::endl;
        std::vector<unsigned char> status;
        status.reserve(cam0_ransac_inliers.size());

        for (int i = 0; i < cam0_ransac_inliers.size(); i++ ) {
          status.push_back(cam0_ransac_inliers[i] && cam1_ransac_inliers[i]);
        }

        if (RemoveOutliers(status, tracked_pts_cam0_t0) ||
            RemoveOutliers(status, tracked_pts_cam0_t1) ||
            RemoveOutliers(status, tracked_pts_cam1_t0) ||
            RemoveOutliers(status, tracked_pts_cam1_t1) ||
            RemoveOutliers(status, ids_tracked_t1)) {
          std::cout << "RemoveOutliers failed after Ransac\n" << std::endl;
        }
        debug_num_pts[3] = tracked_pts_cam0_t1.size();
        // std::cout <<" after RANSAC " << tracked_pts_cam0_t1.size() << std::endl;
      }
    } else {
      if (d_tracked_pts_cam0_t1.cols > 0)
        d_tracked_pts_cam0_t1.download(tracked_pts_cam0_t1);
      else
        tracked_pts_cam0_t1.clear();
      if (d_tracked_pts_cam1_t1.cols > 0)
        d_tracked_pts_cam1_t1.download(tracked_pts_cam1_t1);
      else
        tracked_pts_cam1_t1.clear();
      if (d_tracked_pts_cam0_t0.cols > 0)
        d_tracked_pts_cam0_t0.download(tracked_pts_cam0_t0);
      else
        tracked_pts_cam0_t0.clear();
      if (d_tracked_pts_cam1_t0.cols > 0)
        d_tracked_pts_cam1_t0.download(tracked_pts_cam1_t0);
      else
        tracked_pts_cam1_t0.clear();
    }

    /*********************************************************************
    * Detect new features for the next iteration
    *********************************************************************/
    DetectNewFeatures(detector_ptr, d_frame_cam0_t1, d_tracked_pts_cam0_t1, d_keypoints_cam0_t1);
    // Match the detected features in the second camera
    d_keypoints_cam1_t1.release();
    StereoMatch(d_opt_flow_cam1_temp, d_frame_cam0_t1, d_frame_cam1_t1,
      d_keypoints_cam0_t1, d_keypoints_cam1_t1, d_status);
    if (d_keypoints_cam0_t1.cols != 0) {
      if (RemovePointsOutOfFrame(d_frame_cam0_t1.size(), d_keypoints_cam1_t1, d_status)) {
        std::cout << "RemovePointsOutOfFrame failed after detection\n" << std::endl;
        return -1;
      }
      if (RemoveOutliers(d_status, d_keypoints_cam0_t1) ||
          RemoveOutliers(d_status, d_keypoints_cam1_t1)) {
        std::cout << "RemoveOutliers failed after Detection\n" << std::endl;
        return -1;
      }
    }
    // Fill in the ids vector with our new detected features
    ids_detected_t1.clear();
    ids_detected_t1.reserve(d_keypoints_cam0_t1.cols);
    for (int i = 0; i < d_keypoints_cam0_t1.cols; i++) {
      ids_detected_t1.push_back(current_id++);
    }

    std::cout << "num points: cam0 " << d_tracked_pts_cam0_t1.cols << " cam1 " <<
      d_tracked_pts_cam1_t1.cols << " start: " << debug_num_pts[0] << " tracking: " <<
      debug_num_pts[1] << " matching: " << debug_num_pts[2] << " ransac: " << debug_num_pts[3]
      << std::endl;

    // Signal this loop has finished and output the tracked points
    if (tracked_pts_cam0_t1.size() > 1 && tracked_pts_cam1_t1.size() > 1) {
      // OuputTrackedPoints(tracked_pts_cam0_t1, tracked_pts_cam1_t1, ids_tracked_t1);
      ProcessPoints(tracked_pts_cam0_t0, tracked_pts_cam0_t1, tracked_pts_cam1_t0,
        tracked_pts_cam1_t1, ids_tracked_t1);
    }

    // If requested, output the video stream to the configured gstreamer pipeline
    if (sensor_interface_->cam0_->OutputEnabled()) {
      cv::Mat show_frame, show_frame_color;
      d_frame_cam0_t1.download(show_frame);
      if (draw_points_to_frame) {
        cv::cvtColor(show_frame, show_frame_color, cv::COLOR_GRAY2BGR);
        sensor_interface_->DrawPoints(tracked_pts_cam0_t1, show_frame_color);
      } else {
        show_frame_color = show_frame;
      }
      // DrawPoints(debug_pts, show_frame);
      sensor_interface_->cam0_->SendFrame(show_frame_color);
    }
    if (sensor_interface_->cam1_->OutputEnabled()) {
      cv::Mat show_frame, show_frame_color;
      d_frame_cam1_t1.download(show_frame);
      if (draw_points_to_frame) {
        cv::cvtColor(show_frame, show_frame_color, cv::COLOR_GRAY2BGR);
        sensor_interface_->DrawPoints(tracked_pts_cam1_t1, show_frame_color);
      } else {
        show_frame_color = show_frame;
      }
      sensor_interface_->cam1_->SendFrame(show_frame_color);
    }

    // Set the current t1 values to t0 for the next iteration
    d_frame_cam0_t0 = d_frame_cam0_t1.clone();
    d_frame_cam1_t0 = d_frame_cam1_t1.clone();
    d_tracked_pts_cam0_t1.upload(tracked_pts_cam0_t1);
    d_tracked_pts_cam0_t0 = d_tracked_pts_cam0_t1.clone();
    d_tracked_pts_cam1_t1.upload(tracked_pts_cam1_t1);
    d_tracked_pts_cam1_t0 = d_tracked_pts_cam1_t1.clone();
    d_keypoints_cam0_t0 = d_keypoints_cam0_t1.clone();
    d_keypoints_cam1_t0 = d_keypoints_cam1_t1.clone();
    std::swap(ids_tracked_t0, ids_tracked_t1); ids_tracked_t1.clear();
    std::swap(ids_detected_t0, ids_detected_t1); ids_detected_t1.clear();

    // std::cout << "fps " << 1.0 / static_cast<std::chrono::duration<double> >
    //   ((std::chrono::system_clock::now() - time_end)).count() << std::endl;
    time_end = std::chrono::system_clock::now();
  }
}

int ImageProcessor::StereoMatch(cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> opt,
  const cv::cuda::GpuMat &d_frame_cam0,
  const cv::cuda::GpuMat &d_frame_cam1,
  cv::cuda::GpuMat &d_tracked_pts_cam0,
  cv::cuda::GpuMat &d_tracked_pts_cam1,
  cv::cuda::GpuMat &d_status) {

  if (d_tracked_pts_cam0.cols == 0) {
    d_tracked_pts_cam1.release();
    d_status.release();
    return 1;
  }

  std::vector<cv::Point2f> tracked_pts_cam0_t1;
  std::vector<cv::Point2f> tracked_pts_cam1_t1;
  d_tracked_pts_cam0.download(tracked_pts_cam0_t1);

  cv::fisheye::undistortPoints(tracked_pts_cam0_t1, tracked_pts_cam1_t1,
    K_cam0_, D_cam0_, R_cam0_cam1_);

  std::vector<cv::Point3f> homogenous_pts;
  cv::convertPointsToHomogeneous(tracked_pts_cam1_t1, homogenous_pts);
  // cv::projectPoints(homogenous_pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(),
  //   K_cam1_, D_cam1_, corners_cam1_t1_);

  cv::fisheye::projectPoints(homogenous_pts, tracked_pts_cam1_t1, cv::Vec3d::zeros(),
    cv::Vec3d::zeros(), K_cam1_, D_cam1_);

  d_tracked_pts_cam1.upload(tracked_pts_cam1_t1);

  opt->calc(d_frame_cam0, d_frame_cam1, d_tracked_pts_cam0, d_tracked_pts_cam1, d_status);

  // Check if we have zero tracked points, this is a corner case and these variables need
  // to be reset to prevent errors in sizing, calc() does not return all zero length vectors
  if (d_tracked_pts_cam1.cols == 0) {
    d_tracked_pts_cam0.release();
    d_tracked_pts_cam1.release();
    d_status.release();
    return 0;
  }


  d_tracked_pts_cam1.download(tracked_pts_cam1_t1);

  if (RemovePointsOutOfFrame(d_frame_cam1.size(), d_tracked_pts_cam1, d_status)) {
    std::cout << "RemovePointsOutOfFrame failed 2\n" << std::endl;
    return -1;
  }

  std::vector<cv::Point2f> tracked_pts_cam0_t1_undistorted(0);
  std::vector<cv::Point2f> tracked_pts_cam1_t1_undistorted(0);
  cv::fisheye::undistortPoints(tracked_pts_cam0_t1,
    tracked_pts_cam0_t1_undistorted, K_cam0_, D_cam0_, cv::Mat(), P_cam0_);
  cv::fisheye::undistortPoints(tracked_pts_cam1_t1,
    tracked_pts_cam1_t1_undistorted, K_cam1_, D_cam1_, cv::Mat(), P_cam1_);

  std::vector<cv::Vec3f> epilines;
  cv::computeCorrespondEpilines(tracked_pts_cam0_t1_undistorted, 1, F_, epilines);

  std::vector<unsigned char> status;
  d_status.download(status);
  for (int i = 0; i < epilines.size(); i++) {
    if (status[i] == 0) {
      continue;
    }
    cv::Vec3f pt0(tracked_pts_cam0_t1_undistorted[i].x,
      tracked_pts_cam0_t1_undistorted[i].y, 1.0);

    cv::Vec3f pt1(tracked_pts_cam1_t1_undistorted[i].x,
      tracked_pts_cam1_t1_undistorted[i].y, 1.0);

    // Calculates the distance from the point to the epipolar line (in pixels)
    double error = fabs(pt1.dot(epilines[i]));

    if (error > stereo_threshold_)
      status[i] = 0;
  }
  d_status.upload(status);
  return 0;
}


int ImageProcessor::GetInputMaskFromPoints(const cv::cuda::GpuMat &d_input_corners,
  const cv::Size frame_size, cv::Mat &mask) {
  double mask_box_size = 15.0;
  cv::Mat local_mask(frame_size, CV_8U, cv::Scalar(1));

  if (d_input_corners.cols > 0) {
    std::vector<cv::Point2f> corners;
    d_input_corners.download(corners);
    for (const auto& point : corners) {
      const int x = static_cast<int>(point.x);
      const int y = static_cast<int>(point.y);

      int up_lim = y - floor(mask_box_size/2);
      int bottom_lim = y + ceil(mask_box_size/2);
      int left_lim = x - floor(mask_box_size/2);
      int right_lim = x + ceil(mask_box_size/2);
      if (up_lim < 0) up_lim = 0;
      if (bottom_lim > frame_size.height) bottom_lim = frame_size.height;
      if (left_lim < 0) left_lim = 0;
      if (right_lim > frame_size.width) right_lim = frame_size.width;

      cv::Range row_range(up_lim, bottom_lim);
      cv::Range col_range(left_lim, right_lim);
      local_mask(row_range, col_range) = 0;
    }
  }
  mask = local_mask;
  return 0;
}

int ImageProcessor::DetectNewFeatures(const cv::Ptr<cv::cuda::FastFeatureDetector> &detector_ptr,
  const cv::cuda::GpuMat &d_frame,
  const cv::cuda::GpuMat &d_input_corners,
  cv::cuda::GpuMat &d_output) {
  // Create a mask to avoid redetecting existing features.
  
  cv::Mat mask;
  GetInputMaskFromPoints(d_input_corners, d_frame.size(), mask);

  // Find features in the cam1 for tracking
  cv::cuda::GpuMat d_mask(mask);

  std::vector<cv::KeyPoint> temp_kp;
  detector_ptr->detect(d_frame, temp_kp, d_mask);

  std::vector<cv::Point2f> temp_pt;
  cv::KeyPoint::convert(temp_kp, temp_pt);
  d_output.upload(temp_pt);
}


int ImageProcessor::DetectNewFeatures(const cv::Ptr<cv::cuda::CornersDetector> &detector_ptr,
  const cv::cuda::GpuMat &d_frame,
  const cv::cuda::GpuMat &d_input_corners,
  cv::cuda::GpuMat &d_output) {
  // Create a mask to avoid redetecting existing features.
  
  cv::Mat mask;
  GetInputMaskFromPoints(d_input_corners, d_frame.size(), mask);

  // Find features in the cam1 for tracking
  cv::cuda::GpuMat d_mask(mask);

  detector_ptr->detect(d_frame, d_output, d_mask);
}

void ImageProcessor::rescalePoints(
    std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2,
    float& scaling_factor) {

  scaling_factor = 0.0f;

  for (int i = 0; i < pts1.size(); ++i) {
    scaling_factor += sqrt(pts1[i].dot(pts1[i]));
    scaling_factor += sqrt(pts2[i].dot(pts2[i]));
  }

  scaling_factor = (pts1.size()+pts2.size()) /
    scaling_factor * sqrt(2.0f);

  for (int i = 0; i < pts1.size(); ++i) {
    pts1[i] *= scaling_factor;
    pts2[i] *= scaling_factor;
  }

  return;
}

int ImageProcessor::twoPointRansac(
    const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
    const cv::Matx33f& R_p_c, const cv::Matx33d& intrinsics,
    const cv::Vec4d& distortion_coeffs,
    const double& inlier_error,
    const double& success_probability,
    std::vector<uchar>& inlier_markers) {

  // Check the size of input point size.
  if (pts1.size() != pts2.size()) {
    std::cerr << "Sets of different size (" << pts1.size() <<" and " <<
      pts2.size() << ") are used..." << std::endl;
    return -1;
  }

  double norm_pixel_unit = 2.0 / (intrinsics(0, 0)+intrinsics(1, 1));
  int iter_num = static_cast<int>(
      ceil(log(1-success_probability) / log(1-0.7*0.7)));

  // Initially, mark all points as inliers.
  inlier_markers.clear();
  inlier_markers.resize(pts1.size(), 1);

  // Undistort all the points.
  std::vector<cv::Point2f> pts1_undistorted(pts1.size());
  std::vector<cv::Point2f> pts2_undistorted(pts2.size());
  // TODO: make this fisheye
  cv::undistortPoints(pts1, pts1_undistorted, intrinsics, distortion_coeffs);
  cv::undistortPoints(pts2, pts2_undistorted, intrinsics, distortion_coeffs);

  // Compenstate the points in the previous image with
  // the relative rotation.
  for (auto& pt : pts1_undistorted) {
    cv::Vec3f pt_h(pt.x, pt.y, 1.0f);
    //Vec3f pt_hc = dR * pt_h;
    cv::Vec3f pt_hc = R_p_c * pt_h;
    pt.x = pt_hc[0];
    pt.y = pt_hc[1];
  }

  // Normalize the points to gain numerical stability.
  float scaling_factor = 0.0f;
  rescalePoints(pts1_undistorted, pts2_undistorted, scaling_factor);
  norm_pixel_unit *= scaling_factor;

  // Compute the difference between previous and current points,
  // which will be used frequently later.
  std::vector<cv::Point2d> pts_diff(pts1_undistorted.size());
  for (int i = 0; i < pts1_undistorted.size(); ++i)
    pts_diff[i] = pts1_undistorted[i] - pts2_undistorted[i];

  // Mark the point pairs with large difference directly.
  // BTW, the mean distance of the rest of the point pairs
  // are computed.
  double mean_pt_distance = 0.0;
  int raw_inlier_cntr = 0;
  for (int i = 0; i < pts_diff.size(); ++i) {
    double distance = sqrt(pts_diff[i].dot(pts_diff[i]));
    // 25 pixel distance is a pretty large tolerance for normal motion.
    // However, to be used with aggressive motion, this tolerance should
    // be increased significantly to match the usage.
    if (distance > 50.0*norm_pixel_unit) {
      inlier_markers[i] = 0;
    } else {
      mean_pt_distance += distance;
      ++raw_inlier_cntr;
    }
  }
  mean_pt_distance /= raw_inlier_cntr;

  // If the current number of inliers is less than 3, just mark
  // all input as outliers. This case can happen with fast
  // rotation where very few features are tracked.
  if (raw_inlier_cntr < 3) {
    for (auto& marker : inlier_markers) marker = 0;
    return 0;
  }

  // Before doing 2-point RANSAC, we have to check if the motion
  // is degenerated, meaning that there is no translation between
  // the frames, in which case, the model of the RANSAC does not
  // work. If so, the distance between the matched points will
  // be almost 0.
  //if (mean_pt_distance < inlier_error*norm_pixel_unit) {
  if (mean_pt_distance < norm_pixel_unit) {
    //ROS_WARN_THROTTLE(1.0, "Degenerated motion...");
    for (int i = 0; i < pts_diff.size(); ++i) {
      if (inlier_markers[i] == 0) continue;
      if (sqrt(pts_diff[i].dot(pts_diff[i])) >
          inlier_error*norm_pixel_unit)
        inlier_markers[i] = 0;
    }
    return 0;
  }

  // In the case of general motion, the RANSAC model can be applied.
  // The three column corresponds to tx, ty, and tz respectively.
  Eigen::MatrixXd coeff_t(pts_diff.size(), 3);
  for (int i = 0; i < pts_diff.size(); ++i) {
    coeff_t(i, 0) = pts_diff[i].y;
    coeff_t(i, 1) = -pts_diff[i].x;
    coeff_t(i, 2) = pts1_undistorted[i].x*pts2_undistorted[i].y -
      pts1_undistorted[i].y*pts2_undistorted[i].x;
  }

  std::vector<int> raw_inlier_idx;
  for (int i = 0; i < inlier_markers.size(); ++i) {
    if (inlier_markers[i] != 0)
      raw_inlier_idx.push_back(i);
  }

  std::vector<int> best_inlier_set;
  double best_error = 1e10;

  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution0(0, raw_inlier_idx.size()-1);
  std::uniform_int_distribution<int> distribution1(1, raw_inlier_idx.size()-1);


  for (int iter_idx = 0; iter_idx < iter_num; ++iter_idx) {
    // Randomly select two point pairs.
    // Although this is a weird way of selecting two pairs, but it
    // is able to efficiently avoid selecting repetitive pairs.
    int select_idx1 = distribution0(generator);
    int select_idx_diff = distribution1(generator);
    int select_idx2 = select_idx1+select_idx_diff<raw_inlier_idx.size() ?
      select_idx1+select_idx_diff :
      select_idx1+select_idx_diff-raw_inlier_idx.size();

    int pair_idx1 = raw_inlier_idx[select_idx1];
    int pair_idx2 = raw_inlier_idx[select_idx2];

    // Construct the model;
    Eigen::Vector2d coeff_tx(coeff_t(pair_idx1, 0), coeff_t(pair_idx2, 0));
    Eigen::Vector2d coeff_ty(coeff_t(pair_idx1, 1), coeff_t(pair_idx2, 1));
    Eigen::Vector2d coeff_tz(coeff_t(pair_idx1, 2), coeff_t(pair_idx2, 2));
    std::vector<double> coeff_l1_norm(3);
    coeff_l1_norm[0] = coeff_tx.lpNorm<1>();
    coeff_l1_norm[1] = coeff_ty.lpNorm<1>();
    coeff_l1_norm[2] = coeff_tz.lpNorm<1>();
    int base_indicator = min_element(coeff_l1_norm.begin(),
        coeff_l1_norm.end())-coeff_l1_norm.begin();

    Eigen::Vector3d model(0.0, 0.0, 0.0);
    if (base_indicator == 0) {
      Eigen::Matrix2d A;
      A << coeff_ty, coeff_tz;
      Eigen::Vector2d solution = A.inverse() * (-coeff_tx);
      model(0) = 1.0;
      model(1) = solution(0);
      model(2) = solution(1);
    } else if (base_indicator ==1) {
      Eigen::Matrix2d A;
      A << coeff_tx, coeff_tz;
      Eigen::Vector2d solution = A.inverse() * (-coeff_ty);
      model(0) = solution(0);
      model(1) = 1.0;
      model(2) = solution(1);
    } else {
      Eigen::Matrix2d A;
      A << coeff_tx, coeff_ty;
      Eigen::Vector2d solution = A.inverse() * (-coeff_tz);
      model(0) = solution(0);
      model(1) = solution(1);
      model(2) = 1.0;
    }

    // Find all the inliers among point pairs.
    Eigen::VectorXd error = coeff_t * model;

    std::vector<int> inlier_set;
    for (int i = 0; i < error.rows(); ++i) {
      if (inlier_markers[i] == 0) continue;
      if (std::abs(error(i)) < inlier_error*norm_pixel_unit)
        inlier_set.push_back(i);
    }

    // If the number of inliers is small, the current
    // model is probably wrong.
    if (inlier_set.size() < 0.2*pts1_undistorted.size())
      continue;

    // Refit the model using all of the possible inliers.
    Eigen::VectorXd coeff_tx_better(inlier_set.size());
    Eigen::VectorXd coeff_ty_better(inlier_set.size());
    Eigen::VectorXd coeff_tz_better(inlier_set.size());
    for (int i = 0; i < inlier_set.size(); ++i) {
      coeff_tx_better(i) = coeff_t(inlier_set[i], 0);
      coeff_ty_better(i) = coeff_t(inlier_set[i], 1);
      coeff_tz_better(i) = coeff_t(inlier_set[i], 2);
    }

    Eigen::Vector3d model_better(0.0, 0.0, 0.0);
    if (base_indicator == 0) {
      Eigen::MatrixXd A(inlier_set.size(), 2);
      A << coeff_ty_better, coeff_tz_better;
      Eigen::Vector2d solution =
          (A.transpose() * A).inverse() * A.transpose() * (-coeff_tx_better);
      model_better(0) = 1.0;
      model_better(1) = solution(0);
      model_better(2) = solution(1);
    } else if (base_indicator ==1) {
      Eigen::MatrixXd A(inlier_set.size(), 2);
      A << coeff_tx_better, coeff_tz_better;
      Eigen::Vector2d solution =
          (A.transpose() * A).inverse() * A.transpose() * (-coeff_ty_better);
      model_better(0) = solution(0);
      model_better(1) = 1.0;
      model_better(2) = solution(1);
    } else {
      Eigen::MatrixXd A(inlier_set.size(), 2);
      A << coeff_tx_better, coeff_ty_better;
      Eigen::Vector2d solution =
          (A.transpose() * A).inverse() * A.transpose() * (-coeff_tz_better);
      model_better(0) = solution(0);
      model_better(1) = solution(1);
      model_better(2) = 1.0;
    }

    // Compute the error and upate the best model if possible.
    Eigen::VectorXd new_error = coeff_t * model_better;

    double this_error = 0.0;
    for (const auto& inlier_idx : inlier_set)
      this_error += std::abs(new_error(inlier_idx));
    this_error /= inlier_set.size();

    if (inlier_set.size() > best_inlier_set.size()) {
      best_error = this_error;
      best_inlier_set = inlier_set;
    }
  }

  // Fill in the markers.
  inlier_markers.clear();
  inlier_markers.resize(pts1.size(), 0);
  for (const auto& inlier_idx : best_inlier_set)
    inlier_markers[inlier_idx] = 1;

  //printf("inlier ratio: %lu/%lu\n",
  //    best_inlier_set.size(), inlier_markers.size());

  return 0;
}

void ImageProcessor::ReceiveImu(const mavlink_imu_t &msg) {
  sensor_interface_->ReceiveImu(msg);
}
