
#include <random>

#include "fly_stereo/image_processor.h"

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "Eigen/Dense"


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
  R_cam1_cam0_ = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["T"].as<std::vector<double>>();
  T_cam1_cam0_ = cv::Vec3d(interface_vec.data());

  interface_vec = stereo_calibration["E"]["data"].as<std::vector<double>>();
  E_ = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["F"]["data"].as<std::vector<double>>();
  F_ = cv::Matx33d(interface_vec.data());

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

  // Make a copy of our configuration for later use
  input_params_ = input_params;
}

ImageProcessor::~ImageProcessor() {
  is_running_.store(false);

  if (image_processor_thread_.joinable())
    image_processor_thread_.join();
}

int ImageProcessor::Init() {
  cam0_ = std::make_unique<Camera> (input_params_["Camera0"]);
  if(cam0_->Init()) {
    return -1;
  }
  cam1_ = std::make_unique<Camera> (input_params_["Camera1"]);
  if (cam1_->Init()) {
    return -1;
  }
  trigger_ = std::make_unique<CameraTrigger> (input_params_["CameraTrigger"]);

  if (trigger_->Init()) {
    return -1;
  }

  // // Trigger the first image
  trigger_->TriggerCamera();
  // std::this_thread::sleep_for(std::chrono::microseconds(30000));
  // trigger_->TriggerCamera();
  // std::this_thread::sleep_for(std::chrono::microseconds(30000));
  // trigger_->TriggerCamera();

  is_running_.store(true);
  image_processor_thread_ = std::thread(&ImageProcessor::ProcessThread, this);
}

int ImageProcessor::InitFirstFrame(const cv::Mat &frame_cam0_t0,
    const cv::Mat &frame_cam1_t0) {
  // Find features in cam0 for tracking
  cv::goodFeaturesToTrack(frame_cam0_t0, corners_cam0_t0_, max_corners_,
    quality_level_, min_dist_);
  cv::buildOpticalFlowPyramid(frame_cam0_t0, pyramid_cam0_t0_,
    cv::Size(window_size_, window_size_), max_pyramid_level_);

  // Find features in the cam1 for tracking
  cv::goodFeaturesToTrack(frame_cam1_t0, corners_cam1_t0_, max_corners_,
    quality_level_, min_dist_);
  cv::buildOpticalFlowPyramid(frame_cam1_t0, pyramid_cam1_t0_,
    cv::Size(window_size_, window_size_), max_pyramid_level_);
}

void ImageProcessor::DrawPoints(const std::vector<cv::Point2f> &mypoints,
    cv::Mat &myimage) {
  int myradius=5;
  for (int i = 0; i < mypoints.size(); i++) {
    circle(myimage ,cv::Point(mypoints[i].x, mypoints[i].y), myradius,
      CV_RGB(255, 0, 0), -1, 8, 0);
  }
}

int ImageProcessor::RemoveOutliers(const cv::Size framesize,
  std::vector<uchar> &status, std::vector<cv::Point2f> &points) {
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
    if (points[i].y < 0 ||
        points[i].y > framesize.height - 1 ||
        points[i].x < 0 ||
        points[i].x > framesize.width - 1) {
      status[i] = 0;
    }
  }

  for (int i = status.size() - 1; i >= 0; i--) {
    if (status[i] != 1) {
      points.erase(points.begin() + i);
    }
  }
}

int ImageProcessor::ProcessThread() {
  // Frames of the left and right camera
  cv::Mat frame_cam0_t1, frame_cam1_t1;
  cv::Mat frame_cam0_t0, frame_cam1_t0;

  std::vector<cv::Point2f> prev_cam0_stereo;
  std::vector<cv::Point2f> prev_cam1_stereo;

  // Outputs for the calcOpticalFlowPyrLK call
  std::vector<unsigned char> status;
  std::vector<float> err;

  int counter = 0;
  int error_counter = 0;
  std::chrono::time_point<std::chrono::system_clock> time_end;
  cv::Ptr<cv::Feature2D> detector_ptr = cv::FastFeatureDetector::create(50,
    true);
  
  // trigger_->TriggerCamera();
  
  while (is_running_.load()) {
    // Read the frame and check for errors
    if (cam0_->GetFrame(frame_cam0_t1) || cam1_->GetFrame(frame_cam1_t1)) {
      if (++error_counter >= max_error_counter_) {
        std::cerr << "Error! Failed to read more than " << max_error_counter_
          << " frames" << std::endl;
        is_running_.store(false);
        return -1;
      } else {
        continue;
      }
    }
    // TODO: Find a better place to do this
    trigger_->TriggerCamera();

    cv::flip(frame_cam0_t1, frame_cam0_t1, 0);
    cv::flip(frame_cam1_t1, frame_cam1_t1, 0);

    // Initialize some values if this is our first frame
    if (counter++ == 0) {
      InitFirstFrame(frame_cam0_t1, frame_cam1_t1);
      frame_cam0_t0 = frame_cam0_t1;
      frame_cam1_t0 = frame_cam1_t1;
      continue;
    } else if (counter % 100 == 0) {
      DetectNewFeatures(frame_cam0_t0, detector_ptr);

    }

    cv::buildOpticalFlowPyramid(frame_cam0_t1, pyramid_cam0_t1_, cv::Size(
      window_size_, window_size_), max_pyramid_level_);
    if (corners_cam0_t0_.size() > 0) {
      cv::calcOpticalFlowPyrLK(pyramid_cam0_t0_, pyramid_cam0_t1_,
        corners_cam0_t0_, corners_cam0_t1_, status, err);
    }

    RemoveOutliers(frame_cam0_t1.size(), status, corners_cam0_t1_);

    // Vector showing which points are matched between both cameras
    std::vector<unsigned char> stereo_points_status;
    StereoMatch(frame_cam1_t1, stereo_points_status);

    RemoveOutliers(frame_cam0_t1.size(), stereo_points_status,
      corners_cam0_t1_);
    RemoveOutliers(frame_cam1_t1.size(), stereo_points_status,
      corners_cam1_t1_);


    if (!prev_cam0_stereo.empty() && !prev_cam1_stereo.empty()) {
      std::vector<uchar> cam0_ransac_inliers(0);
      int ret1 = twoPointRansac(prev_cam0_stereo, corners_cam0_t0_,
          cv::Matx33f::eye(), K_cam0_, D_cam0_,
          ransac_threshold_,
          0.99, cam0_ransac_inliers);

      std::vector<uchar> cam1_ransac_inliers(0);
      int ret2 = twoPointRansac(prev_cam1_stereo, corners_cam1_t1_,
          cv::Matx33f::eye(), K_cam1_, D_cam1_,
          ransac_threshold_,
          0.99, cam1_ransac_inliers);
  
      prev_cam0_stereo = corners_cam0_t1_;
      prev_cam1_stereo = corners_cam1_t1_;

      if (ret1 == 0 && ret2 == 0) {
        std::cout <<" before RANSAC " << corners_cam1_t1_.size();
        RemoveOutliers(frame_cam0_t1.size(), cam0_ransac_inliers,
          corners_cam0_t1_);
        RemoveOutliers(frame_cam1_t1.size(), cam1_ransac_inliers,
          corners_cam1_t1_);
        std::cout <<" after RANSAC " << corners_cam1_t1_.size() << std::endl;
      }
    } else {
      prev_cam0_stereo = corners_cam0_t1_;
      prev_cam1_stereo = corners_cam1_t1_;
    }


    std::cout << "num points: cam0 " << corners_cam0_t1_.size()
      << " cam1 " << corners_cam1_t1_.size() << std::endl;

    //TODO fix
    if (1) {
      cv::Mat show_frame = frame_cam0_t1.clone();
      DrawPoints(corners_cam0_t1_, show_frame);
      cam0_->SendFrame(show_frame);
    }
    if (1) {
      cv::Mat show_frame = frame_cam1_t1.clone();
      DrawPoints(corners_cam1_t1_, show_frame);
      cam1_->SendFrame(show_frame);
    }

    // Set the current t1 values to t0
    pyramid_cam0_t0_.swap(pyramid_cam0_t1_);
    corners_cam0_t0_.swap(corners_cam0_t1_);
    frame_cam0_t0 = frame_cam0_t1;
    frame_cam1_t0 = frame_cam1_t1;

    std::cout << "fps " << 1.0 / static_cast<std::chrono::duration<double> >
      ((std::chrono::system_clock::now() - time_end)).count() << std::endl;
    time_end = std::chrono::system_clock::now();
  
    counter++;
  }
}

int ImageProcessor::StereoMatch(cv::Mat frame_cam1_t1, std::vector<
  unsigned char> &status) {

  if (corners_cam0_t1_.size() == 0)
    return 0;

  if (corners_cam1_t1_.size() != corners_cam0_t1_.size()) {  
    cv::undistortPoints(corners_cam0_t1_, corners_cam1_t1_, K_cam0_, D_cam0_, R_cam1_cam0_.t());

    std::vector<cv::Point3f> homogenous_pts;
    cv::convertPointsToHomogeneous(corners_cam1_t1_, homogenous_pts);
    cv::projectPoints(homogenous_pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(),
      K_cam1_, D_cam1_, corners_cam1_t1_);
  }

  // Outputs for the calcOpticalFlowPyrLK call
  std::vector<float> err;

  cv::buildOpticalFlowPyramid(frame_cam1_t1, pyramid_cam1_t1_, cv::Size(
    window_size_, window_size_), max_pyramid_level_);
  calcOpticalFlowPyrLK(pyramid_cam0_t1_, pyramid_cam1_t1_, corners_cam0_t1_,
    corners_cam1_t1_, status, err, cv::Size(window_size_, window_size_),
    max_pyramid_level_, cv::TermCriteria(cv::TermCriteria::COUNT +
      cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

  // Filter outliers based on the distance between a point and its
  // corresponding epipolar line
  std::vector<cv::Point2f> corners_cam0_t1_undistorted(0);
  std::vector<cv::Point2f> corners_cam1_t1_undistorted(0);
  cv::undistortPoints(corners_cam0_t1_, corners_cam0_t1_undistorted, K_cam0_,
    D_cam0_, cv::Mat(), P_cam0_);
  cv::undistortPoints(corners_cam1_t1_, corners_cam1_t1_undistorted, K_cam1_,
    D_cam1_, cv::Mat(), P_cam1_);

  std::vector<cv::Vec3f> epilines;
  cv::computeCorrespondEpilines(corners_cam0_t1_undistorted, 1, F_, epilines);

  for (int i=0; i < epilines.size(); i++) {
    cv::Vec3f pt0(corners_cam0_t1_undistorted[i].x,
      corners_cam0_t1_undistorted[i].y, 1.0);

    cv::Vec3f pt1(corners_cam1_t1_undistorted[i].x,
      corners_cam1_t1_undistorted[i].y, 1.0);

    // Calculates the distance from the point to the epipolar line (in pixels)
    double error = fabs(pt1.dot(epilines[i]));
    if (error > stereo_threshold_)
      status[i] = 0;
  }

  return 0;
}

int ImageProcessor::DetectNewFeatures(cv::Mat frame_cam0_t0,
  cv::Ptr<cv::Feature2D> detector_ptr) {
  double mask_box_size = 15.0;
  // Create a mask to avoid redetecting existing features.
  cv::Mat mask(frame_cam0_t0.rows, frame_cam0_t0.cols, CV_8U, cv::Scalar(1));

  for (const auto& point : corners_cam0_t0_) {
    const int x = static_cast<int>(point.x);
    const int y = static_cast<int>(point.y);

    int up_lim = y - floor(mask_box_size/2);
    int bottom_lim = y + ceil(mask_box_size/2);
    int left_lim = x - floor(mask_box_size/2);
    int right_lim = x + ceil(mask_box_size/2);
    if (up_lim < 0) up_lim = 0;
    if (bottom_lim > frame_cam0_t0.rows) bottom_lim = frame_cam0_t0.rows;
    if (left_lim < 0) left_lim = 0;
    if (right_lim > frame_cam0_t0.cols) right_lim = frame_cam0_t0.cols;

    cv::Range row_range(up_lim, bottom_lim);
    cv::Range col_range(left_lim, right_lim);
    mask(row_range, col_range) = 0;
  }


  // Find features in the cam1 for tracking
  // cv::goodFeaturesToTrack(frame_cam0_t0, corners_cam0_t0_, max_corners_,
  //   quality_level_, min_dist_);
  std::vector<cv::Mat> tmp_frame_vec; 
  tmp_frame_vec.push_back(frame_cam0_t0);
  std::vector<cv::KeyPoint> keypoints;
  std::vector<cv::Point2f> tmp_frame_vec_pt; 
  detector_ptr->detect(frame_cam0_t0, keypoints, mask);

  cv::KeyPoint::convert(keypoints, tmp_frame_vec_pt); 
  std::cout << "detected: " << keypoints.size() << std::endl;

  // Append the features we just detected to our existing vector
  corners_cam0_t0_.insert(corners_cam0_t0_.end(), tmp_frame_vec_pt.begin(),
    tmp_frame_vec_pt.end());

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
