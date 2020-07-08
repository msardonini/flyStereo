
#include <random>

#include "fly_stereo/image_processor.h"

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/cuda.hpp"
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

  // Make a copy of our configuration for later use
  input_params_ = input_params;
}

ImageProcessor::~ImageProcessor() {
  is_running_.store(false);

  trigger_->TriggerCamera();

  if (image_processor_thread_.joinable()) {
    // If we have started this thread, sometimes we can hang on read().
    // Reset the camera trigger to send pulsues async to get through this
    YAML::Node trigger_node = input_params_["CameraTrigger"];
    trigger_node["auto_trigger_async"] = true;
    trigger_node["auto_trigger_async_rate"] = "20";
    trigger_.reset();
    trigger_ = std::make_unique<CameraTrigger>(trigger_node);

    image_processor_thread_.join();
  }
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

  is_running_.store(true);
  image_processor_thread_ = std::thread(&ImageProcessor::ProcessThread, this);
}

void ImageProcessor::DrawPoints(const std::vector<cv::Point2f> &mypoints,
    cv::Mat &myimage) {
  int myradius=5;
  for (int i = 0; i < mypoints.size(); i++) {
    cv::Scalar color;
    if (i == 0) {
      color = CV_RGB(0, 0, 255);
    } else {
      color = CV_RGB(0, 0, 0);
    }
    circle(myimage, cv::Point(mypoints[i].x, mypoints[i].y), myradius, color,
      -1, 8, 0);
  }
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

int ImageProcessor::ProcessPoints(std::vector<cv::Point2f> pts_cam0,
  std::vector<cv::Point2f> pts_cam1) {

  if (pts_cam0.size() != pts_cam1.size()) {
    std::cerr << "Error! points are not the same size!" << std::endl;
    return -1;
  } else if (pts_cam0.size() == 0) {
    return -1;
  }

  // Cast all of our paramters to floating points for this function
  cv::Matx33f K_cam0 = static_cast<cv::Matx33f>(K_cam0_);
  cv::Matx33f K_cam1 = static_cast<cv::Matx33f>(K_cam1_);
  cv::Matx34f P_cam0 = static_cast<cv::Matx34f>(P_cam0_);
  cv::Matx34f P_cam1 = static_cast<cv::Matx34f>(P_cam1_);
  cv::Vec4f D_cam0 = static_cast<cv::Vec4f>(D_cam0_);
  cv::Vec4f D_cam1 = static_cast<cv::Vec4f>(D_cam1_);

  cv::Mat tmp_cam0(pts_cam0);
  cv::Mat tmp_cam1(pts_cam1);
  cv::Mat output;

  cv::fisheye::undistortPoints(pts_cam0, pts_cam0, K_cam0, D_cam0, R_cam0_, P_cam0);
  cv::fisheye::undistortPoints(pts_cam1, pts_cam1, K_cam1, D_cam1, R_cam1_, P_cam1);


  cv::triangulatePoints(P_cam0, P_cam1, tmp_cam0, tmp_cam1, output);

  std::vector<cv::Vec3f> vec;
  cv::convertPointsFromHomogeneous(output.t(), vec);

  std::cout << "size of vec: " << vec.size() << std::endl;
  if (vec.size() > 0) {
    std::cout << "point 0 X,Y,Z: " << vec[0] << std::endl;
  }

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
  std::chrono::time_point<std::chrono::system_clock> time_end = std::chrono::
    system_clock::now();

  cv::Ptr<cv::cuda::CornersDetector> detector_ptr = cv::cuda::
    createGoodFeaturesToTrackDetector(CV_8U, 1000, 0.2, 15);

  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_opt_flow_cam0 = cv::cuda::
    SparsePyrLKOpticalFlow::create();
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_opt_flow_cam1 = cv::cuda::
    SparsePyrLKOpticalFlow::create();
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_opt_flow_cam1_temp = cv::cuda::
    SparsePyrLKOpticalFlow::create();

  // std::vector<cv::Point2f> debug_pts;

  // debug_pts.push_back(cv::Point2f(320, 180));
  // debug_pts.push_back(cv::Point2f(640, 180));
  // debug_pts.push_back(cv::Point2f(960, 180));
  // debug_pts.push_back(cv::Point2f(320, 360));
  // debug_pts.push_back(cv::Point2f(640, 360));
  // debug_pts.push_back(cv::Point2f(960, 360));
  // debug_pts.push_back(cv::Point2f(320, 540));
  // debug_pts.push_back(cv::Point2f(640, 540));
  // debug_pts.push_back(cv::Point2f(960, 540));
  // cv::cuda::GpuMat d_debug_pts; d_debug_pts.upload(debug_pts);
  //   d_debug_pts.download(debug_pts);
  // std::cout << "debug_pts " << debug_pts.size() << " x,y: " << debug_pts[0].x << ", " << debug_pts[0].y << std::endl;

  while (is_running_.load()) {
    // Trigger the camera to start acquiring an image
    trigger_->TriggerCamera();

    // Read the frame and check for errors
    if (cam0_->GetFrame(d_frame_cam0_t1) || cam1_->GetFrame(d_frame_cam1_t1)) {
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
    int retval = GenerateImuXform(counter, rotation_t0_t1_cam0, rotation_t0_t1_cam1);

    // if (retval == 0) {
    //   // cv::cuda::GpuMat tmp;
    //   std::cout << "running UpdatePointsViaImu \n";
    //   if(UpdatePointsViaImu(d_debug_pts, rotation_t0_t1_cam0, K_cam0_, d_debug_pts)) {
    //     std::cout << "Error in UpdatePointsViaImu \n";
    //   }
    //   // d_debug_pts = tmp;
    // }


    if (retval == 0) {
      UpdatePointsViaImu(d_tracked_pts_cam0_t0, rotation_t0_t1_cam0, K_cam0_,
        d_tracked_pts_cam0_t1);
      UpdatePointsViaImu(d_tracked_pts_cam1_t0, rotation_t0_t1_cam1, K_cam1_,
        d_tracked_pts_cam1_t1);
    }
    std::cout << counter << std::endl;


    d_tracked_pts_cam0_t0 = AppendGpuMatColwise(d_tracked_pts_cam0_t0, d_keypoints_cam0_t0);
    d_tracked_pts_cam1_t0 = AppendGpuMatColwise(d_tracked_pts_cam1_t0, d_keypoints_cam1_t0);
    ids_tracked_t0.insert(std::end(ids_tracked_t0), std::begin(ids_detected_t0), std::end(
      ids_detected_t0));
    ids_tracked_t1 = ids_tracked_t0;

    if (counter++ != 0 && d_tracked_pts_cam0_t0.rows > 0) {
      // Apply the optical flow from the previous frame to the current frame
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

      StereoMatch(d_opt_flow_cam1, d_frame_cam0_t1, d_frame_cam1_t1, d_tracked_pts_cam0_t1,
        d_tracked_pts_cam1_t1, d_status);

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
        return -1;
      }
    }

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

    // Run the Two Point RANSAC algorithm to find more outliers
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

        if (RemoveOutliers(status, tracked_pts_cam0_t1) ||
            RemoveOutliers(status, tracked_pts_cam1_t1) ||
            RemoveOutliers(status, ids_tracked_t1)) {
          std::cout << "RemoveOutliers failed after Ransac\n" << std::endl;
        }
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
    }

    // Detect new features for the next iteration
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
      d_tracked_pts_cam1_t1.cols << std::endl;


    // Signal this loop has finished and output the tracked points
    if (tracked_pts_cam0_t1.size() > 1 && tracked_pts_cam1_t1.size() > 1) {
      OuputTrackedPoints(tracked_pts_cam0_t1, tracked_pts_cam1_t1, ids_tracked_t1);
    }

    // If requested, output the video stream to the configured gstreamer pipeline
    if (cam0_->OutputEnabled()) {
      cv::Mat show_frame, show_frame_color;
      d_frame_cam0_t1.download(show_frame);
      if (draw_points_to_frame) {
        cv::cvtColor(show_frame, show_frame_color, cv::COLOR_GRAY2BGR);
        DrawPoints(tracked_pts_cam0_t1, show_frame_color);
      } else {
        show_frame_color = show_frame;
      }
      // DrawPoints(debug_pts, show_frame);
      cam0_->SendFrame(show_frame_color);
    }
    if (cam1_->OutputEnabled()) {
      cv::Mat show_frame, show_frame_color;
      d_frame_cam1_t1.download(show_frame);
      if (draw_points_to_frame) {
        cv::cvtColor(show_frame, show_frame_color, cv::COLOR_GRAY2BGR);
        DrawPoints(tracked_pts_cam0_t1, show_frame_color);
      } else {
        show_frame_color = show_frame;
      }
      cam1_->SendFrame(show_frame_color);
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

  if (d_tracked_pts_cam0.cols == 0)
    return 0;

  std::vector<cv::Point2f> tracked_pts_cam0_t1;
  std::vector<cv::Point2f> tracked_pts_cam1_t1;
  d_tracked_pts_cam0.download(tracked_pts_cam0_t1);

  cv::fisheye::undistortPoints(tracked_pts_cam0_t1, tracked_pts_cam1_t1,
    K_cam0_, D_cam0_, R_cam0_cam1_);

  std::vector<cv::Point3f> homogenous_pts;
  cv::convertPointsToHomogeneous(tracked_pts_cam1_t1, homogenous_pts);
  // cv::projectPoints(homogenous_pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(),
  //   K_cam1_, D_cam1_, corners_cam1_t1_);

  cv::fisheye::projectPoints(homogenous_pts, tracked_pts_cam1_t1,
    cv::Vec3d::zeros(), cv::Vec3d::zeros(), K_cam1_, D_cam1_);

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
  cv::computeCorrespondEpilines(tracked_pts_cam0_t1_undistorted, 1, F_,
    epilines);

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

int ImageProcessor::DetectNewFeatures(const cv::Ptr<cv::cuda::CornersDetector> &detector_ptr,
  const cv::cuda::GpuMat &d_frame,
  const cv::cuda::GpuMat &d_input_corners,
  cv::cuda::GpuMat &d_output) {
  // Create a mask to avoid redetecting existing features.
  double mask_box_size = 15.0;
  cv::Mat mask_host(d_frame.rows, d_frame.cols, CV_8U,
    cv::Scalar(1));

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
      if (bottom_lim > d_frame.rows) bottom_lim = d_frame.rows;
      if (left_lim < 0) left_lim = 0;
      if (right_lim > d_frame.cols) right_lim = d_frame.cols;

      cv::Range row_range(up_lim, bottom_lim);
      cv::Range col_range(left_lim, right_lim);
      mask_host(row_range, col_range) = 0;
    }
  }

  // Find features in the cam1 for tracking
  cv::cuda::GpuMat d_mask(mask_host);


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
  std::lock_guard<std::mutex> lock(imu_queue_mutex_);
  imu_queue_.push(msg);
}

int ImageProcessor::GenerateImuXform(int image_counter, cv::Matx33f &rotation_t0_t1_cam0,
  cv::Matx33f &rotation_t0_t1_cam1) {

  // The first image will not have relvant imu data
  if (image_counter == 0) {
    rotation_t0_t1_cam0 = cv::Matx33f::eye();
    rotation_t0_t1_cam1 = cv::Matx33f::eye();
    return 0;
  }

  if (imu_queue_.size() == 0) {
    std::cerr << "Imu queue empty!" << std::endl;
    rotation_t0_t1_cam0 = cv::Matx33f::eye();
    rotation_t0_t1_cam1 = cv::Matx33f::eye();
    return -1;
  }
  std::vector<mavlink_imu_t> current_frame_msgs;

  // Reserve some memory in our vector to prevent copies. We should expect this to fill up to  
  //  roughly (IMU acquisition rate / image acquisition rate )
  current_frame_msgs.reserve(20);

  std::queue<std::pair<uint32_t, uint64_t> > trigger_queue = trigger_->GetTriggerCount();

  while(trigger_queue.size() > 1) {
    std::cerr << "Error! Mismatch between number of triggers and proccesed frames" << std::endl;
    trigger_queue.pop();
  }

  if (image_counter != std::get<0>(trigger_queue.front())) {
    std::cerr << "Error! Mismatch between number of triggers and proccesed frames, image: "
      << image_counter << ", from_imu: " << std::get<0>(trigger_queue.front()) << std::endl;
    rotation_t0_t1_cam0 = cv::Matx33f::eye();
    rotation_t0_t1_cam1 = cv::Matx33f::eye();
    return -1;
  }

  // The objective is to get the rotation matrix from the previous frame to the current, so
  // take all the imu measurements that correspond to the current frame minus 1

  int image_of_interest = image_counter - 1;
  std::lock_guard<std::mutex> lock(imu_queue_mutex_);
  while (!imu_queue_.empty() && imu_queue_.front().trigger_count <= image_of_interest) {
    if (imu_queue_.front().trigger_count == image_of_interest) {
      current_frame_msgs.push_back(imu_queue_.front());
    }
    imu_queue_.pop();
  }

  // Continue processesing now we have the imu data from the appropriate image frame
  // We need at least 2 messages in order to run the integration
  if (current_frame_msgs.size() < 2) {
    std::cerr << "Error! Not enough imu messages to run integration" << std::endl;
    return -1;
  }

  // Integrate the roll/pitch/yaw values
  cv::Vec3f delta_rpw = {0.0f, 0.0f, 0.0f};
  float dt = 0.0f;
  for (int i = 0; i < current_frame_msgs.size(); i++) {
    // Calculate the dt. For the last iteration, just use the same dt as before
    if (i != current_frame_msgs.size() - 1) {
      dt = current_frame_msgs[i + 1].timestamp_us - current_frame_msgs[i].timestamp_us;
      dt /= 1.0E6f;
    }

    delta_rpw[0] -= current_frame_msgs[i].gyroXYZ[0] * dt; 
    delta_rpw[1] += current_frame_msgs[i].gyroXYZ[1] * dt; 
    delta_rpw[2] += current_frame_msgs[i].gyroXYZ[2] * dt; 
  }
  cv::Rodrigues(R_imu_cam0_ * delta_rpw, rotation_t0_t1_cam0); 
  cv::Rodrigues(R_imu_cam1_ * delta_rpw, rotation_t0_t1_cam1);
  // std::cout << delta_rpw: " << delta_rpw << std::endl;
  return 0;
}
