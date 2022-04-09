#include "fly_stereo/image_processor.h"

#include <fstream>  // std::ofstream
#include <random>

#include "Eigen/Dense"
#include "fly_stereo/utility.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opengv/types.hpp"
#include "spdlog/spdlog.h"

// Debug includes
#include "debug_video_recorder.h"

bool write_to_debug = false;
debug_video_recorder debug_record;

ImageProcessor::ImageProcessor(const YAML::Node &input_params, const YAML::Node &stereo_calibration) {
  YAML::Node image_processor_params = input_params["image_processor"];

  rate_limit_fps_ = input_params["image_processor"]["rate_limit_fps"].as<float>();

  // Params for OpenCV function goodFeaturesToTrack
  YAML::Node features_params = image_processor_params["goodFeaturesToTrack"];
  max_corners_ = features_params["max_corners"].as<int>();
  quality_level_ = features_params["quality_level"].as<float>();
  min_dist_ = features_params["min_dist"].as<float>();

  YAML::Node binning = image_processor_params["binning"];
  bins_width_ = binning["bins_width"].as<unsigned int>();
  bins_height_ = binning["bins_height"].as<unsigned int>();
  max_pts_in_bin_ = binning["max_pts_in_bin"].as<unsigned int>();

  // Params for OpenCV function calcOpticalFlowPyrLK
  YAML::Node optical_flow_params = image_processor_params["calcOpticalFlowPyrLK"];
  window_size_ = optical_flow_params["window_size"].as<int>();
  max_pyramid_level_ = optical_flow_params["max_pyramid_level"].as<int>();
  max_iters_ = optical_flow_params["max_iters"].as<int>();

  draw_points_to_frame_ = image_processor_params["draw_points_to_frame"].as<bool>();
  max_error_counter_ = image_processor_params["max_error_counter"].as<int>();

  // Thresholds
  YAML::Node thresholds = image_processor_params["thresholds"];
  stereo_threshold_ = thresholds["stereo_threshold"].as<double>();
  ransac_threshold_ = thresholds["ransac_threshold"].as<double>();

  // Load the stereo camera calibration
  std::vector<double> interface_vec = stereo_calibration["K0"]["data"].as<std::vector<double>>();
  K_cam0_ = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["K1"]["data"].as<std::vector<double>>();
  K_cam1_ = cv::Matx33d(interface_vec.data());

  D_cam0_ = stereo_calibration["D0"]["data"].as<std::vector<double>>();

  D_cam1_ = stereo_calibration["D1"]["data"].as<std::vector<double>>();

  interface_vec = stereo_calibration["R"]["data"].as<std::vector<double>>();
  R_cam0_cam1_ = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["T"]["data"].as<std::vector<double>>();
  T_cam0_cam1_ = cv::Vec3d(interface_vec.data());

  // Calculate the essential matrix and fundamental matrix
  const cv::Matx33d T_cross_mat(0.0, -T_cam0_cam1_[2], T_cam0_cam1_[1], T_cam0_cam1_[2], 0.0, -T_cam0_cam1_[0],
                                -T_cam0_cam1_[1], T_cam0_cam1_[0], 0.0);

  E_ = T_cross_mat * R_cam0_cam1_;
  F_ = K_cam0_.inv().t() * E_ * K_cam1_.inv();

  std::vector<float> imu_cam0_vec = stereo_calibration["R_imu_cam0"].as<std::vector<float>>();
  cv::Vec3f angles_imu_cam0 = {imu_cam0_vec[0], imu_cam0_vec[1], imu_cam0_vec[2]};
  R_imu_cam0_ = utility::eulerAnglesToRotationMatrix<float>(angles_imu_cam0);

  cv::Matx33f R_cam0_cam1_f = R_cam0_cam1_;
  R_imu_cam1_ = R_imu_cam0_ * R_cam0_cam1_f;

  pose_ = Eigen::Matrix4d::Identity();
  prev_xform_ = Eigen::Matrix4d::Identity();

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
  return 0;
}

int ImageProcessor::UpdatePointsViaImu(const std::vector<cv::Point2f> &current_pts, const cv::Matx33d &rotation,
                                       const cv::Matx33d &camera_matrix, std::vector<cv::Point2f> &updated_pts) {
  if (current_pts.size() == 0) {
    return -1;
  }

  cv::Matx33f H = camera_matrix * rotation.t() * camera_matrix.inv();

  updated_pts.resize(current_pts.size());
  for (size_t i = 0; i < current_pts.size(); i++) {
    cv::Vec3f temp_current(current_pts[i].x, current_pts[i].y, 1.0f);
    cv::Vec3f temp_updated = H * temp_current;
    updated_pts[i].x = temp_updated[0] / temp_updated[2];
    updated_pts[i].y = temp_updated[1] / temp_updated[2];
  }
  return 0;
}

int ImageProcessor::UpdatePointsViaImu(const cv::cuda::GpuMat &d_current_pts, const cv::Matx33d &rotation,
                                       const cv::Matx33d &camera_matrix, cv::cuda::GpuMat &d_updated_pts) {
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

int ImageProcessor::OuputTrackedPoints(OpticalFlowPoints &points, const std::vector<unsigned int> &ids,
                                       const std::vector<mavlink_imu_t> &imu_msgs,
                                       const cv::Matx33f &rotation_t0_t1_cam0) {
  // Check to make sure that our input data is sized correctly
  unsigned int num_pts = points.GetCpu(t_c0_t0).size();
  if (num_pts != points.GetCpu(t_c0_t1).size() || num_pts != points.GetCpu(t_c1_t0).size() ||
      num_pts != points.GetCpu(t_c1_t1).size() || num_pts != ids.size()) {
    spdlog::error("Error! Vectors do not match in OutputTrackedPoints");
    return -1;
  }

  // Acquire access to shared data, then write to it
  std::lock_guard<std::mutex> lock(output_mutex_);
  output_points_.timestamp_us =
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
          .count();
  output_points_.pts.clear();
  output_points_.pts.reserve(num_pts);

  for (size_t i = 0; i < num_pts; i++) {
    ImagePoint pt(ids[i], {points.GetCpu(t_c0_t0)[i].x, points.GetCpu(t_c0_t0)[i].y},
                  {points.GetCpu(t_c0_t1)[i].x, points.GetCpu(t_c0_t1)[i].y},
                  {points.GetCpu(t_c1_t0)[i].x, points.GetCpu(t_c1_t0)[i].y},
                  {points.GetCpu(t_c1_t1)[i].x, points.GetCpu(t_c1_t1)[i].y});
    output_points_.pts.push_back(pt);
  }

  // Also save a copy of the imu messages that are associated to this camera frame
  output_points_.imu_pts = imu_msgs;
  cv::Matx33d temp_rot = rotation_t0_t1_cam0;
  cv::cv2eigen(temp_rot, output_points_.R_t0_t1_cam0);

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
#include "opencv2/highgui.hpp"

int ImageProcessor::ProcessThread() {
  // Vector of imu messages
  std::vector<mavlink_imu_t> imu_msgs;

  // Frames of the left and right camera in current timestep and previous
  cv::cuda::GpuMat d_frame_cam0_t1, d_frame_cam1_t1;
  cv::cuda::GpuMat d_frame_cam0_t0, d_frame_cam1_t0;

  OpticalFlowPoints points;
  unsigned int current_id = 0;

  int counter = 0;
  int error_counter = 0;
  // int debug_num_pts[6];
  std::chrono::time_point<std::chrono::system_clock> time_end = std::chrono::system_clock::now();

  cv::Ptr<cv::cuda::CornersDetector> detector_ptr =
      cv::cuda::createGoodFeaturesToTrackDetector(CV_8U, max_corners_, quality_level_, min_dist_);

  // cv::Ptr<cv::cuda::FastFeatureDetector> detector_ptr = cv::cuda::FastFeatureDetector::create();

  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_opt_flow_cam0 = cv::cuda::SparsePyrLKOpticalFlow::create(
      cv::Size(window_size_, window_size_), max_pyramid_level_, max_iters_, true);
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_opt_flow_stereo_t0 = cv::cuda::SparsePyrLKOpticalFlow::create(
      cv::Size(window_size_, window_size_), max_pyramid_level_, max_iters_, true);
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_opt_flow_stereo_t1 = cv::cuda::SparsePyrLKOpticalFlow::create(
      cv::Size(window_size_, window_size_), max_pyramid_level_, max_iters_, true);

  auto invFpsLimit =
      std::chrono::round<std::chrono::system_clock::duration>(std::chrono::duration<double>(1. / rate_limit_fps_));

  // Time checks for performance monitoring
  std::chrono::time_point<std::chrono::system_clock> t_start = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> t_data = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> t_lk1 = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> t_det = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> t_log = std::chrono::system_clock::now();

  while (is_running_.load()) {
    t_start = std::chrono::system_clock::now();
    uint64_t current_time;
    // Read the frames and check for errors
    int ret_val = sensor_interface_->GetSynchronizedData(d_frame_cam0_t1, d_frame_cam1_t1, imu_msgs, current_time);
    if (ret_val == -1) {
      if (++error_counter >= max_error_counter_) {
        spdlog::error("Error! Failed to read more than {} consecutive frames", max_error_counter_);
        is_running_.store(false);
        return -1;
      } else {
        continue;
      }
    } else if (ret_val == 2) {
      // This will happen if we don't have IMU data, for now continue on as is
      continue;
    } else if (ret_val == -2) {
      spdlog::info("Sensor error or end of replay, shutting down");
      // This will happen if we have a critical error or the replay has finished. Shut down
      is_running_.store(false);
      continue;
    }
    t_data = std::chrono::system_clock::now();
    // Reset the Error Counter
    error_counter = 0;

    cv::Matx33f rotation_t0_t1_cam0;
    cv::Matx33f rotation_t0_t1_cam1;
    int retval = sensor_interface_->GenerateImuXform(imu_msgs, R_imu_cam0_, R_imu_cam1_, rotation_t0_t1_cam0,
                                                     current_time, rotation_t0_t1_cam1);

    // Append the kepypoints from the previous iteration to the tracked points of the current
    // iteration. Points that pass through all the outlier checks will remain as tracked points
    points.AppendGpuMatColwise(d_c0_t0, t_c0_t0);
    points.AppendGpuMatColwise(d_c1_t0, t_c1_t0);
    points.ids[ids_t0].insert(std::end(points.ids[ids_t0]), std::begin(points.ids[ids_t1]),
                              std::end(points.ids[ids_t1]));

    if (points[t_c0_t0].cols > 2) {
      // Vector to indicate whether a point should be deleted or kept after binning
      std::vector<unsigned char> bin_status;
      points.BinAndMarkPoints(t_c0_t0, ids_t0, d_frame_cam0_t1.size(), bins_width_, bins_height_, max_pts_in_bin_,
                              bin_status);
      if (!points.RemoveOutliers(bin_status, {t_c0_t0, t_c1_t0}, {ids_t0})) {
        spdlog::info("status size {}", bin_status.size());
        spdlog::info("t_c0_t0 size {}", points.GetCpu(t_c0_t0).size());
        spdlog::info("RemoveOutliers failed after Binning");
        return -1;
      }
    }

    // Make a prediction of where the points will be in the current image given the IMU data
    if (retval == 0) {
      UpdatePointsViaImu(points[t_c0_t0], rotation_t0_t1_cam0, K_cam0_, points[t_c0_t1]);
      UpdatePointsViaImu(points[t_c1_t0], rotation_t0_t1_cam1, K_cam1_, points[t_c1_t1]);
    } else {
      spdlog::error("Error receiving IMU transform");
      continue;
    }

    // debug_num_pts[0] = points[t_c0_t0].cols;
    /*********************************************************************
     * Apply the optical flow from the previous frame to the current frame
     *********************************************************************/
    if (counter++ != 0 && !points[t_c0_t0].empty()) {
      // // DEBUG CODE
      // std::vector<cv::Point2f> pts_t0;
      // write_to_debug = true;
      // if (write_to_debug) {
      //   cv::Mat frame0;
      //   d_frame_cam0_t0.download(frame0);
      //   if (points[t_c0_t0].cols > 0) {
      //     points[t_c0_t0].download(pts_t0);
      //   }
      //   debug_record.DrawPts(frame0, 0, pts_t0);
      // }
      // std::vector<cv::Point2f> pts_t1_debug;
      // points[t_c0_t1].download(pts_t1_debug);
      // // END DEBUG CODE

      cv::cuda::GpuMat d_status;
      d_opt_flow_cam0->calc(d_frame_cam0_t0, d_frame_cam0_t1, points[t_c0_t0], points[t_c0_t1], d_status);

      // Check if we have zero tracked points, this is a corner case and these variables need
      // to be reset to prevent errors in sizing, calc() does not return all zero length vectors
      if (points[t_c0_t1].empty()) {
        points[t_c0_t0].release();
        points[t_c0_t1].release();
        d_status.release();
      }

      if (!points.MarkPointsOutOfFrame(d_frame_cam0_t1.size(), t_c0_t1, d_status)) {
        spdlog::info("MarkPointsOutOfFrame failed after tracking");
        spdlog::info("status size: {}");
        return -1;
      }

      // TODO: Make this function accept a vector (non gpu) for first arg and templated vector as
      // the second argument
      if (!points.RemoveOutliers(d_status, {t_c0_t0, t_c0_t1, t_c1_t0}, {ids_t0})) {
        spdlog::info("RemoveOutliers failed after Tracking");
        return -1;
      }
      // debug_num_pts[1] = points[t_c0_t1].cols;

      // // DEBUG CODE
      // if (write_to_debug) {
      //   cv::Mat frame1;
      //   std::vector<cv::Point2f> pts_t1;
      //   d_frame_cam0_t1.download(frame1);
      //   if (points[t_c0_t1].cols > 0) {
      //     points[t_c0_t1].download(pts_t1);
      //   }
      //   debug_record.DrawPts(frame1, 1, pts_t0, pts_t1_debug, pts_t1);
      //   debug_record.WriteFrame();
      // }
      // // END DEBUG CODE

      // Match the points from camera 0 to camera 1
      int ret_val =
          StereoMatch(d_opt_flow_stereo_t0, d_frame_cam0_t1, d_frame_cam1_t1, points, {t_c0_t1, t_c1_t1}, d_status);
      // Remove the outliers from the StereoMatch algorithm
      if (ret_val == 0) {
        if (!points.RemoveOutliers(d_status, {t_c0_t0, t_c0_t1, t_c1_t0, t_c1_t1}, {ids_t0})) {
          spdlog::info("RemoveOutliers failed after StereoMatch");
          return -1;
        }
      }
      // debug_num_pts[2] = points[t_c0_t1].cols;

      // Perform a check to make sure that all of our tracked vectors are the same length,
      // otherwise something is wrong
      int tracked_pts = points[t_c0_t0].cols;
      if (tracked_pts != points[t_c0_t1].cols || tracked_pts != points[t_c1_t0].cols ||
          tracked_pts != points[t_c1_t1].cols) {
        spdlog::error("ERROR! There are differences in the numbers of tracked points ");
        return -1;
      }
    }

    t_lk1 = std::chrono::system_clock::now();

    /*********************************************************************
     * Run the Two Point RANSAC algorithm to find more outliers
     *********************************************************************/
    if (points[t_c0_t0].cols > 1 && points[t_c1_t0].cols > 1 && 0) {
      // Copy local versions for the CPU run ransac algorithm
      std::vector<cv::Point2f> vec_t_c0_t0 = points.GetCpu(t_c0_t0);
      std::vector<cv::Point2f> vec_t_c0_t1 = points.GetCpu(t_c0_t1);
      std::vector<cv::Point2f> vec_t_c1_t0 = points.GetCpu(t_c1_t0);
      std::vector<cv::Point2f> vec_t_c1_t1 = points.GetCpu(t_c1_t1);

      std::vector<unsigned char> cam0_ransac_inliers(0);
      int ret1 = twoPointRansac(vec_t_c0_t0, vec_t_c0_t1, rotation_t0_t1_cam0.t(), K_cam0_, D_cam0_, ransac_threshold_,
                                0.99, cam0_ransac_inliers);

      std::vector<unsigned char> cam1_ransac_inliers(0);
      int ret2 = twoPointRansac(vec_t_c1_t0, vec_t_c1_t1, rotation_t0_t1_cam1.t(), K_cam1_, D_cam1_, ransac_threshold_,
                                0.99, cam1_ransac_inliers);

      if (ret1 == 0 && ret2 == 0) {
        // spdlog::info(" before RANSAC " << tracked_pts_cam0_t1.size());
        std::vector<unsigned char> status;
        status.reserve(cam0_ransac_inliers.size());

        for (size_t i = 0; i < cam0_ransac_inliers.size(); i++) {
          status.push_back(cam0_ransac_inliers[i] && cam1_ransac_inliers[i]);
        }

        if (!points.RemoveOutliers(status, {t_c0_t0, t_c0_t1, t_c1_t0, t_c1_t1}, {ids_t0})) {
          spdlog::info("RemoveOutliers failed after Ransac");
          spdlog::info("RemoveOutliers failed after Detection");
        }
        // spdlog::info("vec after {}", points.GetCpu(t_c0_t0).size());
        // spdlog::info(" after RANSAC {}", tracked_pts_cam0_t1.size());
      }
    }
    // debug_num_pts[3] = points.GetCpu(t_c0_t1).size();

    /*********************************************************************
     * Detect new features for the next iteration
     *********************************************************************/
    DetectNewFeatures(detector_ptr, d_frame_cam0_t1, points[t_c0_t1], points[d_c0_t1]);
    // debug_num_pts[4] = points[d_c0_t1].cols;
    // Match the detected features in the second camera
    points[d_c1_t1].release();
    cv::cuda::GpuMat d_status;
    StereoMatch(d_opt_flow_stereo_t1, d_frame_cam0_t1, d_frame_cam1_t1, points, {d_c0_t1, d_c1_t1}, d_status);

    if (points[d_c0_t1].cols != 0) {
      if (!points.MarkPointsOutOfFrame(d_frame_cam0_t1.size(), d_c1_t1, d_status)) {
        spdlog::info("d status: {}", d_status.cols);
        spdlog::info("vec: {}", points[d_c1_t1].cols);
        spdlog::info("MarkPointsOutOfFrame failed after detection");
        return -1;
      }

      if (!points.RemoveOutliers(d_status, {d_c0_t1, d_c1_t1})) {
        spdlog::info("d status: {}", d_status.cols);
        spdlog::info("vec: {}", points[d_c0_t1].cols);
        spdlog::info("vec: {}", points[d_c1_t1].cols);
        spdlog::info("RemoveOutliers failed after Detection");
        return -1;
      }
      // debug_num_pts[5] = points[d_c0_t1].cols;
    }
    t_det = std::chrono::system_clock::now();

    // Fill in the ids vector with our new detected features
    points.ids[ids_t1].clear();
    points.ids[ids_t1].reserve(points[d_c0_t1].cols);

    for (int i = 0; i < points[d_c0_t1].cols; i++) {
      points.ids[ids_t1].push_back(current_id++);
    }

    // spdlog::info("num points: cam0 " << points[1).cols << " cam1 " <<
    //   points[3).cols << " start: " << debug_num_pts[0] << " tracking: " <<
    //   debug_num_pts[1] << " matching: " << debug_num_pts[2] << " ransac: " << debug_num_pts[3]
    //    << " detection: " << debug_num_pts[4] << " detection matching: " << debug_num_pts[5]
    //   << std::endl;

    // Signal this loop has finished and output the tracked points
    if (points.GetCpu(t_c0_t1).size() > 1 && points.GetCpu(t_c1_t1).size() > 1) {
      OuputTrackedPoints(points, points.ids[ids_t0], imu_msgs, rotation_t0_t1_cam0);
    }

    /*********************************************************************
     * Output Images to the sink, if requested
     *********************************************************************/
    if (sensor_interface_->cam0_->OutputEnabled()) {
      cv::Mat show_frame, show_frame_color;
      d_frame_cam0_t1.download(show_frame);
      if (draw_points_to_frame_) {
        cv::cvtColor(show_frame, show_frame_color, cv::COLOR_GRAY2BGR);
        sensor_interface_->DrawPoints(points.GetCpu(t_c0_t1), show_frame_color);
      } else {
        show_frame_color = show_frame;
      }
      // DrawPoints(debug_pts, show_frame);
      sensor_interface_->cam0_->SendFrame(show_frame_color);
    }
    if (sensor_interface_->cam1_->OutputEnabled()) {
      cv::Mat show_frame, show_frame_color;
      d_frame_cam1_t1.download(show_frame);
      if (draw_points_to_frame_) {
        cv::cvtColor(show_frame, show_frame_color, cv::COLOR_GRAY2BGR);
        sensor_interface_->DrawPoints(points.GetCpu(t_c1_t1), show_frame_color);
      } else {
        show_frame_color = show_frame;
      }
      sensor_interface_->cam1_->SendFrame(show_frame_color);
    }

    // Set the current t1 values to t0 for the next iteration
    d_frame_cam0_t0 = d_frame_cam0_t1.clone();
    d_frame_cam1_t0 = d_frame_cam1_t1.clone();
    points[t_c0_t0] = points[t_c0_t1].clone();
    points[t_c1_t0] = points[t_c1_t1].clone();
    points[d_c0_t0] = points[d_c0_t1].clone();
    points[d_c1_t0] = points[d_c1_t1].clone();

    t_log = std::chrono::system_clock::now();
    spdlog::trace("ip dts, data: {}, lk1: {}, det: {}, log {}", (t_data - t_start).count() / 1E6,
                  (t_lk1 - t_data).count() / 1E6, (t_det - t_lk1).count() / 1E6, (t_log - t_det).count() / 1E6);

    // Apply the rate limiting
    std::this_thread::sleep_until(time_end + invFpsLimit);

    spdlog::trace(
        "fps: {}",
        1.0 / static_cast<std::chrono::duration<double>>((std::chrono::system_clock::now() - time_end)).count());
    time_end = std::chrono::system_clock::now();
  }
  return 0;
}

int ImageProcessor::StereoMatch(cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> opt, cv::cuda::GpuMat &d_frame_cam0,
                                cv::cuda::GpuMat &d_frame_cam1, OpticalFlowPoints &points,
                                std::pair<unsigned int, unsigned int> indices, cv::cuda::GpuMat &d_status) {
  if (points[indices.first].empty()) {
    points[indices.second].release();
    d_status.release();
    return 1;
  }

  // Using the stereo calibration, project points from one camera onto the other
  std::vector<cv::Point2f> calib_pts;
  cv::undistortPoints(points.GetCpu(indices.first), calib_pts, K_cam0_, D_cam0_, R_cam0_cam1_);
  std::vector<cv::Point3f> homogenous_pts;
  cv::convertPointsToHomogeneous(calib_pts, homogenous_pts);
  std::vector<cv::Point2f> projected_pts;
  cv::projectPoints(homogenous_pts, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K_cam1_, D_cam1_, projected_pts);

  // Load the local variables back into the points object
  points.LoadCpu(projected_pts, indices.second);

  opt->calc(d_frame_cam0, d_frame_cam1, points[indices.first], points[indices.second], d_status);

  // Check if we have zero tracked points, this is a corner case and these variables need
  // to be reset to prevent errors in sizing, calc() does not return all zero length vectors
  if (points[indices.second].empty()) {
    points[indices.second].release();
    points[indices.first].release();
    d_status.release();
    return 0;
  }

  // Mark the points out of the frame
  if (!points.MarkPointsOutOfFrame(d_frame_cam1.size(), indices.second, d_status)) {
    spdlog::info("RemovePointsOutOfFrame failed 2");
    return -1;
  }

  std::vector<cv::Point2f> tracked_pts_cam0_t1_undistorted(0);
  std::vector<cv::Point2f> tracked_pts_cam1_t1_undistorted(0);
  cv::undistortPoints(points.GetCpu(indices.first), tracked_pts_cam0_t1_undistorted, K_cam0_, D_cam0_);
  cv::undistortPoints(points.GetCpu(indices.second), tracked_pts_cam1_t1_undistorted, K_cam1_, D_cam1_);

  std::vector<cv::Vec3f> pts_cam0_und(tracked_pts_cam0_t1_undistorted.size());
  std::vector<cv::Vec3f> pts_cam1_und(tracked_pts_cam1_t1_undistorted.size());
  for (size_t i = 0; i < tracked_pts_cam0_t1_undistorted.size(); i++) {
    cv::Vec3f tmp0(tracked_pts_cam0_t1_undistorted[i].x, tracked_pts_cam0_t1_undistorted[i].y, 1);
    pts_cam0_und[i] = (K_cam0_ * tmp0);
    cv::Vec3f tmp1(tracked_pts_cam1_t1_undistorted[i].x, tracked_pts_cam1_t1_undistorted[i].y, 1);
    pts_cam1_und[i] = (K_cam1_ * tmp1);
  }

  cv::convertPointsFromHomogeneous(pts_cam0_und, tracked_pts_cam0_t1_undistorted);
  cv::convertPointsFromHomogeneous(pts_cam1_und, tracked_pts_cam1_t1_undistorted);
  std::vector<cv::Vec3f> epilines;
  cv::computeCorrespondEpilines(tracked_pts_cam0_t1_undistorted, 1, F_, epilines);

  std::vector<unsigned char> status;
  d_status.download(status);
  for (size_t i = 0; i < epilines.size(); i++) {
    if (status[i] == 0) {
      continue;
    }
    // cv::Vec3f pt0(tracked_pts_cam0_t1_undistorted[i].x,
    //   tracked_pts_cam0_t1_undistorted[i].y, 1.0);

    cv::Vec3f pt1(tracked_pts_cam1_t1_undistorted[i].x, tracked_pts_cam1_t1_undistorted[i].y, 1.0);

    // Calculates the distance from the point to the epipolar line (in pixels)
    double error = fabs(pt1.dot(epilines[i]));

    if (error > stereo_threshold_) status[i] = 0;
  }
  d_status.upload(status);
  return 0;
}

int ImageProcessor::GetInputMaskFromPoints(const cv::cuda::GpuMat &d_input_corners, const cv::Size frame_size,
                                           cv::Mat &mask) {
  double mask_box_size = 15.0;
  cv::Mat local_mask(frame_size, CV_8U, cv::Scalar(1));

  if (d_input_corners.cols > 0) {
    std::vector<cv::Point2f> corners;
    d_input_corners.download(corners);
    for (const auto &point : corners) {
      const int x = static_cast<int>(point.x);
      const int y = static_cast<int>(point.y);

      int up_lim = y - floor(mask_box_size / 2);
      int bottom_lim = y + ceil(mask_box_size / 2);
      int left_lim = x - floor(mask_box_size / 2);
      int right_lim = x + ceil(mask_box_size / 2);
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
                                      const cv::cuda::GpuMat &d_frame, const cv::cuda::GpuMat &d_input_corners,
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
  return 0;
}

int ImageProcessor::DetectNewFeatures(const cv::Ptr<cv::cuda::CornersDetector> &detector_ptr,
                                      const cv::cuda::GpuMat &d_frame, const cv::cuda::GpuMat &d_input_corners,
                                      cv::cuda::GpuMat &d_output) {
  // Create a mask to avoid redetecting existing features.

  cv::Mat mask;
  GetInputMaskFromPoints(d_input_corners, d_frame.size(), mask);

  // Find features in the cam1 for tracking
  cv::cuda::GpuMat d_mask(mask);

  detector_ptr->detect(d_frame, d_output, d_mask);
  return 0;
}

void ImageProcessor::rescalePoints(std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2,
                                   float &scaling_factor) {
  scaling_factor = 0.0f;

  for (size_t i = 0; i < pts1.size(); ++i) {
    scaling_factor += sqrt(pts1[i].dot(pts1[i]));
    scaling_factor += sqrt(pts2[i].dot(pts2[i]));
  }

  scaling_factor = (pts1.size() + pts2.size()) / scaling_factor * sqrt(2.0f);

  for (size_t i = 0; i < pts1.size(); ++i) {
    pts1[i] *= scaling_factor;
    pts2[i] *= scaling_factor;
  }
}

int ImageProcessor::twoPointRansac(const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2,
                                   const cv::Matx33f &R_p_c, const cv::Matx33d &intrinsics,
                                   const std::vector<double> &distortion_coeffs, const double &inlier_error,
                                   const double &success_probability, std::vector<uchar> &inlier_markers) {
  // Check the size of input point size.
  if (pts1.size() != pts2.size()) {
    spdlog::error("Sets of different size ({}) and ({}) are used...", pts1.size(), pts2.size());
    return -1;
  }

  double norm_pixel_unit = 2.0 / (intrinsics(0, 0) + intrinsics(1, 1));
  int iter_num = static_cast<int>(ceil(log(1 - success_probability) / log(1 - 0.7 * 0.7)));

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
  for (auto &pt : pts1_undistorted) {
    cv::Vec3f pt_h(pt.x, pt.y, 1.0f);
    // Vec3f pt_hc = dR * pt_h;
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
  for (size_t i = 0; i < pts1_undistorted.size(); ++i) pts_diff[i] = pts1_undistorted[i] - pts2_undistorted[i];

  // Mark the point pairs with large difference directly.
  // BTW, the mean distance of the rest of the point pairs
  // are computed.
  double mean_pt_distance = 0.0;
  int raw_inlier_cntr = 0;
  for (size_t i = 0; i < pts_diff.size(); ++i) {
    double distance = sqrt(pts_diff[i].dot(pts_diff[i]));
    // 25 pixel distance is a pretty large tolerance for normal motion.
    // However, to be used with aggressive motion, this tolerance should
    // be increased significantly to match the usage.
    if (distance > 50.0 * norm_pixel_unit) {
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
    for (auto &marker : inlier_markers) marker = 0;
    return 0;
  }

  // Before doing 2-point RANSAC, we have to check if the motion
  // is degenerated, meaning that there is no translation between
  // the frames, in which case, the model of the RANSAC does not
  // work. If so, the distance between the matched points will
  // be almost 0.
  // if (mean_pt_distance < inlier_error*norm_pixel_unit) {
  if (mean_pt_distance < norm_pixel_unit) {
    // ROS_WARN_THROTTLE(1.0, "Degenerated motion...");
    for (size_t i = 0; i < pts_diff.size(); ++i) {
      if (inlier_markers[i] == 0) continue;
      if (sqrt(pts_diff[i].dot(pts_diff[i])) > inlier_error * norm_pixel_unit) inlier_markers[i] = 0;
    }
    return 0;
  }

  // In the case of general motion, the RANSAC model can be applied.
  // The three column corresponds to tx, ty, and tz respectively.
  Eigen::MatrixXd coeff_t(pts_diff.size(), 3);
  for (size_t i = 0; i < pts_diff.size(); ++i) {
    coeff_t(i, 0) = pts_diff[i].y;
    coeff_t(i, 1) = -pts_diff[i].x;
    coeff_t(i, 2) = pts1_undistorted[i].x * pts2_undistorted[i].y - pts1_undistorted[i].y * pts2_undistorted[i].x;
  }

  std::vector<int> raw_inlier_idx;
  for (size_t i = 0; i < inlier_markers.size(); ++i) {
    if (inlier_markers[i] != 0) raw_inlier_idx.push_back(i);
  }

  std::vector<int> best_inlier_set;
  // double best_error = 1e10;

  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution0(0, raw_inlier_idx.size() - 1);
  std::uniform_int_distribution<int> distribution1(1, raw_inlier_idx.size() - 1);

  for (int iter_idx = 0; iter_idx < iter_num; ++iter_idx) {
    // Randomly select two point pairs.
    // Although this is a weird way of selecting two pairs, but it
    // is able to efficiently avoid selecting repetitive pairs.
    int select_idx1 = distribution0(generator);
    int select_idx_diff = distribution1(generator);
    int select_idx2 = static_cast<size_t>(select_idx1 + select_idx_diff) < raw_inlier_idx.size()
                          ? select_idx1 + select_idx_diff
                          : select_idx1 + select_idx_diff - raw_inlier_idx.size();

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
    int base_indicator = min_element(coeff_l1_norm.begin(), coeff_l1_norm.end()) - coeff_l1_norm.begin();

    Eigen::Vector3d model(0.0, 0.0, 0.0);
    if (base_indicator == 0) {
      Eigen::Matrix2d A;
      A << coeff_ty, coeff_tz;
      Eigen::Vector2d solution = A.inverse() * (-coeff_tx);
      model(0) = 1.0;
      model(1) = solution(0);
      model(2) = solution(1);
    } else if (base_indicator == 1) {
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
      if (std::abs(error(i)) < inlier_error * norm_pixel_unit) inlier_set.push_back(i);
    }

    // If the number of inliers is small, the current
    // model is probably wrong.
    if (inlier_set.size() < 0.2 * pts1_undistorted.size()) continue;

    // Refit the model using all of the possible inliers.
    Eigen::VectorXd coeff_tx_better(inlier_set.size());
    Eigen::VectorXd coeff_ty_better(inlier_set.size());
    Eigen::VectorXd coeff_tz_better(inlier_set.size());
    for (size_t i = 0; i < inlier_set.size(); ++i) {
      coeff_tx_better(i) = coeff_t(inlier_set[i], 0);
      coeff_ty_better(i) = coeff_t(inlier_set[i], 1);
      coeff_tz_better(i) = coeff_t(inlier_set[i], 2);
    }

    Eigen::Vector3d model_better(0.0, 0.0, 0.0);
    if (base_indicator == 0) {
      Eigen::MatrixXd A(inlier_set.size(), 2);
      A << coeff_ty_better, coeff_tz_better;
      Eigen::Vector2d solution = (A.transpose() * A).inverse() * A.transpose() * (-coeff_tx_better);
      model_better(0) = 1.0;
      model_better(1) = solution(0);
      model_better(2) = solution(1);
    } else if (base_indicator == 1) {
      Eigen::MatrixXd A(inlier_set.size(), 2);
      A << coeff_tx_better, coeff_tz_better;
      Eigen::Vector2d solution = (A.transpose() * A).inverse() * A.transpose() * (-coeff_ty_better);
      model_better(0) = solution(0);
      model_better(1) = 1.0;
      model_better(2) = solution(1);
    } else {
      Eigen::MatrixXd A(inlier_set.size(), 2);
      A << coeff_tx_better, coeff_ty_better;
      Eigen::Vector2d solution = (A.transpose() * A).inverse() * A.transpose() * (-coeff_tz_better);
      model_better(0) = solution(0);
      model_better(1) = solution(1);
      model_better(2) = 1.0;
    }

    // Compute the error and upate the best model if possible.
    Eigen::VectorXd new_error = coeff_t * model_better;

    double this_error = 0.0;
    for (const auto &inlier_idx : inlier_set) this_error += std::abs(new_error(inlier_idx));
    this_error /= inlier_set.size();

    if (inlier_set.size() > best_inlier_set.size()) {
      // best_error = this_error;
      best_inlier_set = inlier_set;
    }
  }

  // Fill in the markers.
  inlier_markers.clear();
  inlier_markers.resize(pts1.size(), 0);
  for (const auto &inlier_idx : best_inlier_set) inlier_markers[inlier_idx] = 1;

  // printf("inlier ratio: %lu/%lu\n",
  //    best_inlier_set.size(), inlier_markers.size());

  return 0;
}

void ImageProcessor::ReceiveImu(const mavlink_imu_t &msg) { sensor_interface_->ReceiveImu(msg); }
