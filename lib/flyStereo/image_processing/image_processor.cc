#include "flyStereo/image_processing/image_processor.h"

#include <fstream>
#include <random>

#include "Eigen/Dense"
#include "flyStereo/utility.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
// #include "opengv/types.hpp"
#include "spdlog/spdlog.h"

// Debug includes
#include "flyStereo/debug_video_recorder.h"

// Debug test function
template <typename OptFlowCalculator = OptFlowVpiGpu>
double print_percent_status(UMat<uint8_t> &status) {
  int counter = std::accumulate(status.frame().begin(), status.frame().end(), 0, [](int sum, uint8_t val) {
    if (val == OptFlowCalculator::success_value) {
      return sum + 1;
    } else {
      return sum;
    }
  });
  return static_cast<double>(counter) / status.frame().cols;
}

// Compile-time constants
struct ImageProcessorConstants {
  static constexpr bool write_to_debug = false;

  // calcOptFlowPryLK
  static constexpr int window_size = 21;
  static constexpr int max_pyramid_level = 3;
  static constexpr int max_iters = 30;
  static constexpr bool use_initial_flow = true;

  // Binning
  static constexpr unsigned int bins_width = 20;
  static constexpr unsigned int bins_height = 15;
  static constexpr unsigned int max_pts_in_bin = 30;

  // goodFeraturesToTrack
  static constexpr int max_corners = 300;
  static constexpr float quality_level = 0.15f;
  static constexpr float min_dist = 15.f;

  // Thresholds
  static constexpr double stereo_threshold = 45.0;
  static constexpr double ransac_threshold = 10.0;
};

debug_video_recorder debug_record(true);

template <typename OptFlowCalculator>
ImageProcessor<OptFlowCalculator>::ImageProcessor(float rate_limit_fps, const YAML::Node &stereo_calibration,
                                                  cv::Matx33d R_imu_cam0)
    : ImageProcessor(rate_limit_fps, StereoCalibration(stereo_calibration), R_imu_cam0) {}

template <typename OptFlowCalculator>
ImageProcessor<OptFlowCalculator>::ImageProcessor(float rate_limit_fps, StereoCalibration stereo_calibration,
                                                  cv::Matx33d R_imu_cam0)
    : rate_limit_fps_(rate_limit_fps),
      is_rate_limiting_(rate_limit_fps > 0.f),
      stereo_cal_(stereo_calibration),
      R_imu_cam0_(R_imu_cam0) {
  pts_t_c0_t0_ = UMat<cv::Vec2f>(cv::Size(0, 1));
  pts_t_c0_t1_ = UMat<cv::Vec2f>(cv::Size(0, 1));
  pts_t_c1_t0_ = UMat<cv::Vec2f>(cv::Size(0, 1));
  pts_t_c1_t1_ = UMat<cv::Vec2f>(cv::Size(0, 1));
  pts_d_c0_t0_ = UMat<cv::Vec2f>(cv::Size(0, 1));
  pts_d_c0_t1_ = UMat<cv::Vec2f>(cv::Size(0, 1));
  pts_d_c1_t0_ = UMat<cv::Vec2f>(cv::Size(0, 1));
  pts_d_c1_t1_ = UMat<cv::Vec2f>(cv::Size(0, 1));

  // cv::Matx33f stereo_cal_.R_cam0_cam1 = stereo_cal_.R_cam0_cam1;
  R_imu_cam1_ = R_imu_cam0_ * cv::Matx33f(stereo_cal_.R_cam0_cam1);

  pose_ = Eigen::Matrix4d::Identity();
  prev_xform_ = Eigen::Matrix4d::Identity();
}

template <typename OptFlowCalculator>
void ImageProcessor<OptFlowCalculator>::Init() {
  detector_ptr_ = cv::cuda::createGoodFeaturesToTrackDetector(CV_8U, ImageProcessorConstants::max_corners,
                                                              ImageProcessorConstants::quality_level,
                                                              ImageProcessorConstants::min_dist);

  d_opt_flow_cam0_ = OptFlowCalculator(ImageProcessorConstants::window_size, ImageProcessorConstants::max_pyramid_level,
                                       ImageProcessorConstants::max_iters, ImageProcessorConstants::use_initial_flow);
  d_opt_flow_stereo_t0_ =
      OptFlowCalculator(ImageProcessorConstants::window_size, ImageProcessorConstants::max_pyramid_level,
                        ImageProcessorConstants::max_iters, ImageProcessorConstants::use_initial_flow);
  d_opt_flow_stereo_t1_ =
      OptFlowCalculator(ImageProcessorConstants::window_size, ImageProcessorConstants::max_pyramid_level,
                        ImageProcessorConstants::max_iters, ImageProcessorConstants::use_initial_flow);
  fps_limit_inv_ =
      std::chrono::round<std::chrono::system_clock::duration>(std::chrono::duration<double>(1. / rate_limit_fps_));
}

template <typename OptFlowCalculator>
int ImageProcessor<OptFlowCalculator>::UpdatePointsViaImu(const UMat<cv::Vec2f> &current_pts,
                                                          const cv::Matx33d &rotation, const cv::Matx33d &camera_matrix,
                                                          UMat<cv::Vec2f> &updated_pts) {
  if (current_pts.frame().rows != 1 && current_pts.frame().channels() != 2) {
    throw std::runtime_error("ImageProcessor::UpdatePointsViaImu: current_pts must be 1xNx2");
  }
  if (current_pts.frame().cols == 0) {
    return -1;
  }

  cv::Matx33f H = camera_matrix * rotation.t() * camera_matrix.inv();

  std::vector<cv::Point3f> interface_pts;
  cv::convertPointsToHomogeneous(current_pts.frame(), interface_pts);
  std::for_each(interface_pts.begin(), interface_pts.end(), [&H](auto &pt) { pt = H * pt; });

  cv::Mat updated_pts_mat;
  cv::convertPointsFromHomogeneous(interface_pts, updated_pts_mat);
  updated_pts = std::move(updated_pts_mat.t());

  return 0;
}

template <typename OptFlowCalculator>
TrackedImagePoints ImageProcessor<OptFlowCalculator>::OuputTrackedPoints(const std::vector<mavlink_imu_t> &imu_msgs,
                                                                         const cv::Matx33f &rotation_t0_t1_cam0) {
  const cv::Size pts_size = pts_t_c0_t0_.frame().size();
  if (pts_size != pts_t_c0_t1_.frame().size() || pts_size != pts_t_c1_t0_.frame().size() ||
      pts_size != pts_t_c1_t1_.frame().size() || pts_size.height != 1 ||
      pts_size.width != static_cast<int>(ids_pts_tracked_.size())) {
    throw std::runtime_error("Error! Vectors do not match in OutputTrackedPoints");
  }

  uint64_t timestamp_us =
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
          .count();

  Eigen::Matrix3f rotation_t0_t1_cam0_eigen;
  cv::cv2eigen(rotation_t0_t1_cam0, rotation_t0_t1_cam0_eigen);

  return TrackedImagePoints(timestamp_us, ids_pts_tracked_, pts_t_c0_t0_, pts_t_c0_t1_, pts_t_c1_t0_, pts_t_c1_t1_,
                            imu_msgs, rotation_t0_t1_cam0_eigen);
}

// TODO make a check that the object is initialized before starting
template <typename OptFlowCalculator>
int ImageProcessor<OptFlowCalculator>::process_image(UMat<uint8_t> &d_frame_cam0_t1, UMat<uint8_t> &d_frame_cam1_t1,
                                                     const std::vector<mavlink_imu_t> &imu_msgs,
                                                     uint64_t current_frame_time, TrackedImagePoints &tracked_points) {
  // Time checks for performance monitoring. TODO: Put these in a dedicated class
  auto t_start = std::chrono::system_clock::now();
  auto t_data = std::chrono::system_clock::now();
  auto t_lk1 = std::chrono::system_clock::now();
  auto t_det = std::chrono::system_clock::now();
  auto t_log = std::chrono::system_clock::now();

  cv::Matx33f rotation_t0_t1_cam0;
  cv::Matx33f rotation_t0_t1_cam1;
  int retval = SensorInterface::GenerateImuXform(imu_msgs, R_imu_cam0_, R_imu_cam1_, rotation_t0_t1_cam0,
                                                 current_frame_time, rotation_t0_t1_cam1);

  // Append the kepypoints from the previous iteration to the tracked points of the current
  // iteration. Points that pass through all the outlier checks will remain as tracked points
  pts_t_c0_t0_ = AppendUMatColwise(pts_t_c0_t0_, pts_d_c0_t0_);
  pts_t_c1_t0_ = AppendUMatColwise(pts_t_c1_t0_, pts_d_c1_t0_);

  ids_pts_tracked_.insert(std::end(ids_pts_tracked_), std::begin(ids_pts_detected_), std::end(ids_pts_detected_));

  // Remove the points the exceed the binning threshold
  if (pts_t_c0_t0_.frame().cols > 2) {
    // Vector to indicate whether a point should be deleted or kept after binning
    UMat<uint8_t> bin_status;
    BinAndMarkPoints(pts_t_c0_t0_, ids_pts_tracked_, d_frame_cam0_t1.frame().size(),
                     cv::Size(ImageProcessorConstants::bins_width, ImageProcessorConstants::bins_height),
                     ImageProcessorConstants::max_pts_in_bin, bin_status, OptFlowCalculator::success_value);

    RemovePoints(bin_status, pts_t_c0_t0_, OptFlowCalculator::success_value);
    RemovePoints(bin_status, pts_t_c1_t0_, OptFlowCalculator::success_value);
    RemovePoints(bin_status, ids_pts_tracked_, OptFlowCalculator::success_value);
  }

  // Make a prediction of where the points will be in the current image given the IMU data
  if (retval == 0) {
    // TODO Implement these functions for imu
    UpdatePointsViaImu(pts_t_c0_t0_, rotation_t0_t1_cam0, stereo_cal_.K_cam0, pts_t_c0_t1_);
    UpdatePointsViaImu(pts_t_c1_t0_, rotation_t0_t1_cam1, stereo_cal_.K_cam1, pts_t_c1_t1_);
  } else {
    spdlog::error("Error receiving IMU transform");
    return -1;
  }

  // Apply the optical flow from the previous frame to the current frame
  if (!pts_t_c0_t0_.frame().empty()) {
    UMat<uint8_t> status;

    // UMat<uint8_t> d_frame_cam0_t0_copy(d_frame_cam0_t0_);
    // UMat<uint8_t> d_frame_cam0_t1_copy(d_frame_cam0_t1);

    // auto copy_1 = pts_t_c0_t0_.clone();
    d_opt_flow_cam0_.calc(d_frame_cam0_t0_, d_frame_cam0_t1, pts_t_c0_t0_, pts_t_c0_t1_, status);

    // // DEBUG CODE
    // std::vector<cv::Point2f> pts_t0;
    // bool write_to_debug = true;
    // if (write_to_debug) {
    //   if (pts_t_c0_t0_.frame().cols > 0) {
    //     pts_t_c0_t0_.d_frame().download(pts_t0);
    //   }
    //   debug_record.DrawPts(d_frame_cam0_t0_.frame(), 0, pts_t0);
    // }
    // // std::vector<cv::Point2f> pts_t1_debug;
    // // points_[t_c0_t1].download(pts_t1_debug);
    // // END DEBUG CODE

    // // // DEBUG CODE
    // if (write_to_debug) {
    //   std::vector<cv::Point2f> pts_t1;
    //   if (pts_t_c0_t1_.d_frame().cols > 0) {
    //     pts_t_c0_t1_.d_frame().download(pts_t1);
    //   }
    //   debug_record.DrawPts(d_frame_cam0_t1.frame(), 1, pts_t1);
    //   debug_record.WriteFrame();
    // }
    // // // END DEBUG CODE

    // std::cout << "t0 opt-flow " << print_percent_status(status) << std::endl;

    MarkPointsOutOfFrame(status, pts_t_c0_t1_, d_frame_cam0_t1.frame().size(), OptFlowCalculator::success_value);
    // std::cout << "t0 mark " << print_percent_status(status) << std::endl;

    // std::cout << "t0 pts before " << pts_t_c0_t0_.frame().cols << std::endl;
    RemovePoints(status, pts_t_c0_t0_, OptFlowCalculator::success_value);
    // std::cout << "t0 pts after " << pts_t_c0_t0_.frame().cols << std::endl;
    RemovePoints(status, pts_t_c0_t1_, OptFlowCalculator::success_value);
    RemovePoints(status, pts_t_c1_t0_, OptFlowCalculator::success_value);
    RemovePoints(status, ids_pts_tracked_, OptFlowCalculator::success_value);

    // Match the points from camera 0 to camera 1
    // copy_2_ = pts_t_c0_t1_.clone();
    int ret_val =
        StereoMatch(d_opt_flow_stereo_t0_, d_frame_cam0_t1, d_frame_cam1_t1, pts_t_c0_t1_, pts_t_c1_t1_, status);

    // // DEBUG CODE
    // std::vector<cv::Point2f> pts_t0;
    // bool write_to_debug = true;
    // if (write_to_debug) {
    //   if (pts_t_c0_t1_.frame().cols > 0) {
    //     pts_t_c0_t1_.d_frame().download(pts_t0);
    //   }
    //   debug_record.DrawPts(d_frame_cam0_t1.frame(), 0, pts_t0);
    // }
    // // std::vector<cv::Point2f> pts_t1_debug;
    // // points_[t_c0_t1].download(pts_t1_debug);
    // // END DEBUG CODE

    // // // DEBUG CODE
    // if (write_to_debug) {
    //   std::vector<cv::Point2f> pts_t1;
    //   if (pts_t_c0_t1_.d_frame().cols > 0) {
    //     pts_t_c0_t1_.d_frame().download(pts_t1);
    //   }
    //   debug_record.DrawPts(d_frame_cam1_t1.frame(), 1, pts_t1);
    //   debug_record.WriteFrame();
    // }
    // // // END DEBUG CODE

    // Remove the outliers from the StereoMatch algorithm
    if (ret_val == 0) {
      // std::cout << "t pts before " << pts_t_c0_t1_.frame().cols << std::endl;
      RemovePoints(status, pts_t_c0_t0_, OptFlowCalculator::success_value);
      // std::cout << "t pts after " << pts_t_c0_t1_.frame().cols << std::endl;

      RemovePoints(status, pts_t_c0_t1_, OptFlowCalculator::success_value);
      RemovePoints(status, pts_t_c1_t0_, OptFlowCalculator::success_value);
      RemovePoints(status, pts_t_c1_t1_, OptFlowCalculator::success_value);
      RemovePoints(status, ids_pts_tracked_, OptFlowCalculator::success_value);
    } else {
      throw std::runtime_error("Error in stereo matching");
    }

    // Perform a check to make sure that all of our tracked vectors are the same length,
    // otherwise something is wrong
    int tracked_pts = pts_t_c0_t0_.frame().cols;
    if (tracked_pts != pts_t_c0_t1_.frame().cols || tracked_pts != pts_t_c1_t0_.frame().cols ||
        tracked_pts != pts_t_c1_t1_.frame().cols) {
      spdlog::error("ERROR! There are differences in the numbers of tracked points ");
      return -1;
    }
  }

  t_lk1 = std::chrono::system_clock::now();

  // debug_num_pts[3] = points_.GetCpu(t_c0_t1).size();

  /*********************************************************************
   * Detect new features for the next iterationq
   *********************************************************************/
  DetectNewFeatures<cv::cuda::CornersDetector>(detector_ptr_, d_frame_cam0_t1.d_frame(), pts_t_c0_t1_.d_frame(),
                                               pts_d_c0_t1_);
  // Match the detected features in the second camera
  pts_d_c1_t1_ = cv::Mat_<cv::Vec2d>();

  UMat<uint8_t> status;
  // copy_3_ = pts_d_c0_t1_.clone();
  StereoMatch(d_opt_flow_stereo_t1_, d_frame_cam0_t1, d_frame_cam1_t1, pts_d_c0_t1_, pts_d_c1_t1_, status);

  if (pts_d_c0_t1_.frame().cols != 0) {
    // std::cout << "d pts before " << pts_d_c0_t1_.frame().cols << std::endl;
    MarkPointsOutOfFrame(status, pts_d_c1_t1_, d_frame_cam1_t1.frame().size(), OptFlowCalculator::success_value);
    RemovePoints(status, pts_d_c0_t1_, OptFlowCalculator::success_value);
    // std::cout << "d pts after " << pts_d_c0_t1_.frame().cols << std::endl;
    RemovePoints(status, pts_d_c1_t1_, OptFlowCalculator::success_value);
  }

  t_det = std::chrono::system_clock::now();

  // Fill in the ids vector with our new detected features
  ids_pts_detected_.clear();
  ids_pts_detected_.reserve(pts_d_c0_t1_.frame().cols);

  for (int i = 0; i < pts_d_c0_t1_.frame().cols; i++) {
    ids_pts_detected_.push_back(current_id_++);
  }

  if (pts_t_c0_t1_.frame().cols > 1 && pts_t_c1_t1_.frame().cols > 1) {
    tracked_points = OuputTrackedPoints(imu_msgs, rotation_t0_t1_cam0);
  }

  // Set the current t1 values to t0 for the next iteration
  d_frame_cam0_t0_ = std::move(d_frame_cam0_t1);
  d_frame_cam1_t0_ = std::move(d_frame_cam1_t1);

  pts_t_c0_t0_ = std::move(pts_t_c0_t1_);
  pts_t_c0_t1_ = UMat<cv::Vec2f>(cv::Size(0, 1));
  pts_t_c1_t0_ = std::move(pts_t_c1_t1_);
  pts_t_c1_t1_ = UMat<cv::Vec2f>(cv::Size(0, 1));
  pts_d_c0_t0_ = std::move(pts_d_c0_t1_);
  pts_d_c0_t1_ = UMat<cv::Vec2f>(cv::Size(0, 1));
  pts_d_c1_t0_ = std::move(pts_d_c1_t1_);
  pts_d_c1_t1_ = UMat<cv::Vec2f>(cv::Size(0, 1));

  t_log = std::chrono::system_clock::now();
  spdlog::trace("ip dts, data: {}, lk1: {}, det: {}, log {}", (t_data - t_start).count() / 1E6,
                (t_lk1 - t_data).count() / 1E6, (t_det - t_lk1).count() / 1E6, (t_log - t_det).count() / 1E6);

  // Apply the rate limiting
  if (is_rate_limiting_) {
    std::this_thread::sleep_until(prev_time_end_ + fps_limit_inv_);
  }

  spdlog::trace(
      "fps: {}",
      1.0 / static_cast<std::chrono::duration<double>>((std::chrono::system_clock::now() - prev_time_end_)).count());
  prev_time_end_ = std::chrono::system_clock::now();
  return 0;
}

template <typename OptFlowCalculator>
int ImageProcessor<OptFlowCalculator>::StereoMatch(OptFlowCalculator &opt, const UMat<uint8_t> &d_frame_cam0,
                                                   const UMat<uint8_t> &d_frame_cam1, const UMat<cv::Vec2f> &pts_cam0,
                                                   UMat<cv::Vec2f> &pts_cam1, UMat<uint8_t> &status) {
  if (pts_cam0.frame().empty()) {
    return -1;
  }

  // Using the stereo calibration, project points from one camera onto the other
  std::vector<cv::Point2f> calib_pts;
  cv::undistortPoints(pts_cam0.frame(), calib_pts, stereo_cal_.K_cam0, stereo_cal_.D_cam0, stereo_cal_.R_cam0_cam1);
  std::vector<cv::Point3f> homogenous_pts;
  cv::convertPointsToHomogeneous(calib_pts, homogenous_pts);
  cv::Mat projected_pts;
  cv::projectPoints(homogenous_pts, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), stereo_cal_.K_cam1, stereo_cal_.D_cam1,
                    projected_pts);

  // TODO handle edge cases
  if (projected_pts.rows == 1) {
    pts_cam1 = projected_pts;
  } else if (projected_pts.cols == 1) {
    pts_cam1 = projected_pts.t();
  } else {
    throw std::runtime_error("Projected points have wrong dimensions");
  }

  opt.calc(d_frame_cam0, d_frame_cam1, pts_cam0, pts_cam1, status);

  // // DEBUG CODE
  // bool write_to_debug = true;
  // if (write_to_debug) {
  //   std::vector<cv::Point2f> pts_t0;
  //   if (pts_cam0.frame().cols > 0) {
  //     pts_cam0.d_frame().download(pts_t0);
  //   }
  //   debug_record.DrawPts(d_frame_cam0.frame(), 0, pts_t0);

  //   // SECOND FRAME
  //   std::vector<cv::Point2f> pts_t1;
  //   if (pts_cam1.d_frame().cols > 0) {
  //     pts_cam1.d_frame().download(pts_t1);
  //   }
  //   debug_record.DrawPts(d_frame_cam1.frame(), 1, pts_t1);
  //   debug_record.WriteFrame();
  // }
  // // // END DEBUG CODE

  // Check if we have zero tracked points, this is a corner case and these variables need
  // to be reset to prevent errors in sizing, calc() does not return all zero length vectors
  if (pts_cam1.frame().empty()) {
    return -1;
  }

  MarkPointsOutOfFrame(status, pts_cam1, d_frame_cam1.frame().size(), OptFlowCalculator::success_value);

  std::vector<cv::Point2f> tracked_pts_cam0_t1_undistorted(0);
  std::vector<cv::Point2f> tracked_pts_cam1_t1_undistorted(0);
  cv::undistortPoints(pts_cam0.frame(), tracked_pts_cam0_t1_undistorted, stereo_cal_.K_cam0, stereo_cal_.D_cam0);
  cv::undistortPoints(pts_cam1.frame(), tracked_pts_cam1_t1_undistorted, stereo_cal_.K_cam1, stereo_cal_.D_cam1);

  std::vector<cv::Vec3f> pts_cam0_und(tracked_pts_cam0_t1_undistorted.size());
  std::vector<cv::Vec3f> pts_cam1_und(tracked_pts_cam1_t1_undistorted.size());
  for (size_t i = 0; i < tracked_pts_cam0_t1_undistorted.size(); i++) {
    cv::Vec3f tmp0(tracked_pts_cam0_t1_undistorted[i].x, tracked_pts_cam0_t1_undistorted[i].y, 1);
    pts_cam0_und[i] = (stereo_cal_.K_cam0 * tmp0);
    cv::Vec3f tmp1(tracked_pts_cam1_t1_undistorted[i].x, tracked_pts_cam1_t1_undistorted[i].y, 1);
    pts_cam1_und[i] = (stereo_cal_.K_cam1 * tmp1);
  }

  cv::convertPointsFromHomogeneous(pts_cam0_und, tracked_pts_cam0_t1_undistorted);
  cv::convertPointsFromHomogeneous(pts_cam1_und, tracked_pts_cam1_t1_undistorted);
  std::vector<cv::Vec3f> epilines;
  cv::computeCorrespondEpilines(tracked_pts_cam0_t1_undistorted, 1, stereo_cal_.F, epilines);

  auto &status_f = status.frame();
  for (size_t i = 0; i < epilines.size(); i++) {
    if (status_f(0, i) != OptFlowCalculator::success_value) {
      continue;
    }

    cv::Vec3f pt1(tracked_pts_cam1_t1_undistorted[i].x, tracked_pts_cam1_t1_undistorted[i].y, 1.0);

    // Calculates the distance from the point to the epipolar line (in pixels)
    double error = fabs(pt1.dot(epilines[i]));

    if (error > ImageProcessorConstants::stereo_threshold) {
      status_f(0, i) = !OptFlowCalculator::success_value;
    }
  }
  return 0;
}

template <typename OptFlowCalculator>
auto ImageProcessor<OptFlowCalculator>::GetInputMaskFromPoints(const cv::cuda::GpuMat &d_input_corners,
                                                               const cv::Size frame_size, cv::Mat &mask) -> int {
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

template <typename OptFlowCalculator>
template <typename T>
int ImageProcessor<OptFlowCalculator>::DetectNewFeatures(const cv::Ptr<T> &detector_ptr,
                                                         const cv::cuda::GpuMat &d_frame,
                                                         const cv::cuda::GpuMat &d_input_corners,
                                                         UMat<cv::Vec2f> &d_output) {
  // Create a mask to avoid redetecting existing features.

  cv::Mat mask;
  GetInputMaskFromPoints(d_input_corners, d_frame.size(), mask);

  cv::cuda::GpuMat d_mask(mask);

  // Detect new features
  cv::cuda::GpuMat tmp_output;
  detector_ptr->detect(d_frame, tmp_output, d_mask);
  d_output = tmp_output;
  return 0;
}

template class ImageProcessor<OptFlowCvGpu>;
#ifdef WITH_VPI
template class ImageProcessor<OptFlowVpiGpu>;
#endif
