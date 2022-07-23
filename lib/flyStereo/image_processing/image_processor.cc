#include "flyStereo/image_processing/image_processor.h"

#include <fstream>
#include <random>

#include "flyStereo/sensor_io/arducam_system.h"
#include "flyStereo/utility.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "spdlog/spdlog.h"

// Debug includes
#include "flyStereo/debug_video_recorder.h"

// Debug test function
template <typename IpBackend>
double print_percent_status(typename IpBackend::image_type &status) {
  int counter = std::accumulate(status.frame().begin(), status.frame().end(), 0, [](int sum, uint8_t val) {
    if (val == IpBackend::flow_type::success_value) {
      return sum + 1;
    } else {
      return sum;
    }
  });
  return static_cast<double>(counter) / status.frame().cols;
}

// Compile-time constants
struct IpConstants {
  static constexpr bool write_to_debug = false;

  // calcOptFlowPryLK
  static constexpr int window_size = 21;
  static constexpr int max_pyramid_level = 3;
  static constexpr int max_iters = 30;
  static constexpr bool use_initial_flow = true;

  // Binning
  static constexpr unsigned int bins_width = 20;
  static constexpr unsigned int bins_height = 15;
  static constexpr unsigned int max_pts_in_bin = 1;

  // Thresholds
  static constexpr double stereo_threshold = 45.0;
  static constexpr double ransac_threshold = 10.0;
};

template <typename IpBackend>
ImageProcessor<IpBackend>::ImageProcessor(float rate_limit_fps, const YAML::Node &stereo_calibration,
                                          cv::Matx33d R_imu_cam0)
    : ImageProcessor(rate_limit_fps, StereoCalibration(stereo_calibration), R_imu_cam0) {}

template <typename IpBackend>
ImageProcessor<IpBackend>::ImageProcessor(float rate_limit_fps, StereoCalibration stereo_calibration,
                                          cv::Matx33d R_imu_cam0)
    : rate_limit_fps_(rate_limit_fps),
      is_rate_limiting_(rate_limit_fps > 0.f),
      stereo_cal_(stereo_calibration),
      R_imu_cam0_(R_imu_cam0) {
  pts_t_c0_t0_ = typename IpBackend::array_type(cv::Size(0, 1));
  pts_t_c0_t1_ = typename IpBackend::array_type(cv::Size(0, 1));
  pts_t_c1_t0_ = typename IpBackend::array_type(cv::Size(0, 1));
  pts_t_c1_t1_ = typename IpBackend::array_type(cv::Size(0, 1));
  pts_d_c0_t0_ = typename IpBackend::array_type(cv::Size(0, 1));
  pts_d_c0_t1_ = typename IpBackend::array_type(cv::Size(0, 1));
  pts_d_c1_t0_ = typename IpBackend::array_type(cv::Size(0, 1));
  pts_d_c1_t1_ = typename IpBackend::array_type(cv::Size(0, 1));

  // cv::Matx33f stereo_cal_.R_cam0_cam1 = stereo_cal_.R_cam0_cam1;
  R_imu_cam1_ = R_imu_cam0_ * cv::Matx33f(stereo_cal_.R_cam0_cam1);

  const auto max_pts = IpConstants::bins_width * IpConstants::bins_height * IpConstants::max_pts_in_bin;
  ids_pts_tracked_.reserve(max_pts);
  ids_pts_detected_.reserve(max_pts);
}

template <typename IpBackend>
void ImageProcessor<IpBackend>::Init() {
  // detector_ptr_ = cv::cuda::createGoodFeaturesToTrackDetector(CV_8U, IpConstants::max_corners,
  //                                                             IpConstants::quality_level,
  //                                                             IpConstants::min_dist);

  d_opt_flow_cam0_ = typename IpBackend::flow_type(IpConstants::window_size, IpConstants::max_pyramid_level,
                                                   IpConstants::max_iters, IpConstants::use_initial_flow);
  d_opt_flow_stereo_t0_ = typename IpBackend::flow_type(IpConstants::window_size, IpConstants::max_pyramid_level,
                                                        IpConstants::max_iters, IpConstants::use_initial_flow);
  d_opt_flow_stereo_t1_ = typename IpBackend::flow_type(IpConstants::window_size, IpConstants::max_pyramid_level,
                                                        IpConstants::max_iters, IpConstants::use_initial_flow);
  fps_limit_inv_ =
      std::chrono::round<std::chrono::system_clock::duration>(std::chrono::duration<double>(1. / rate_limit_fps_));
}

template <typename IpBackend>
void ImageProcessor<IpBackend>::UpdatePointsViaImu(const IpBackend::array_type &current_pts,
                                                   const cv::Matx33d &rotation, const cv::Matx33d &camera_matrix,
                                                   IpBackend::array_type &updated_pts) {
  // Sanity Checks
  if (current_pts.frame().rows != 1 && current_pts.frame().channels() != 2) {
    throw std::runtime_error("ImageProcessor::UpdatePointsViaImu: current_pts must be 1xNx2");
  } else if (current_pts.frame().cols == 0) {
    return;
  }

  cv::Matx33f H = camera_matrix * rotation.t() * camera_matrix.inv();

  std::vector<cv::Point3f> interface_pts;
  cv::convertPointsToHomogeneous(current_pts.frame(), interface_pts);
  std::for_each(interface_pts.begin(), interface_pts.end(), [&H](auto &pt) { pt = H * pt; });

  cv::Mat updated_pts_mat;
  cv::convertPointsFromHomogeneous(interface_pts, updated_pts_mat);
  updated_pts = updated_pts_mat.t();
}

template <typename IpBackend>
TrackedImagePoints<IpBackend> ImageProcessor<IpBackend>::OuputTrackedPoints(const std::vector<mavlink_imu_t> &imu_msgs,
                                                                            const cv::Matx33f &rotation_t0_t1_cam0) {
  const cv::Size pts_size = pts_t_c0_t0_.size();
  if (pts_size != pts_t_c0_t1_.size() || pts_size != pts_t_c1_t0_.size() || pts_size != pts_t_c1_t1_.size() ||
      pts_size.height != 1 || pts_size.width != static_cast<int>(ids_pts_tracked_.size())) {
    throw std::runtime_error("Error! Vectors do not match in OutputTrackedPoints");
  }

  uint64_t timestamp_us =
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
          .count();

  return TrackedImagePoints<IpBackend>(timestamp_us, ids_pts_tracked_, pts_t_c0_t0_, pts_t_c0_t1_, pts_t_c1_t0_,
                                       pts_t_c1_t1_, imu_msgs, rotation_t0_t1_cam0);
}

template <typename IpBackend>
void ImageProcessor<IpBackend>::detect(const IpBackend::image_type &mask) {
  // Detect new features to match for the next iteration
  detector_.detect(frame_cam0_t1_, mask, pts_d_c0_t1_);

  // Match the detected features in the second camera
  typename IpBackend::status_type status(pts_d_c0_t1_.size());
  StereoMatch(d_opt_flow_stereo_t1_, frame_cam0_t1_, frame_cam1_t1_, pts_d_c0_t1_, pts_d_c1_t1_, status,
              detector_stream_);
  detector_stream_.sync();

  if (pts_d_c0_t1_.frame().cols != 0) {
    MarkPointsOutOfFrame(status, pts_d_c1_t1_, frame_cam1_t1_.size(), IpBackend::flow_type::success_value);
    RemovePoints(status, IpBackend::flow_type::success_value, pts_d_c0_t1_, pts_d_c1_t1_);

    auto counter = 0;
    std::for_each(status.frame().begin(), status.frame().end(), [&counter](auto &status_row) {
      if (status_row == IpBackend::flow_type::success_value) {
        counter++;
      }
    });

    // Fill in the ids vector with our new detected features
    ids_pts_detected_.clear();
    ids_pts_detected_.reserve(pts_d_c0_t1_.frame().cols);

    for (int i = 0; i < pts_d_c0_t1_.frame().cols; i++) {
      ids_pts_detected_.push_back(current_id_++);
    }
  } else {
    // No new detections
    ids_pts_detected_.clear();
    pts_d_c0_t1_ = typename IpBackend::array_type(cv::Size(0, 1));
    pts_d_c1_t1_ = typename IpBackend::array_type(cv::Size(0, 1));
  }

  if (pts_d_c0_t1_.size() != pts_d_c1_t1_.size() ||
      pts_d_c1_t1_.size().width != static_cast<int>(ids_pts_detected_.size())) {
    throw std::runtime_error("sizes do not match in detect");
  }
}

// TODO make a check that the object is initialized before starting
template <typename IpBackend>
TrackedImagePoints<IpBackend> ImageProcessor<IpBackend>::process_image(IpBackend::image_type &&frame_cam0_t1_umat,
                                                                       IpBackend::image_type &&frame_cam1_t1_umat,
                                                                       const std::vector<mavlink_imu_t> &imu_msgs,
                                                                       uint64_t current_frame_time) {
  frame_cam0_t1_ = frame_cam0_t1_umat;
  frame_cam1_t1_ = frame_cam1_t1_umat;

  cv::Matx33f rotation_t0_t1_cam0;
  cv::Matx33f rotation_t0_t1_cam1;
  int retval = ArducamSystem::GenerateImuXform(imu_msgs, R_imu_cam0_, R_imu_cam1_, rotation_t0_t1_cam0,
                                               current_frame_time, rotation_t0_t1_cam1);

  if (pts_t_c0_t0_.size() != pts_t_c1_t0_.size()) {
    throw std::runtime_error("points do not match across cameras! 0");
  }

  // Append the kepypoints from the previous iteration to the tracked points of the current
  // iteration. Points that pass through all the outlier checks will remain as tracked points

  pts_t_c0_t0_ = AppendUMatColwise(pts_t_c0_t0_, pts_d_c0_t1_);
  pts_t_c1_t0_ = AppendUMatColwise(pts_t_c1_t0_, pts_d_c1_t1_);

  ids_pts_tracked_.insert(std::end(ids_pts_tracked_), std::begin(ids_pts_detected_), std::end(ids_pts_detected_));

  if (pts_t_c0_t0_.size() != pts_t_c1_t0_.size()) {
    throw std::runtime_error("points do not match across cameras!");
  }

  // Remove the points the exceed the binning threshold
  if (pts_t_c0_t0_.frame().cols > 1) {
    // Vector to indicate whether a point should be deleted or kept after binning
    typename IpBackend::status_type bin_status;
    BinAndMarkPoints(pts_t_c0_t0_, ids_pts_tracked_, frame_cam0_t1_.size(),
                     cv::Size(IpConstants::bins_width, IpConstants::bins_height), IpConstants::max_pts_in_bin,
                     bin_status, IpBackend::flow_type::success_value);

    if (pts_t_c0_t0_.size() != pts_t_c1_t0_.size()) {
      throw std::runtime_error("points do not match across cameras!  1");
    }

    RemovePoints(bin_status, IpBackend::flow_type::success_value, pts_t_c0_t0_, pts_t_c1_t0_, ids_pts_tracked_);
  }

  // Make a prediction of where the points will be in the current image given the IMU data
  if (retval == 0) {
    UpdatePointsViaImu(pts_t_c0_t0_, rotation_t0_t1_cam0, stereo_cal_.K_cam0, pts_t_c0_t1_);
    UpdatePointsViaImu(pts_t_c1_t0_, rotation_t0_t1_cam1, stereo_cal_.K_cam1, pts_t_c1_t1_);
  } else {
    throw std::runtime_error("Error receiving IMU transform");
  }

  // Apply the optical flow from the previous frame to the current frame
  if (!pts_t_c0_t0_.frame().empty()) {
    typename IpBackend::status_type status(pts_t_c0_t0_.size());

    // auto copy_1 = pts_t_c0_t0_.clone();
    d_opt_flow_cam0_.calc(frame_cam0_t0_, frame_cam0_t1_, pts_t_c0_t0_, pts_t_c0_t1_, status, &tracker_stream_);
    // d_opt_flow_cam0_.calc(frame_cam0_t0_, frame_cam0_t1, pts_t_c0_t0_, pts_t_c0_t1_, status, tracker_stream_);
    tracker_stream_.sync();
    {
      MarkPointsOutOfFrame(status, pts_t_c0_t1_, frame_cam0_t1_.size(), IpBackend::flow_type::success_value);

      RemovePoints(status, IpBackend::flow_type::success_value, pts_t_c0_t0_, pts_t_c0_t1_, pts_t_c1_t0_, pts_t_c1_t1_,
                   ids_pts_tracked_);
    }

    // Match the points from camera 0 to camera 1
    status = typename IpBackend::status_type(pts_t_c0_t1_.size());
    int ret_val = StereoMatch(d_opt_flow_stereo_t0_, frame_cam0_t1_, frame_cam1_t1_, pts_t_c0_t1_, pts_t_c1_t1_, status,
                              tracker_stream_);

    // Remove the outliers from the StereoMatch algorithm
    if (ret_val == 0) {
      // std::cout << "t pts before " << pts_t_c0_t1_.frame().cols << std::endl;
      // std::cout << "t pts after " << pts_t_c0_t1_.frame().cols << std::endl;
      RemovePoints(status, IpBackend::flow_type::success_value, pts_t_c0_t0_, pts_t_c0_t1_, pts_t_c1_t0_, pts_t_c1_t1_,
                   ids_pts_tracked_);

    } else if (ret_val < 0) {
      throw std::runtime_error("Error in stereo matching");
    }

    // Perform a check to make sure that all of our tracked vectors are the same length,
    // otherwise something is wrong
    int tracked_pts = pts_t_c0_t0_.frame().cols;
    if (tracked_pts != pts_t_c0_t1_.frame().cols || tracked_pts != pts_t_c1_t0_.frame().cols ||
        tracked_pts != pts_t_c1_t1_.frame().cols) {
      throw std::runtime_error("ERROR! There are differences in the numbers of tracked points ");
    }
  }

  auto mask = GetInputMaskFromPoints(pts_t_c0_t1_, frame_cam0_t1_.size());

  detect(mask);

  TrackedImagePoints<IpBackend> tracked_points;
  if (pts_t_c0_t1_.frame().cols > 1 && pts_t_c1_t1_.frame().cols > 1) {
    tracked_points = OuputTrackedPoints(imu_msgs, rotation_t0_t1_cam0);
  }

  // Plot output if enabled
  {
    plot_points_overlay_2x1(frame_cam0_t1_, {pts_d_c0_t1_}, frame_cam1_t1_, {pts_d_c1_t1_});
    plot_points_overlay_2x2(frame_cam0_t0_, {pts_t_c0_t0_}, frame_cam1_t0_, {pts_t_c1_t0_}, frame_cam0_t1_,
                            {pts_t_c0_t1_}, frame_cam1_t1_, {pts_t_c1_t1_});

    // cv::imwrite("left.png", frame_cam0_t1_.frame());
    // cv::imwrite("right.png", frame_cam1_t1_.frame());
  }

  // Set the current t1 values to t0 for the next iteration
  std::swap(frame_cam0_t1_, frame_cam0_t0_);
  std::swap(frame_cam1_t1_, frame_cam1_t0_);

  if (pts_t_c0_t1_.size() != pts_t_c1_t1_.size()) {
    throw std::runtime_error("points do not match across cameras! 3");
  }

  pts_t_c0_t0_ = std::move(pts_t_c0_t1_);
  pts_t_c0_t1_ = typename IpBackend::array_type(cv::Size(0, 1));
  pts_t_c1_t0_ = std::move(pts_t_c1_t1_);
  pts_t_c1_t1_ = typename IpBackend::array_type(cv::Size(0, 1));

  // Apply the rate limiting
  if (is_rate_limiting_) {
    std::this_thread::sleep_until(prev_time_end_ + fps_limit_inv_);
  }

  spdlog::trace(
      "fps: {}",
      1.0 / static_cast<std::chrono::duration<double>>((std::chrono::system_clock::now() - prev_time_end_)).count());
  prev_time_end_ = std::chrono::system_clock::now();
  return tracked_points;
}

template <typename IpBackend>
int ImageProcessor<IpBackend>::StereoMatch(IpBackend::flow_type &opt, const IpBackend::image_type &frame_cam0,
                                           const IpBackend::image_type &frame_cam1,
                                           const IpBackend::array_type &pts_cam0, IpBackend::array_type &pts_cam1,
                                           IpBackend::status_type &status, IpBackend::stream_type &stream) {
  if (pts_cam0.frame().empty()) {
    return 1;
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

  opt.calc(frame_cam0, frame_cam1, pts_cam0, pts_cam1, status, &stream);

  stream.sync();

  // Check the make sure our output vector was populated as expected
  if (pts_cam1.frame().empty()) {
    return -1;
  }

  MarkPointsOutOfFrame(status, pts_cam1, frame_cam1.size(), IpBackend::flow_type::success_value);

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
    if (status_f(0, i) != IpBackend::flow_type::success_value) {
      continue;
    }

    cv::Vec3f pt1(tracked_pts_cam1_t1_undistorted[i].x, tracked_pts_cam1_t1_undistorted[i].y, 1.0);

    // Calculates the distance from the point to the epipolar line (in pixels)
    double error = fabs(pt1.dot(epilines[i]));

    if (error > IpConstants::stereo_threshold) {
      status_f(0, i) = !IpBackend::flow_type::success_value;
    }
  }
  return 0;
}

template <typename IpBackend>
auto ImageProcessor<IpBackend>::GetInputMaskFromPoints(const IpBackend::array_type &d_input_corners,
                                                       const cv::Size frame_size) -> IpBackend::image_type {
  typename IpBackend::image_type mask(frame_size);
  mask.frame().setTo(1);
  double mask_box_size = 15.0;

  if (d_input_corners.d_frame().cols > 0) {
    for (auto point = d_input_corners.frame().begin(); point != d_input_corners.frame().end(); point++) {
      const int x = static_cast<int>((*point)[0]);
      const int y = static_cast<int>((*point)[1]);

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
      mask.frame()(row_range, col_range) = 0;
    }
  }
  return mask;
}

template <typename IpBackend>
template <typename T>
void ImageProcessor<IpBackend>::DetectNewFeatures(const cv::Ptr<T> &detector_ptr, const IpBackend::image_type &d_frame,
                                                  const IpBackend::image_type &mask, IpBackend::array_type &d_output,
                                                  IpBackend::stream_type &stream) {
  // Detect new features
  cv::cuda::GpuMat tmp_output;
  // TODO implement VPI detector
  if constexpr (std::is_same<typename IpBackend::stream_type, cv::cuda::Stream>::value) {
    detector_ptr->detect(d_frame.d_frame(), tmp_output, mask.d_frame(), stream);
  } else {
    detector_ptr->detect(d_frame.d_frame(), tmp_output, mask.d_frame());
  }

  d_output = tmp_output;
}

#include "flyStereo/image_processing/cv_backend.h"
template class ImageProcessor<CvBackend>;
#ifdef WITH_VPI
#include "flyStereo/image_processing/vpi_backend.h"
template class ImageProcessor<VpiBackend>;
#endif
