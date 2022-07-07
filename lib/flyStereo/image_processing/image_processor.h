#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "flyStereo/image_processing/point_ops.h"
#include "flyStereo/interface.h"
#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "flyStereo/stereo_calibration.h"
#include "flyStereo/timed_logger.hpp"
#include "yaml-cpp/yaml.h"

template <typename IpBackend>
class ImageProcessor {
 public:
  /**
   * @brief Construct a new Image Processor object
   *
   * @param rate_limit_fps The maximum number of frames to process per second. Excecution will sleep to limit the rate
   * to this value.
   * @param stereo_calibration The stereo calibration to use for the image processor.
   * @param R_imu_cam0 The rotation matrix from the IMU to the camera frame.
   */
  ImageProcessor(float rate_limit_fps, StereoCalibration stereo_calibration, cv::Matx33d R_imu_cam0);

  /**
   * @brief Construct a new Image Processor object
   *
   * @param rate_limit_fps The maximum number of frames to process per second. Excecution will sleep to limit the rate
   * to this value.
   * @param stereo_calibration The stereo calibration to use for the image processor. A 'StereoCalibration' object is
   * instantiated from a YAML::Node
   * @param R_imu_cam0 The rotation matrix from the IMU to the camera frame.
   */
  ImageProcessor(float rate_limit_fps, const YAML::Node &stereo_calibration, cv::Matx33d R_imu_cam0);

  // Forbid the unused constructors
  ImageProcessor() = delete;
  ImageProcessor(const ImageProcessor &) = delete;
  auto operator=(const ImageProcessor &) = delete;
  ImageProcessor(ImageProcessor &&) = delete;
  auto operator=(ImageProcessor &&) = delete;

  ~ImageProcessor() = default;

  /**
   * @brief Initialize the image processor. Thie must be called once before processing can be done
   */
  void Init();

  /**
   * @brief Process a set of stereo images and synchronized imu data. This function calculates a tracked set of features
   * that is uniquely identifiable in both images, AND both images from the previous timestep. This function blocks
   * until the processing is complete.
   *
   * @param frame_cam0 The frame from camera 0. This function consumes the data of frame.
   * @param frame_cam1 The frame from camera 1. This function consumes the data of frame.
   * @param imu_data The imu data points that are synchronized with the images.
   * @param current_frame_time The timestamp of the current frame.
   * @param points The calculated tracked points from the images
   * @return int
   */
  auto process_image(IpBackend::image_type &&frame_cam0, IpBackend::image_type &&frame_cam1,
                     const std::vector<mavlink_imu_t> &imu_data, uint64_t current_frame_time)
      -> TrackedImagePoints<IpBackend>;

  /**
   * @brief Get the image from camera 0 and timestamp 0 (the previous frame)
   *
   * @return const IpBackend::image_type&
   */
  const IpBackend::image_type &get_image_c0_t0() const { return frame_cam0_t0_; }

  /**
   * @brief Get the image from camera 1 and timestamp 0 (the previous frame)
   *
   * @return const IpBackend::image_type&
   */
  const IpBackend::image_type &get_image_c1_t0() const { return frame_cam1_t0_; }

  /**
   * @brief Get the image from camera 0 and timestamp 1 (the current frame)
   *
   * @return const IpBackend::image_type&
   */
  const IpBackend::image_type &get_image_c0_t1() const { return frame_cam0_t1_; }

  /**
   * @brief Get the image from camera 1 and timestamp 1 (the current frame)
   *
   * @return const IpBackend::image_type&
   */
  const IpBackend::image_type &get_image_c1_t1() const { return frame_cam1_t1_; }

 private:
  /**
   * @brief Applies an inverse rotation to a set of 2D points, intended to 'undo' the roatation of the system between
   * the previous timestep and the current, using data measured by the IMU
   *
   * @param current_pts The input points
   * @param rotation The rotation matrix to apply, this typically derived from IMU data collected between two images
   * frames
   * @param camera_matrix The camera matrix for the camera that the points are from
   * @param updated_pts The output points
   * @return int
   */
  auto UpdatePointsViaImu(const IpBackend::array_type &current_pts, const cv::Matx33d &rotation,
                          const cv::Matx33d &camera_matrix, IpBackend::array_type &updated_pts) -> void;

  /**
   * @brief Find the set of tracked points from camera 0 in the image of camera 1. This function uses camera extrinsics
   * to predict feature locations, optical flow to find features, and epipolar geometry to remove outliers.
   *
   * @param opt The type of optical flow to apply
   * @param frame_cam0 The frame from camera 0
   * @param frame_cam1 The frame from camera 1
   * @param pts_cam0 The points from camera 0 to find in camera 1
   * @param pts_cam1 The output points from camera 1
   * @param status A vector of status flags for each point in pts_cam1. Points with poor status should be removed. The
   * value for 'good' status vs. 'bad' status is dependend on the IpBackend used.
   * @param stream The stream object
   * @return int
   */
  int StereoMatch(IpBackend::flow_type &opt, const IpBackend::image_type &frame_cam0,
                  const IpBackend::image_type &frame_cam1, const IpBackend::array_type &pts_cam0,
                  IpBackend::array_type &pts_cam1, IpBackend::status_type &status, IpBackend::stream_type &stream);

  /**
   * @brief Generates a mask image that identifies where current points are. This is used prevent re-detection and
   * processesing of the same point
   *
   * @param d_input_corners The input points
   * @param frame_size The size of the frame
   * @return IpBackend::image_type
   */
  auto GetInputMaskFromPoints(const IpBackend::array_type &d_input_corners, const cv::Size frame_size)
      -> IpBackend::image_type;

  // TODO finish documenting
  /**
   * @brief Detects a new set of features to use for image processing
   *
   * @param detector_ptr The detector object to use
   * @param d_frame The frame to detect features in
   * @param mask The mask which shows where existing detections are located
   * @param d_output The output points
   * @param stream The stream object
   * @tparam T The type of the detector
   */
  template <typename T>
  auto DetectNewFeatures(const cv::Ptr<T> &detector_ptr, const IpBackend::image_type &d_frame,
                         const IpBackend::image_type &mask, IpBackend::array_type &d_output,
                         IpBackend::stream_type &stream) -> void;

  /**
   * @brief Consolidate all the output points in the interface object 'TrackedImagePoint'
   *
   * @param imu_msgs The imu messages for this timestep
   * @param rotation_t0_t1_cam0  The rotation matrix between the previous timestep and the current timestep for camera 0
   * @return TrackedImagePoints<IpBackend>
   */
  auto OuputTrackedPoints(const std::vector<mavlink_imu_t> &imu_msgs, const cv::Matx33f &rotation_t0_t1_cam0)
      -> TrackedImagePoints<IpBackend>;

  /**
   * @brief Runs the detection routine to find new keypoints for tracking
   *
   * @param mask
   */
  auto detect(const IpBackend::image_type &mask) -> void;

  std::atomic<bool> is_running_;

  // Local version of input params
  YAML::Node input_params_;

  // Algorithm state
  unsigned int current_id_ = 0;  // The id of the last tracked point TODO CHANGE NAME
  IpBackend::detector_type detector_;

  IpBackend::flow_type d_opt_flow_cam0_;
  IpBackend::flow_type d_opt_flow_stereo_t0_;
  IpBackend::flow_type d_opt_flow_stereo_t1_;
  IpBackend::stream_type detector_stream_;
  IpBackend::stream_type tracker_stream_;

  std::chrono::time_point<std::chrono::system_clock> prev_time_end_;
  std::chrono::system_clock::duration fps_limit_inv_;
  IpBackend::image_type frame_cam0_t0_;
  IpBackend::image_type frame_cam1_t0_;
  IpBackend::image_type frame_cam0_t1_;
  IpBackend::image_type frame_cam1_t1_;

  // Points to track
  IpBackend::array_type pts_t_c0_t0_;  //< Tracked points in camera 0 frame at time t0
  IpBackend::array_type pts_t_c0_t1_;  //< Tracked points in camera 0 frame at time t1
  IpBackend::array_type pts_t_c1_t0_;  //< Tracked points in camera 1 frame at time t0
  IpBackend::array_type pts_t_c1_t1_;  //< Tracked points in camera 1 frame at time t1
  IpBackend::array_type pts_d_c0_t0_;  //< Detected points in camera 0 frame at time t0
  IpBackend::array_type pts_d_c0_t1_;  //< Detected points in camera 0 frame at time t1
  IpBackend::array_type pts_d_c1_t0_;  //< Detected points in camera 1 frame at time t0
  IpBackend::array_type pts_d_c1_t1_;  //< Detected points in camera 1 frame at time t1

  // IDs of the points
  std::vector<uint32_t> ids_pts_tracked_;
  std::vector<uint32_t> ids_pts_detected_;

  // Rate limit factor
  float rate_limit_fps_;
  bool is_rate_limiting_ = false;

  // Stereo Camera Parameters
  StereoCalibration stereo_cal_;

  // Imu to camera rotations
  cv::Matx33f R_imu_cam0_;
  cv::Matx33f R_imu_cam1_;

  unsigned int curr_pts_index_ = 0;
  TimedLogger &logger_ = TimedLogger::get_instance();
};

#include "flyStereo/image_processing/image_processor.tpp"
