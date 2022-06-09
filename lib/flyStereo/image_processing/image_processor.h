#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "flyStereo/image_processing/point_ops.h"
#include "flyStereo/interface.h"
#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "flyStereo/sensor_io/sensor_interface.h"
#include "flyStereo/stereo_calibration.h"
#include "flyStereo/timed_logger.hpp"
#include "flyStereo/types/umat_vpiarray.h"
#include "flyStereo/types/umat_vpiimage.h"
#include "yaml-cpp/yaml.h"

template <typename IpBackend>
class ImageProcessor {
 public:
  ImageProcessor(float rate_limit_fps, const YAML::Node &stereo_calibration, cv::Matx33d R_imu_cam0);
  ImageProcessor(float rate_limit_fps, StereoCalibration stereo_calibration, cv::Matx33d R_imu_cam0);

  // Forbit the unused constructors
  ImageProcessor() = delete;
  ImageProcessor(const ImageProcessor &) = delete;
  auto operator=(const ImageProcessor &) = delete;
  ImageProcessor(ImageProcessor &&) = delete;
  auto operator=(ImageProcessor &&) = delete;

  ~ImageProcessor();

  void Init();

  auto process_image(cv::Mat_<uint8_t> &frame_cam0, cv::Mat_<uint8_t> &frame_cam1,
                     const std::vector<mavlink_imu_t> &imu_data, uint64_t current_frame_time,
                     TrackedImagePoints &points) -> int;

 private:
  auto UpdatePointsViaImu(const IpBackend::array_type &current_pts, const cv::Matx33d &rotation,
                          const cv::Matx33d &camera_matrix, IpBackend::array_type &updated_pts) -> int;

  int StereoMatch(IpBackend::flow_type &opt, IpBackend::image_type &frame_cam0, IpBackend::image_type &frame_cam1,
                  IpBackend::array_type &pts_cam0, IpBackend::array_type &pts_cam1, IpBackend::status_type &status,
                  IpBackend::stream_type &stream);

  auto GetInputMaskFromPoints(const IpBackend::array_type &d_input_corners, const cv::Size frame_size)
      -> IpBackend::image_type;

  // cv::cuda::FastFeatureDetector or cv::cuda::CornersDetector
  template <typename T>
  auto DetectNewFeatures(const cv::Ptr<T> &detector_ptr, const IpBackend::image_type &d_frame,
                         const IpBackend::image_type &mask, IpBackend::array_type &d_output,
                         IpBackend::stream_type &stream) -> void;

  auto OuputTrackedPoints(const std::vector<mavlink_imu_t> &imu_msgs, const cv::Matx33f &rotation_t0_t1_cam0)
      -> TrackedImagePoints;

  auto detect(const IpBackend::image_type &mask) -> void;
  auto track() -> void;

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

  // 3D output
  unsigned int curr_pts_index_ = 0;
  std::array<std::map<unsigned int, cv::Vec3f>, 2> points_3d_;
  cv::Vec3f vio_sum_ = cv::Vec3f(0.0, 0.0, 0.0);
  Eigen::MatrixXd pose_;
  Eigen::MatrixXd prev_xform_;

  TimedLogger &logger_ = TimedLogger::get_instance();
};
