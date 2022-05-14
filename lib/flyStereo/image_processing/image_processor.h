#pragma once

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "Eigen/Dense"
#include "flyStereo/image_processing/opt_flow_cv_gpu.h"
#include "flyStereo/image_processing/opt_flow_vpi_gpu.h"
#include "flyStereo/image_processing/optical_flow_points.h"
#include "flyStereo/interface.h"
#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "flyStereo/sensor_io/sensor_interface.h"
#include "flyStereo/stereo_calibration.h"
#include "opencv2/core.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/features2d.hpp"
#include "yaml-cpp/yaml.h"

template <typename OptFlowCalculator = OptFlowCvGpu>
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

  ~ImageProcessor() = default;

  void Init();

  auto process_image(UMat<uint8_t> &d_frame_cam0, UMat<uint8_t> &d_frame_cam1,
                     const std::vector<mavlink_imu_t> &imu_data, uint64_t current_frame_time,
                     TrackedImagePoints &points) -> int;

 private:
  auto UpdatePointsViaImu(const UMat<cv::Vec2f> &current_pts, const cv::Matx33d &rotation,
                          const cv::Matx33d &camera_matrix, UMat<cv::Vec2f> &updated_pts) -> int;

  auto StereoMatch(OptFlowCalculator &opt, const UMat<uint8_t> &d_frame_cam0, const UMat<uint8_t> &d_frame_cam1,
                   const UMat<cv::Vec2f> &pts_cam0, UMat<cv::Vec2f> &pts_cam1, UMat<uint8_t> &status) -> int;

  auto GetInputMaskFromPoints(const cv::cuda::GpuMat &d_input_corners, const cv::Size frame_size, cv::Mat &mask) -> int;

  // cv::cuda::FastFeatureDetector or cv::cuda::CornersDetector
  template <typename T>
  auto DetectNewFeatures(const cv::Ptr<T> &detector_ptr, const cv::cuda::GpuMat &d_frame,
                         const cv::cuda::GpuMat &d_input_corners, UMat<cv::Vec2f> &d_output) -> int;

  auto OuputTrackedPoints(const std::vector<mavlink_imu_t> &imu_msgs, const cv::Matx33f &rotation_t0_t1_cam0)
      -> TrackedImagePoints;

  // Local version of input params
  YAML::Node input_params_;

  // Algorithm state
  unsigned int current_id_ = 0;  // The id of the last tracked point TODO CHANGE NAME
  cv::Ptr<cv::cuda::CornersDetector> detector_ptr_;
  OptFlowCalculator d_opt_flow_cam0_;
  OptFlowCalculator d_opt_flow_stereo_t0_;
  OptFlowCalculator d_opt_flow_stereo_t1_;
  std::chrono::time_point<std::chrono::system_clock> prev_time_end_;
  std::chrono::system_clock::duration fps_limit_inv_;
  UMat<uint8_t> d_frame_cam0_t0_;
  UMat<uint8_t> d_frame_cam1_t0_;

  // Points to track
  UMat<cv::Vec2f> pts_t_c0_t0_;  //< Tracked points in camera 0 frame at time t0
  UMat<cv::Vec2f> pts_t_c0_t1_;  //< Tracked points in camera 0 frame at time t1
  UMat<cv::Vec2f> pts_t_c1_t0_;  //< Tracked points in camera 1 frame at time t0
  UMat<cv::Vec2f> pts_t_c1_t1_;  //< Tracked points in camera 1 frame at time t1
  UMat<cv::Vec2f> pts_d_c0_t0_;  //< Detected points in camera 0 frame at time t0
  UMat<cv::Vec2f> pts_d_c0_t1_;  //< Detected points in camera 0 frame at time t1
  UMat<cv::Vec2f> pts_d_c1_t0_;  //< Detected points in camera 1 frame at time t0
  UMat<cv::Vec2f> pts_d_c1_t1_;  //< Detected points in camera 1 frame at time t1

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
};
