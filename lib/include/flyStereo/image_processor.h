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
#include "flyStereo/interface.h"
#include "flyStereo/optical_flow_points.h"
#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "flyStereo/sensor_io/sensor_interface.h"
#include "opencv2/core.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaoptflow.hpp"
#include "opencv2/features2d.hpp"
#include "yaml-cpp/yaml.h"

class ImageProcessor {
 public:
  ImageProcessor(const YAML::Node &input_params, const YAML::Node &stereo_calibration);

  // Forbit the unused constructors
  ImageProcessor() = delete;
  ImageProcessor(const ImageProcessor &) = delete;
  auto operator=(const ImageProcessor &) = delete;
  ImageProcessor(ImageProcessor &&) = delete;
  auto operator=(ImageProcessor &&) = delete;

  ~ImageProcessor() = default;

  void Init();

  auto process_image(const cv::cuda::GpuMat &d_frame_cam0, const cv::cuda::GpuMat &d_frame_cam1,
                     const std::vector<mavlink_imu_t> &imu_data, uint64_t current_frame_time, ImagePoints &points)
      -> int;

 private:
  auto UpdatePointsViaImu(const std::vector<cv::Point2f> &current_pts, const cv::Matx33d &rotation,
                          const cv::Matx33d &camera_matrix, std::vector<cv::Point2f> &updated_pts) -> int;

  auto UpdatePointsViaImu(const cv::cuda::GpuMat &current_pts, const cv::Matx33d &rotation,
                          const cv::Matx33d &camera_matrix, cv::cuda::GpuMat &updated_pts) -> int;

  auto StereoMatch(cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> opt, const cv::cuda::GpuMat &d_frame_cam0,
                   const cv::cuda::GpuMat &d_frame_cam1, OpticalFlowPoints &points,
                   std::pair<unsigned int, unsigned int> indices, cv::cuda::GpuMat &d_status) -> int;

  auto GetInputMaskFromPoints(const cv::cuda::GpuMat &d_input_corners, const cv::Size frame_size, cv::Mat &mask) -> int;

  // cv::cuda::FastFeatureDetector or cv::cuda::CornersDetector
  template <typename T>
  auto DetectNewFeatures(const cv::Ptr<T> &detector_ptr, const cv::cuda::GpuMat &d_frame,
                         const cv::cuda::GpuMat &d_input_corners, cv::cuda::GpuMat &d_output) -> int;

  auto OuputTrackedPoints(OpticalFlowPoints &points, const std::vector<unsigned int> &ids,
                          const std::vector<mavlink_imu_t> &imu_msgs, const cv::Matx33f &rotation_t0_t1_cam0,
                          ImagePoints &output_points) -> int;

  // Local version of input params
  YAML::Node input_params_;

  // Algorithm state
  OpticalFlowPoints points_;
  unsigned int current_id = 0;  // The id of the last tracked point TODO CHANGE NAME
  cv::Ptr<cv::cuda::CornersDetector> detector_ptr_;
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_opt_flow_cam0_;
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_opt_flow_stereo_t0_;
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_opt_flow_stereo_t1_;
  std::chrono::time_point<std::chrono::system_clock> prev_time_end_;
  std::chrono::system_clock::duration fps_limit_inv_;
  cv::cuda::GpuMat d_frame_cam0_t0_;
  cv::cuda::GpuMat d_frame_cam1_t0_;

  // Rate limit factor
  float rate_limit_fps_;

  // config params for LK optical flow
  int window_size_;
  int max_pyramid_level_;
  int max_iters_;

  // Config params for goodFeaturesToTrack
  int max_corners_;
  float quality_level_;
  int min_dist_;

  // Running config params
  int max_error_counter_;
  double stereo_threshold_;

  // Config Params for RANSAC
  double ransac_threshold_;

  // Binning Params
  unsigned int bins_width_;
  unsigned int bins_height_;
  unsigned int max_pts_in_bin_;

  // Stereo Camera Parameters
  cv::Matx33d K_cam0_;
  cv::Matx33d K_cam1_;
  std::vector<double> D_cam0_;
  std::vector<double> D_cam1_;
  cv::Matx33d R_cam0_cam1_;
  cv::Vec3d T_cam0_cam1_;
  cv::Matx33d E_;
  cv::Matx33d F_;
  cv::Matx33d R_cam0_;
  cv::Matx33d R_cam1_;
  cv::Matx34d P_cam0_;
  cv::Matx34d P_cam1_;
  cv::Matx44d Q_;
  cv::Matx33f R_imu_cam0_;
  cv::Matx33f R_imu_cam1_;

  // 3D output
  unsigned int curr_pts_index_ = 0;
  std::array<std::map<unsigned int, cv::Vec3f>, 2> points_3d_;
  cv::Vec3f vio_sum_ = cv::Vec3f(0.0, 0.0, 0.0);
  Eigen::MatrixXd pose_;
  Eigen::MatrixXd prev_xform_;
};
