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
#include "fly_stereo/interface.h"
#include "fly_stereo/optical_flow_points.h"
#include "fly_stereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "fly_stereo/sensor_io/sensor_interface.h"
#include "opencv2/core.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaoptflow.hpp"
#include "opencv2/features2d.hpp"
#include "yaml-cpp/yaml.h"

class ImageProcessor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImageProcessor(const YAML::Node &input_params, const YAML::Node &stereo_calibration);

  // Forbit the unused constructors
  ImageProcessor() = delete;
  ImageProcessor(const ImageProcessor &) = delete;
  ImageProcessor(ImageProcessor &&) = delete;

  ~ImageProcessor();

  int Init();

  bool IsRunning() { return is_running_.load(); }

  void ReceiveImu(const mavlink_imu_t &msg);

  bool GetTrackedPoints(ImagePoints *usr_pts);

 private:
  cv::cuda::GpuMat AppendGpuMatColwise(const cv::cuda::GpuMat &mat1, const cv::cuda::GpuMat &mat2);

  int UpdatePointsViaImu(const std::vector<cv::Point2f> &current_pts, const cv::Matx33d &rotation,
                         const cv::Matx33d &camera_matrix, std::vector<cv::Point2f> &updated_pts);

  int UpdatePointsViaImu(const cv::cuda::GpuMat &current_pts, const cv::Matx33d &rotation,
                         const cv::Matx33d &camera_matrix, cv::cuda::GpuMat &updated_pts);

  int ProcessThread();

  int StereoMatch(cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> opt, cv::cuda::GpuMat &d_frame_cam0,
                  cv::cuda::GpuMat &d_frame_cam1, OpticalFlowPoints &points,
                  std::pair<unsigned int, unsigned int> indices, cv::cuda::GpuMat &d_status);

  int RemoveOutliers(const std::vector<uchar> &status, std::vector<cv::Point2f> &points);
  // Overloaded constructor for the GPU implementation
  int RemoveOutliers(const cv::cuda::GpuMat &d_status, cv::cuda::GpuMat &d_points);
  int RemoveOutliers(const std::vector<uchar> &status, std::vector<unsigned int> &points);

  int RemovePointsOutOfFrame(const cv::Size framesize, const std::vector<cv::Point2f> &points,
                             std::vector<unsigned char> &status);
  // Overloaded constructor for the GPU implementation
  int RemovePointsOutOfFrame(const cv::Size framesize, const cv::cuda::GpuMat &d_points, cv::cuda::GpuMat &d_status);

  int GetInputMaskFromPoints(const cv::cuda::GpuMat &d_input_corners, const cv::Size frame_size, cv::Mat &mask);
  int DetectNewFeatures(const cv::Ptr<cv::cuda::FastFeatureDetector> &detector_ptr, const cv::cuda::GpuMat &d_frame,
                        const cv::cuda::GpuMat &d_input_corners, cv::cuda::GpuMat &d_output);
  int DetectNewFeatures(const cv::Ptr<cv::cuda::CornersDetector> &detector_ptr, const cv::cuda::GpuMat &d_frame,
                        const cv::cuda::GpuMat &d_input_corners, cv::cuda::GpuMat &d_output);

  void rescalePoints(std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2, float &scaling_factor);

  int twoPointRansac(const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2,
                     const cv::Matx33f &R_p_c, const cv::Matx33d &intrinsics,
                     const std::vector<double> &distortion_coeffs, const double &inlier_error,
                     const double &success_probability, std::vector<uchar> &inlier_markers);

  int GenerateImuXform(const std::vector<mavlink_imu_t> &imu_msgs, cv::Matx33f &rotation_t0_t1_cam0,
                       cv::Matx33f &rotation_t0_t1_cam1);

  int ProcessPoints(std::vector<cv::Point2f> pts_cam0_t0, std::vector<cv::Point2f> pts_cam0_t1,
                    std::vector<cv::Point2f> pts_cam1_t0, std::vector<cv::Point2f> pts_cam1_t1,
                    std::vector<unsigned int> ids);

  int OuputTrackedPoints(OpticalFlowPoints &points, const std::vector<unsigned int> &ids,
                         const std::vector<mavlink_imu_t> &imu_msgs, const cv::Matx33f &rotation_t0_t1_cam0);

  // Local version of input params
  YAML::Node input_params_;

  // Thread mgmt
  std::atomic<bool> is_running_;
  std::thread image_processor_thread_;

  // Video I/O
  std::unique_ptr<SensorInterface> sensor_interface_;
  bool draw_points_to_frame_;

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

  // Class output and it's mutex guard
  std::mutex output_mutex_;
  ImagePoints output_points_;
  std::condition_variable output_cond_var_;

  // 3D output
  unsigned int curr_pts_index_ = 0;
  std::array<std::map<unsigned int, cv::Vec3f>, 2> points_3d_;
  cv::Vec3f vio_sum_ = cv::Vec3f(0.0, 0.0, 0.0);
  Eigen::MatrixXd pose_;
  Eigen::MatrixXd prev_xform_;
};
