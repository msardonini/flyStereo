#ifndef INCLUDE_FLY_STEREO_IMAGE_PROCESSOR_H_
#define INCLUDE_FLY_STEREO_IMAGE_PROCESSOR_H_

#include <iostream>
#include <string>
#include <memory>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>

#include "fly_stereo/interface.h"
#include "fly_stereo/camera.h"
#include "fly_stereo/camera_trigger.h"
#include "yaml-cpp/yaml.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaoptflow.hpp"
#include "fly_stereo/mavlink/fly_stereo/mavlink.h"

class ImageProcessor {
 public:
  ImageProcessor(const YAML::Node &input_params);

  // Forbit the unused constructors
  ImageProcessor() = delete;
  ImageProcessor(const ImageProcessor&) = delete;
  ImageProcessor(ImageProcessor&&) = delete;

  ~ImageProcessor();

  int Init();

  bool IsRunning() {return is_running_.load();}

  void ReceiveImu(const mavlink_imu_t &msg);

  bool GetTrackedPoints(ImagePoints *usr_pts);

 private:
  cv::cuda::GpuMat AppendGpuMatColwise(const cv::cuda::GpuMat &mat1, const cv::cuda::GpuMat &mat2);

  int UpdatePointsViaImu(const std::vector<cv::Point2f> &current_pts,
    const cv::Matx33d &rotation,
    const cv::Matx33d &camera_matrix,
    std::vector<cv::Point2f> &updated_pts);

  int UpdatePointsViaImu(const cv::cuda::GpuMat &current_pts,
    const cv::Matx33d &rotation,
    const cv::Matx33d &camera_matrix,
    cv::cuda::GpuMat &updated_pts);

  int ProcessThread();

  void DrawPoints(const std::vector<cv::Point2f> &mypoints, cv::Mat &myimage);

  int StereoMatch(cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> opt,
    const cv::cuda::GpuMat &d_frame_cam0,
    const cv::cuda::GpuMat &d_frame_cam1, 
    cv::cuda::GpuMat &d_tracked_pts_cam0,
    cv::cuda::GpuMat &d_tracked_pts_cam1,
    cv::cuda::GpuMat &d_status);

  int RemoveOutliers(const std::vector<uchar> &status, std::vector<cv::Point2f> &points);
  // Overloaded constructor for the GPU implementation
  int RemoveOutliers(const cv::cuda::GpuMat &d_status, cv::cuda::GpuMat &d_points);
  int RemoveOutliers(const std::vector<uchar> &status, std::vector<unsigned int> &points);

  int RemovePointsOutOfFrame(const cv::Size framesize, const std::vector<cv::Point2f> &points,
    std::vector<unsigned char> &status); 
  // Overloaded constructor for the GPU implementation
  int RemovePointsOutOfFrame(const cv::Size framesize, const cv::cuda::GpuMat &d_points, 
    cv::cuda::GpuMat &d_status);

  int DetectNewFeatures(const cv::Ptr<cv::cuda::CornersDetector> &detector_ptr,
    const cv::cuda::GpuMat &d_frame,
    const cv::cuda::GpuMat &d_input_corners,
    cv::cuda::GpuMat &d_output);

  void rescalePoints(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2,
    float& scaling_factor);

  int twoPointRansac(
    const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
    const cv::Matx33f& R_p_c, const cv::Matx33d& intrinsics,
    const cv::Vec4d& distortion_coeffs,
    const double& inlier_error,
    const double& success_probability,
    std::vector<uchar>& inlier_markers);

  int GenerateImuXform(int image_counter, cv::Matx33f &rotation_t0_t1_cam0,
    cv::Matx33f &rotation_t0_t1_cam1);

  int ProcessPoints(std::vector<cv::Point2f> pts_cam0, std::vector<cv::Point2f> pts_cam1);

  int OuputTrackedPoints(std::vector<cv::Point2f> pts_cam0, std::vector<cv::Point2f> pts_cam1,
    std::vector<unsigned int> ids);

  // Local version of input params
  YAML::Node input_params_;

  // Thread mgmt
  std::atomic <bool> is_running_;
  std::thread image_processor_thread_;
  
  // Video I/O
  std::unique_ptr<Camera> cam0_;
  std::unique_ptr<Camera> cam1_;
  std::unique_ptr<CameraTrigger> trigger_;

  // config params for LK optical flow
  int window_size_;
  int max_pyramid_level_;

  // Config params for goodFeaturesToTrack
  int max_corners_;
  float quality_level_;
  int min_dist_;

  // Running config params
  int max_error_counter_;
  double stereo_threshold_;

  // Config Params for RANSAC
  double ransac_threshold_;

  // Stereo Camera Parameters
  cv::Matx33d K_cam0_;
  cv::Matx33d K_cam1_;
  cv::Vec4d D_cam0_;
  cv::Vec4d D_cam1_;
  cv::Matx33d R_cam0_cam1_;
  cv::Vec3f T_cam0_cam1_;
  cv::Matx33d E_;
  cv::Matx33d F_;
  cv::Matx33d R_cam0_;
  cv::Matx33d R_cam1_;
  cv::Matx34d P_cam0_;
  cv::Matx34d P_cam1_;
  cv::Matx44d Q_;
  cv::Matx33f R_imu_cam0_;
  cv::Matx33f R_imu_cam1_;

  // IMU objects
  std::queue<mavlink_imu_t> imu_queue_;
  std::mutex imu_queue_mutex_;

  // Class output and it's mutex guard
  std::mutex output_mutex_;
  ImagePoints output_points_;
  std::condition_variable output_cond_var_;
};

#endif  // INCLUDE_FLY_STEREO_IMAGE_PROCESSOR_H_
