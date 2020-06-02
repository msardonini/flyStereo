#ifndef INCLUDE_FLY_STEREO_IMAGE_PROCESSOR_H_
#define INCLUDE_FLY_STEREO_IMAGE_PROCESSOR_H_

#include <iostream>
#include <string>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>

#include "yaml-cpp/yaml.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"


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

 private:
  int InitFirstFrame(const cv::Mat &frame_cam0_t0,
                     const cv::Mat &frame_cam1_t0);

  int ProcessThread();

  void DrawPoints(const std::vector<cv::Point2f> &mypoints, cv::Mat &myimage);

  int StereoMatch(cv::Mat frame_cam1_t1, std::vector<unsigned char> &status);

  int RemoveOutliers(const cv::Size framesize, std::vector<uchar> &status,
    std::vector<cv::Point2f> &points);

  int DetectNewFeatures(cv::Mat frame_cam0_t0, cv::Ptr<cv::Feature2D>
    detector_ptr);


  void rescalePoints(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2,
    float& scaling_factor);

  int twoPointRansac(
    const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
    const cv::Matx33f& R_p_c, const cv::Matx33d& intrinsics,
    const cv::Vec4d& distortion_coeffs,
    const double& inlier_error,
    const double& success_probability,
    std::vector<uchar>& inlier_markers);

  // Thread mgmt
  std::atomic <bool> is_running_;
  std::thread image_processor_thread_;
  
  // Video I/O
  std::string src_pipeline_cam0_;
  std::string sink_pipeline_cam0_;
  std::unique_ptr<cv::VideoCapture> reader_cam0_;
  std::unique_ptr<cv::VideoWriter> writer_cam0_;
  std::string src_pipeline_cam1_;
  std::string sink_pipeline_cam1_;
  std::unique_ptr<cv::VideoCapture> reader_cam1_;
  std::unique_ptr<cv::VideoWriter> writer_cam1_;

  // OpenCV
  std::vector<cv::Mat> pyramid_cam0_t0_;
  std::vector<cv::Mat> pyramid_cam0_t1_;
  std::vector<cv::Mat> pyramid_cam1_t0_;
  std::vector<cv::Mat> pyramid_cam1_t1_;
  std::vector<cv::Point2f> corners_cam0_t0_;
  std::vector<cv::Point2f> corners_cam0_t1_;
  std::vector<cv::Point2f> corners_cam1_t0_;
  std::vector<cv::Point2f> corners_cam1_t1_;

  // config params for LK optical flow
  int window_size_;
  int max_pyramid_level_;

  // Config params for goodFeaturesToTrack
  int max_corners_;
  float quality_level_;
  int min_dist_;

  // Running config params
  int max_error_counter_;
  int framerate_;
  double stereo_threshold_;

  // Config Params for RANSAC
  double ransac_threshold_;

  // Stereo Camera Parameters
  cv::Matx33d K_cam0_;
  cv::Matx33d K_cam1_;
  cv::Vec4d D_cam0_;
  cv::Vec4d D_cam1_;
  cv::Matx33f R_cam1_cam0_;
  cv::Vec3f T_cam1_cam0_;
  cv::Matx33d E_;
  cv::Matx33d F_;
  cv::Matx33d R_cam0_;
  cv::Matx33d R_cam1_;
  cv::Matx34d P_cam0_;
  cv::Matx34d P_cam1_;
  cv::Matx44d Q_;
};


#endif  // INCLUDE_FLY_STEREO_IMAGE_PROCESSOR_H_
