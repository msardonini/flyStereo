#pragma once

#include <string>

#include "opencv2/core/types.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/core/cuda.hpp"
#include "yaml-cpp/yaml.h"

class ImageSink {
 public:
  ImageSink(const YAML::Node& config);

  ImageSink(const std::string_view pipeline, cv::Size frame_size, int framerate);

  operator bool() { return !sink_pipeline_.empty();}

  int operator << (cv::Mat& frame) { return SendFrame(frame);}


  bool Init(bool is_color);
  int SendFrame(cv::Mat &frame);
  int SendFrame(cv::cuda::GpuMat &d_frame);

 private:
  std::unique_ptr<cv::VideoWriter> cam_sink_;
  std::string sink_pipeline_;
  cv::Size frame_size_;
  int framerate_;
};