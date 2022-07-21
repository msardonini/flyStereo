#pragma once

#include <string>

#include "flyStereo/types/umat.h"
#include "opencv2/core/cuda.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/videoio.hpp"
#include "yaml-cpp/yaml.h"

class ImageSink {
 public:
  ImageSink(const YAML::Node& config);

  ImageSink(const std::string_view pipeline, cv::Size frame_size, int framerate);

  operator bool() { return !sink_pipeline_.empty(); }

  template <typename T>
  int operator<<(cv::Mat_<T>& frame) {
    return SendFrame(frame);
  }

  bool Init(bool is_color);

  template <typename T>
  int SendFrame(cv::Mat_<T>& frame) {
    if (!cam_sink_) {
      if (Init(frame.channels() > 1)) {
        return -1;
      }
    }
    cam_sink_->write(frame);
    return 0;
  }

 private:
  std::unique_ptr<cv::VideoWriter> cam_sink_;
  std::string sink_pipeline_;
  cv::Size frame_size_;
  int framerate_;
};
