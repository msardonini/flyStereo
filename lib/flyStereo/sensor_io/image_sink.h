#pragma once

#include <string>

#include "flyStereo/umat.h"
#include "opencv2/core/cuda.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/videoio.hpp"
#include "yaml-cpp/yaml.h"

class ImageSink {
 public:
  ImageSink(const YAML::Node& config);

  ImageSink(const std::string_view pipeline, cv::Size frame_size, int framerate);

  operator bool() { return !sink_pipeline_.empty(); }

  int operator<<(UMat<uint8_t>& frame) { return SendFrame(frame); }

  bool Init(bool is_color);

  template <typename T>
  int SendFrame(UMat<T>& frame) {
    if (!cam_sink_) {
      if (Init(frame.frame().channels() > 1)) {
        return -1;
      }
    }
    cam_sink_->write(frame.frame());
    return 0;
  }

 private:
  std::unique_ptr<cv::VideoWriter> cam_sink_;
  std::string sink_pipeline_;
  cv::Size frame_size_;
  int framerate_;
};
