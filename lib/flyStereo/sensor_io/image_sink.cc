#include "flyStereo/sensor_io/image_sink.h"

ImageSink::ImageSink(const std::string_view sink_pipeline, cv::Size frame_size, int framerate)
    : sink_pipeline_(sink_pipeline), frame_size_(frame_size), framerate_(framerate) {}

ImageSink::ImageSink(const YAML::Node& config)
    : ImageSink(config["sink_pipeline"].as<std::string>(),
                cv::Size(config["width"].as<int>(), config["height"].as<int>()), config["framerate"].as<int>()) {}

bool ImageSink::Init(bool is_color) {
  // If configured, create the image sinks
  if (!sink_pipeline_.empty()) {
    cam_sink_ = std::make_unique<cv::VideoWriter>(sink_pipeline_, 0, framerate_, frame_size_, is_color);
    if (!cam_sink_->isOpened()) {
      throw std::runtime_error("Error! VideoWriter did not open");
    }
  } else {
    throw std::runtime_error("Error! Called GetFrame without defining the sink pipeline");
  }
  return 0;
}
