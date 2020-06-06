#include "fly_stereo/camera.h"

#include <iostream>

Camera::Camera(YAML::Node input_params) {
  flip_method_ = input_params["flip_method"].as<int>();
  hardware_trigger_mode_ = input_params["hardware_trigger_mode"].as<bool>();
  gain_ = input_params["gain"].as<int>();
  exposure_time_ = input_params["exposure_time"].as<int>();
  device_num_ = input_params["device_num"].as<int>();
  src_pipeline_ = input_params["src_pipeline"].as<std::string>();
  sink_pipeline_ = input_params["sink_pipeline"].as<std::string>();
  height_ = input_params["height"].as<int>();
  width_ = input_params["width"].as<int>();
  framerate_ = input_params["framerate"].as<int>();
}

Camera::~Camera() {}

int Camera::Init() {
  std::cout << "pipeline is " << src_pipeline_ << std::endl;
  cam_src_ = std::make_unique<cv::VideoCapture> (src_pipeline_);
  if (!cam_src_->isOpened()) {
    std::cerr << "Error! VideoCapture on cam" << device_num_ << " did not open"
      << std::endl;
    return -1;
  }

  // Enable the hardware trigger mode if requested
  if (hardware_trigger_mode_) {
    std::string trigger_cmd("v4l2-ctl -d " + std::to_string(device_num_) + 
      " -c trigger_mode=1");
    system(trigger_cmd.c_str());
  }

  std::string gain_cmd("v4l2-ctl -d " + std::to_string(device_num_) + 
    " -c gain=" + std::to_string(gain_));
  system(gain_cmd.c_str());

  std::string exposure_cmd("v4l2-ctl -d " + std::to_string(device_num_) +
    " -c exposure=" + std::to_string(exposure_time_));
  system(exposure_cmd.c_str());

  // If configured, create the image sinks
  if (!sink_pipeline_.empty()) {
    cam_sink_ = std::make_unique<cv::VideoWriter> (sink_pipeline_, 0,
      framerate_, cv::Size(width_,height_), false);
    if (!cam_sink_->isOpened()) {
      std::cerr << "Error! VideoWriter on cam" << device_num_ <<
        " did not open" << std::endl;
      return -1;
    }
  }
  return 0;
}

int Camera::GetFrame(cv::Mat &frame) {
  if (!cam_src_->read(frame)) {
    return -1;
  }

  cv::flip(frame, frame, flip_method_);
  return 0;
}

int Camera::SendFrame(cv::Mat &frame) {
  if (cam_sink_) {
    cam_sink_->write(frame);
  }
}

