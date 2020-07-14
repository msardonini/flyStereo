#include "fly_stereo/sensor_io/camera.h"

#include <iostream>

#include <opencv2/cudaarithm.hpp>

Camera::Camera(YAML::Node input_params) {
  flip_method_ = input_params["flip_method"].as<int>();
  hardware_trigger_mode_ = input_params["hardware_trigger_mode"].as<bool>();
  gain_ = input_params["gain"].as<int>();
  exposure_time_ = input_params["exposure_time"].as<int>();
  device_num_ = input_params["device_num"].as<int>();
  src_pipeline_ = input_params["src_pipeline"].as<std::string>();
  if (input_params["sink_pipeline"]) {
    sink_pipeline_ = input_params["sink_pipeline"].as<std::string>();
  }
  height_ = input_params["height"].as<int>();
  width_ = input_params["width"].as<int>();
  framerate_ = input_params["framerate"].as<int>();
  if (input_params["auto_exposure"]) {
    YAML::Node auto_exposure_node = input_params["auto_exposure"]; 
    auto_exposure_ = true;
    std::vector<unsigned int> tmp_vec;
    tmp_vec = auto_exposure_node["pixel_range_limits"].as<std::vector<unsigned int> >();
    pixel_range_limits_[0] = tmp_vec[0];
    pixel_range_limits_[1] = tmp_vec[1];
    tmp_vec = auto_exposure_node["exposure_limits"].as<std::vector<unsigned int> >();
    exposure_limits_[0] = tmp_vec[0];
    exposure_limits_[1] = tmp_vec[1];
    num_frames_to_calc_ = auto_exposure_node["num_frames_to_calc"].as<int>();
    curr_frame_ = 0;
  } else {
    auto_exposure_ = false;
  }
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

  UpdateGain();

  UpdateExposure();

  return 0;
}

int Camera::UpdateGain() {
  std::string gain_cmd("v4l2-ctl -d " + std::to_string(device_num_) +
    " -c gain=" + std::to_string(gain_));
  system(gain_cmd.c_str());
  return 0;
}


int Camera::UpdateExposure() {
  std::string exposure_cmd("v4l2-ctl -d " + std::to_string(device_num_) +
    " -c exposure=" + std::to_string(exposure_time_));
  system(exposure_cmd.c_str());
  return 0;
}

int Camera::InitSink(bool is_color) {
  // If configured, create the image sinks
  if (!sink_pipeline_.empty()) {
    cam_sink_ = std::make_unique<cv::VideoWriter> (sink_pipeline_, 0, framerate_, cv::Size(width_,
      height_), is_color);
    if (!cam_sink_->isOpened()) {
      std::cerr << "Error! VideoWriter on cam" << device_num_ << " did not open" << std::endl;
      return -1;
    }
  } else {
    std::cerr << "Error! Called GetFrame without defining the sink pipeline" << std::endl;
    return -1;
  }
  return 0;
}


int Camera::GetFrame(cv::Mat &frame) {
  if (!cam_src_->read(frame)) {
    return -1;
  }

  // cv::flip(frame, frame, flip_method_);
  return 0;
}

int Camera::GetFrame(cv::cuda::GpuMat &frame) {
  cv::Mat host_frame;
  if (GetFrame(host_frame)) {
    return -1;
  }
  frame.upload(host_frame);

  if (auto_exposure_) {
    if (++curr_frame_ == num_frames_to_calc_) {
      curr_frame_ = 0;
      cv::Scalar mean, st_dev;
      cv::cuda::meanStdDev(frame, mean, st_dev);

      if (mean(0) < pixel_range_limits_[0]) {
        if (exposure_time_ >= exposure_limits_[1]) {
          gain_++;
          UpdateGain();
        } else {
          exposure_time_ += 250;
          UpdateExposure();
        }
      } else if (mean(0) > pixel_range_limits_[1]) {
        if (exposure_time_ <= exposure_limits_[0]) {
          gain_--;
          UpdateGain();
        } else {
          exposure_time_ -= 250;
          UpdateExposure();
        }
      }
    }
  }
  return 0;
}

int Camera::SendFrame(cv::Mat &frame) {
  if (cam_sink_) {
    cam_sink_->write(frame);
  } else {
    bool is_color = frame.channels() > 1;
    if(InitSink(is_color)) {
      return -1;
    }
    cam_sink_->write(frame);
  }
  return 0;
}
