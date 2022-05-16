#pragma once

#include <memory>
#include <string>

#include "flyStereo/umat.h"
#include "gst/app/gstappsink.h"
#include "gst/gst.h"
#include "gst/video/video.h"
#include "opencv2/core.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/videoio.hpp"
#include "yaml-cpp/yaml.h"

struct GstParams {
  GMainLoop *loop;
  GstElement *pipeline;
  GstElement *v4l2src;
  GstElement *capsfilter;
  GstElement *nvvidconv;
  GstElement *capsfilter2;
  GstElement *appsink;

  // Params
  int appsink_max_buffers;
};

class Camera {
 public:
  explicit Camera(YAML::Node input_params, bool replay_mode = false);
  ~Camera();

  Camera() = delete;
  Camera(const Camera &cam) = delete;
  Camera(Camera &&cam) = delete;

  int Init();

  int GetFrame(UMat<uint8_t> &frame);

  uint64_t GetTimestampNs() const { return timestamp_ns_; }

  // Gets the latest cv frame already acquired from the hardware. Does NOT get a frame from the
  // Camera
  cv::Mat GetFrameCopy() { return frame_; }

 private:
  // Initialize the image sink object
  int UpdateGain();
  int UpdateExposure();
  int InitGstPipeline();
  int GetFrameGst(UMat<uint8_t> &frame);
  int RunAutoExposure(const cv::Scalar &mean_pixel_val);

  // Save a local copy of the latest cv::Mat for reference if needed
  cv::Mat frame_;

  bool enable_videoflip_;
  int flip_method_;
  bool hardware_trigger_mode_;
  int gain_;
  int exposure_time_;
  std::string src_pipeline_;
  int height_;
  int width_;
  int device_num_;

  // Params for auto exposure
  bool auto_exposure_;
  float auto_exposure_update_percentage_;
  int num_frames_to_calc_;
  std::array<unsigned int, 2> pixel_range_limits_;
  std::array<unsigned int, 2> exposure_limits_;
  int curr_frame_;

  std::unique_ptr<cv::VideoCapture> cam_src_;
  std::unique_ptr<cv::VideoWriter> cam_sink_;

  bool replay_mode_;
  bool use_gstreamer_pipeline_;
  struct GstParams gst_params_;
  uint64_t timestamp_ns_ = 0;
};