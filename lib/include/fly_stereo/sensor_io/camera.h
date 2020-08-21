#ifndef FLY_STEREO_INCLUDE_CAMERA_H_
#define FLY_STEREO_INCLUDE_CAMERA_H_

#include <memory>
#include <string>

#include "yaml-cpp/yaml.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
#include "gst/video/video.h"



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
  explicit Camera(YAML::Node input_params);
  ~Camera();

  Camera() = delete;
  Camera(const Camera &cam) = delete;
  Camera(Camera &&cam) = delete;

  int Init();

  int GetFrame(cv::Mat &frame);
  int SendFrame(cv::Mat &frame);
  int GetFrame(cv::cuda::GpuMat &frame);

  bool OutputEnabled() {
    return static_cast<bool>(!sink_pipeline_.empty());
  }

  uint64_t GetTimestampNs() const {
    return timestamp_ns_;
  }

 private:
  // Initialize the image sink object
  int InitSink(bool is_color);
  int UpdateGain();
  int UpdateExposure();
  int InitGstPipeline();
  int GetFrameGst(cv::Mat &frame);

  int flip_method_;
  bool hardware_trigger_mode_;
  int gain_;
  int exposure_time_;
  std::string src_pipeline_;
  std::string sink_pipeline_;
  int height_;
  int width_;
  int framerate_;
  int device_num_;

  // Params for auto exposure
  bool auto_exposure_;
  int num_frames_to_calc_;
  std::array<unsigned int, 2> pixel_range_limits_;
  std::array<unsigned int, 2> exposure_limits_;
  int curr_frame_;

  std::unique_ptr<cv::VideoCapture> cam_src_;
  std::unique_ptr<cv::VideoWriter> cam_sink_;

  bool use_gstreamer_pipeline_;
  struct GstParams gst_params_;
  uint64_t timestamp_ns_ = 0;
};

#endif  // FLY_STEREO_INCLUDE_CAMERA_H_