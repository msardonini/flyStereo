#pragma once

#include <iostream>
#include <memory>
#include <string>

#include "gst/app/gstappsink.h"
#include "gst/gst.h"
#include "gst/video/video.h"
#include "opencv2/core.hpp"
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

  int GetFrame(cv::Mat_<uint8_t> &frame) {
    if (use_gstreamer_pipeline_) {
      return GetFrameGst(frame);
    } else {
      cv::Mat frame_tmp;
      if (!cam_src_->read(frame_tmp)) {
        return -1;
      }
      frame = frame_tmp;
    }

    if (enable_videoflip_) {
      cv::Mat_<uint8_t> temp(frame);
      cv::flip(temp, frame, flip_method_);
    }

    if (auto_exposure_) {
      if (++curr_frame_ == num_frames_to_calc_) {
        curr_frame_ = 0;
        cv::Scalar mean, st_dev;
        cv::meanStdDev(frame, mean, st_dev);

        // Run and apply the auto exposure if necessary
        RunAutoExposure(mean);
      }
    }
    return 0;
  }

  uint64_t GetTimestampNs() const { return timestamp_ns_; }

  // Gets the latest cv frame already acquired from the hardware. Does NOT get a frame from the
  // Camera
  cv::Mat GetFrameCopy() { return frame_; }

 private:
  // Initialize the image sink object
  int UpdateGain();
  int UpdateExposure();
  int InitGstPipeline();

  int GetFrameGst(cv::Mat_<uint8_t> &frame) {
    // Pull in the next sample
    GstAppSink *appsink = reinterpret_cast<GstAppSink *>(gst_params_.appsink);

    GstSample *sample = gst_app_sink_try_pull_sample(appsink, 2.5E7);  // timeout of 0.25 seconds
    // At EOS or timeout this will return NULL
    if (sample == NULL) {
      if (gst_app_sink_is_eos(appsink)) {
        std::cerr << "Gstreamer End of Stream!" << std::endl;
        return -1;
      } else {  // Else this was a timeout
        std::cerr << "timeout on device: " << device_num_ << std::endl;
        return 1;
      }
    }

    // Take the buffer from the supplied sample
    GstBuffer *buffer = gst_sample_get_buffer(sample);

    // Get the caps & structure objects from the GstSample
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);

    // Extract the info we need from this caps object
    gint width, height, framerate_numerator, framerate_denominator;
    if (!(gst_structure_get_int(structure, "width", &width) && gst_structure_get_int(structure, "height", &height) &&
          gst_structure_get_fraction(structure, "framerate", &framerate_numerator, &framerate_denominator))) {
      std::cerr << "Error getting metadata" << std::endl;
      return -1;
    }

    // Check to see if the frame has been initialized
    if (frame.size() != cv::Size(1280, 720)) {
      frame = cv::Mat_<uint8_t>(cv::Size(1280, 720));
    }

    // Get the frame from gstreamer
    GstMapInfo info;
    if (gst_buffer_map(buffer, &info, GST_MAP_READ)) {
      // Copy the buffer
      memcpy(frame.data, info.data, info.size);

      timestamp_ns_ = static_cast<uint64_t>(GST_BUFFER_PTS(buffer));

      gst_buffer_unmap(buffer, &info);
      gst_sample_unref(sample);
    } else {
      std::cerr << "Error getting map" << std::endl;
      return -1;
    }

    return 0;
  }

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
