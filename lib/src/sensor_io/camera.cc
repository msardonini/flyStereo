#include "fly_stereo/sensor_io/camera.h"

#include <iostream>
#include <thread>

#include "opencv2/cudaarithm.hpp"
#include "spdlog/spdlog.h"


Camera::Camera(YAML::Node input_params, bool replay_mode) {
  replay_mode_ = replay_mode;

  hardware_trigger_mode_ = input_params["hardware_trigger_mode"].as<bool>();
  gain_ = input_params["gain"].as<int>();
  exposure_time_ = input_params["exposure_time"].as<int>();
  device_num_ = input_params["device_num"].as<int>();
  src_pipeline_ = input_params["src_pipeline"].as<std::string>();

  // If using the gstreamer pipeline, load those params
  if (input_params["gstreamer_pipeline"]) {
    YAML::Node gst_params  = input_params["gstreamer_pipeline"];
    use_gstreamer_pipeline_ = gst_params["enable"].as<bool>();
  }

  // If using the image sink, load the params
  if (input_params["sink_pipeline"]) {
    sink_pipeline_ = input_params["sink_pipeline"].as<std::string>();
  }

  height_ = input_params["height"].as<int>();
  width_ = input_params["width"].as<int>();
  framerate_ = input_params["framerate"].as<int>();

  enable_videoflip_ = input_params["enable_videoflip"].as<bool>();
  if (enable_videoflip_) {
    flip_method_ = input_params["flip_method"].as<int>();
  }

  // If using the image sink, load the params
  if (input_params["auto_exposure"] && input_params["auto_exposure"]["enable"].as<bool>()) {
    YAML::Node auto_exposure_node = input_params["auto_exposure"];
    auto_exposure_update_percentage_ = auto_exposure_node["auto_exposure_update_percentage"].as<
      float>();
    pixel_range_limits_ = auto_exposure_node["pixel_range_limits"].as<std::array<
      unsigned int, 2> >();
    exposure_limits_ = auto_exposure_node["exposure_limits"].as<std::array<unsigned int, 2> >();
    num_frames_to_calc_ = auto_exposure_node["num_frames_to_calc"].as<int>();
    curr_frame_ = 0;
    auto_exposure_ = true;
  } else {
    auto_exposure_ = false;
  }
}

Camera::~Camera() {}

int Camera::Init() {
  // In replay mode we are not accessing hardware here
  if (replay_mode_) {
    return 0;
  }

  if (use_gstreamer_pipeline_) {
    InitGstPipeline();
  } else {
    std::cout << "pipeline is " << src_pipeline_ << std::endl;
    cam_src_ = std::make_unique<cv::VideoCapture> (src_pipeline_);
    if (!cam_src_->isOpened()) {
      std::cerr << "Error! VideoCapture on cam" << device_num_ << " did not open" << std::endl;
      return -1;
    }
  }

  // Enable the hardware trigger mode if requested
  if (hardware_trigger_mode_) {
    // Sleep for a bit to allow some time for the camera to wake up and start to receive messages
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::string trigger_cmd("v4l2-ctl -d " + std::to_string(device_num_) + " -c trigger_mode=1");
    if(system(trigger_cmd.c_str())) {
      std::cerr << "Error setting trigger mode" << std::endl;
    }

    // Get all of the non-triggered frames out of the pipeline
    if (use_gstreamer_pipeline_) {
      cv::Mat not_used;
      while (GetFrameGst(not_used) != 1) {}
    }
  }

  UpdateGain();

  UpdateExposure();

  return 0;
}

int Camera::UpdateGain() {
  spdlog::debug("Setting Camera {} to gain {}", std::to_string(device_num_),
    std::to_string(gain_));
  std::string gain_cmd("v4l2-ctl -d " + std::to_string(device_num_) + " -c gain=" +
    std::to_string(gain_));
  if (system(gain_cmd.c_str())) {
    std::cerr << "Error setting gain" << std::endl;
  }
  return 0;
}

int Camera::UpdateExposure() {
  // Set hard limits on the min/max values of the camera
  if (exposure_time_ < 1) {
    exposure_time_ = 1;
  } else if (exposure_time_ > 65535) {
    exposure_time_ = 65535;
  }

  spdlog::debug("Setting Camera {} to exposure {}", std::to_string(device_num_), std::to_string(
    exposure_time_));

  std::string exposure_cmd("v4l2-ctl -d " + std::to_string(device_num_) + " -c exposure=" +
    std::to_string(exposure_time_));
  if (system(exposure_cmd.c_str())) {
    std::cerr << "Error setting exposure time" << std::endl;
  }
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

int Camera::RunAutoExposure(const cv::Scalar &mean_pixel_val) {
  if (mean_pixel_val(0) < pixel_range_limits_[0]) {
    // If the image is too dark then update the exposure time first, then gain when that maxes
    // Also, calculate the value the exposure time would get updated to
    unsigned int new_exposure_time = exposure_time_ * (1.0f + auto_exposure_update_percentage_);
    // Check for the edge case where the update is less than 1 ms
    if (new_exposure_time == exposure_time_) {
      new_exposure_time = exposure_time_ + 1;
    }

    if (new_exposure_time <= exposure_limits_[1]) {
      exposure_time_ = new_exposure_time;
      UpdateExposure();
    } else if (gain_ < 15) {  // Only update the gain within its limits
      gain_++;
      UpdateGain();
    } else {
      spdlog::warn("Value of gain and exposure time at maximum on camera {}, cannot make image"
        " brighter", device_num_);
    }
  } else if (mean_pixel_val(0) > pixel_range_limits_[1]) {
    unsigned int new_exposure_time = exposure_time_ * (1.0f - auto_exposure_update_percentage_);
    // Check for the edge case where the update is less than 1 ms
    if (new_exposure_time == exposure_time_) {
      new_exposure_time = exposure_time_ - 1;
    }
    // If the image is too bright lower the gain first, then move to exposure time
    if (gain_ > 1) {
      gain_--;
      UpdateGain();
    } else if (new_exposure_time >= exposure_limits_[0]) {
      exposure_time_ = new_exposure_time;
      UpdateExposure();
    } else {
      spdlog::warn("Value of gain and exposure time at minimum on camera {}, cannot make image"
        " darker", device_num_);
    }
  }
  return 0;
}


int Camera::GetFrame(cv::Mat &frame) {
  if (use_gstreamer_pipeline_) {
    return GetFrameGst(frame);
  } else {
    if (!cam_src_->read(frame)) {
      return -1;
    }
  }
  return 0;
}

int Camera::GetFrame(cv::cuda::GpuMat &frame) {
  cv::Mat host_frame;

  int ret = GetFrame(host_frame);
  if (ret != 0) {
    return ret;
  }

  // Save a local copy in case a CPU version is wanted later
  frame_ = host_frame;

  if (enable_videoflip_) {
    cv::cuda::GpuMat temp;
    temp.upload(host_frame);
    cv::cuda::flip(temp, frame, flip_method_);
  } else {
    frame.upload(host_frame);
  }

  if (auto_exposure_) {
    if (++curr_frame_ == num_frames_to_calc_) {
      curr_frame_ = 0;
      cv::Scalar mean, st_dev;
      cv::cuda::meanStdDev(frame, mean, st_dev);

      // Run and apply the auto exposure if necessary
      RunAutoExposure(mean);
    }
  }
  return 0;
}

int Camera::SendFrame(cv::Mat &frame) {
  if (!cam_sink_) {
    if(InitSink(frame.channels() > 1)) {
      return -1;
    }
  }
  cam_sink_->write(frame);
  return 0;
}

int Camera::SendFrame(cv::cuda::GpuMat &d_frame) {
  cv::Mat frame;
  d_frame.download(frame);
  return SendFrame(frame);
}

int Camera::InitGstPipeline() {
  // Initialize GStreamer
  gst_init(nullptr, nullptr);

  // Create the main loop
  gst_params_.loop = g_main_loop_new(NULL, FALSE);

  // Create the pipeline
  gst_params_.pipeline = gst_pipeline_new("fly_stereo_src");

  // Create all the elements (include caps)
  gst_params_.v4l2src = gst_element_factory_make("v4l2src", NULL);
  gst_params_.capsfilter = gst_element_factory_make("capsfilter", NULL);
  gst_params_.nvvidconv = gst_element_factory_make("videoconvert", NULL);
  gst_params_.capsfilter2 = gst_element_factory_make("capsfilter", NULL);
  gst_params_.appsink = gst_element_factory_make("appsink", NULL);
  gst_bin_add_many(GST_BIN(gst_params_.pipeline), gst_params_.v4l2src, gst_params_.capsfilter,
    gst_params_.nvvidconv, gst_params_.appsink, NULL);

  // Check the elements were created
  if (!gst_params_.v4l2src || !gst_params_.nvvidconv || !gst_params_.capsfilter ||
    !gst_params_.appsink ) {
    g_warning("Could not create the elements\n");
    return -1;
  }

  // v4l2src configuration
  g_object_set(G_OBJECT(gst_params_.v4l2src), "device", std::string("/dev/video" +
    std::to_string(device_num_)).c_str(), NULL);

  // Caps filter for v4l2src
  const GstCaps *caps = gst_caps_new_simple("video/x-raw", "width", G_TYPE_INT, 1280, "height",
    G_TYPE_INT, 720, NULL);
  g_object_set(gst_params_.capsfilter, "caps", caps, NULL);

  // Appsink configuration
  g_object_set(G_OBJECT(gst_params_.appsink), "max-buffers", gst_params_.appsink_max_buffers,
    NULL);
  g_object_set(G_OBJECT(gst_params_.appsink), "drop", false, NULL);

  // Set the params for the caps filters
  const GstCaps *caps2 = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "GRAY8",
    NULL);
  gst_app_sink_set_caps(reinterpret_cast<GstAppSink *>(gst_params_.appsink), caps2);

  int ret = gst_element_link_many(gst_params_.v4l2src, gst_params_.capsfilter,
    gst_params_.nvvidconv, gst_params_.appsink, NULL);

  if (!ret) {
    g_warning("Failed to link the elements!");
    return ret;
  }

  GstStateChangeReturn ret_state = gst_element_set_state(gst_params_.pipeline, GST_STATE_PLAYING);
  if (ret_state != GST_STATE_CHANGE_SUCCESS && ret_state != GST_STATE_CHANGE_ASYNC) {
    std::cerr << "Error changing gstreamer state! Ret state is " << ret_state << std::endl;
    return -1;
  }


  return 0;
}

int Camera::GetFrameGst(cv::Mat &frame) {
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
  GstCaps* caps = gst_sample_get_caps(sample);
  GstStructure* structure = gst_caps_get_structure(caps, 0);

  // Extract the info we need from this caps object
  gint width, height, framerate_numerator, framerate_denominator;
  if (!(gst_structure_get_int(structure, "width", &width)
    && gst_structure_get_int(structure, "height", &height)
    && gst_structure_get_fraction(structure, "framerate", &framerate_numerator,
      &framerate_denominator))) {
    std::cerr << "Error getting metadata" << std::endl;
    return -1;
  }

  // Check to see if the frame has been initialized
  if (frame.empty()) {
    frame = cv::Mat(720, 1280, CV_8UC1);
  }

  // Get the frame from gstreamer
  GstMapInfo info;
  if (gst_buffer_map(buffer, &info, GST_MAP_READ)) {
    // Copy the buffer
    memcpy(frame.ptr(), info.data, info.size);

    timestamp_ns_ = static_cast<uint64_t>(GST_BUFFER_PTS(buffer));

    gst_buffer_unmap(buffer, &info);
    gst_sample_unref(sample);
  } else {
    std::cerr << "Error getting map" << std::endl;
    return -1;
  }

  return 0;
}

