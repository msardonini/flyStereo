#include "flyStereo/pipeline.h"

#include "spdlog/spdlog.h"
#include "flyStereo/sensor_io/image_sink.h"
#include "flyStereo/visualization/draw_to_image.h"

static constexpr unsigned int max_consecutive_missed_frames = 10;

Pipeline::Pipeline(const YAML::Node &params, const YAML::Node &stereo_calibration)
    : params_(params), image_processor_(params, stereo_calibration), vio_(params, stereo_calibration) {}

Pipeline::~Pipeline() {
  // Shutdown the pipeline, this will stop the threads
  is_running_ = false;
  if (process_thread_.joinable()) {
    process_thread_.join();
  }
  if (imu_thread_.joinable()) {
    imu_thread_.join();
  }
}

void Pipeline::Init() {
  // Block until we receive a start command
  if (params_["wait_for_start_command"].as<bool>()) {
    while (is_running_.load()) {
      if (mavlink_reader_.WaitForStartCmd() == true) {
        break;
      }
    }
  }

  // Initialize the sensor suite
  mavlink_reader_.Init(params_);
  sensor_interface_.Init(params_);
  image_processor_.Init();

  is_running_ = true;

  imu_thread_ = std::thread(&Pipeline::imu_thread, this);
  process_thread_ = std::thread(&Pipeline::run, this);
}

void Pipeline::shutdown() { is_running_ = false; }

// Thread to collect the imu data and disperse it to all objects that need it
void Pipeline::imu_thread() {
  mavlink_reader_.ResetShutdownCmds();
  while (is_running_.load()) {
    mavlink_imu_t attitude;
    // Get the next attitude message, block until we have one
    if (mavlink_reader_.GetAttitudeMsg(&attitude, true)) {
      // Send the imu message to the image processor
      sensor_interface_.ReceiveImu(attitude);
    }

    // Check if we have been given the shutdown command
    if (mavlink_reader_.CheckForShutdownCmd()) {
      is_running_ = false;
    }
  }
}

void Pipeline::run() {
  auto consecutive_missed_frames = 0u;

  ImageSink cam0_sink(params_);
  ImageSink cam1_sink(params_);

  while (is_running_.load()) {
    // Read the frames and check for errors
    cv::cuda::GpuMat d_frame_cam0_t1, d_frame_cam1_t1;
    std::vector<mavlink_imu_t> imu_msgs;
    uint64_t current_time;

    int ret_val = sensor_interface_.GetSynchronizedData(d_frame_cam0_t1, d_frame_cam1_t1, imu_msgs, current_time);
    if (ret_val == -1) {
      if (++consecutive_missed_frames >= max_consecutive_missed_frames) {
        spdlog::error("Error! Failed to read more than {} consecutive frames", max_consecutive_missed_frames);
        is_running_.store(false);
        return;
      } else {
        continue;
      }
    } else if (ret_val == 2) {
      // This will happen if we don't have IMU data, for now continue on as is
      continue;
    } else if (ret_val == -2) {
      spdlog::info("Sensor error or end of replay, shutting down");
      // This will happen if we have a critical error or the replay has finished. Shut down
      is_running_.store(false);
      continue;
    }
    // Reset the Error Counter
    consecutive_missed_frames = 0;

    // Process the frames
    ImagePoints output_points;
    if (image_processor_.process_image(d_frame_cam0_t1, d_frame_cam1_t1, imu_msgs, current_time, output_points)) {
      spdlog::warn("error in image processor");
      continue;
    }

    vio_t vio_data;
    // Send the features to the vio object
    vio_.ProcessPoints(output_points, vio_data);

    mavlink_reader_.SendVioMsg(vio_data);

    /*********************************************************************
     * Output Images to the sink, if requested
     *********************************************************************/

    if (cam0_sink) {
      cv::Mat show_frame, show_frame_color;
      d_frame_cam0_t1.download(show_frame);
      if (draw_points_to_frame_) {
        cv::cvtColor(show_frame, show_frame_color, cv::COLOR_GRAY2BGR);
        DrawPoints(output_points, true, show_frame_color);
      } else {
        show_frame_color = show_frame;
      }
      // DrawPoints(debug_pts, show_frame);
      cam0_sink.SendFrame(show_frame_color);
    }

    if (cam1_sink) {
      cv::Mat show_frame, show_frame_color;
      d_frame_cam1_t1.download(show_frame);
      if (draw_points_to_frame_) {
        cv::cvtColor(show_frame, show_frame_color, cv::COLOR_GRAY2BGR);
        DrawPoints(output_points, false, show_frame_color);
      } else {
        show_frame_color = show_frame;
      }
      cam1_sink.SendFrame(show_frame_color);
    }
  }
}
