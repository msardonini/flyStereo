#include "flyStereo/pipeline.h"

#ifdef WITH_VPI
#include "flyStereo/image_processing/vpi_backend.h"
#endif
#include "flyStereo/image_processing/cv_backend.h"
#include "flyStereo/sensor_io/image_sink.h"
#include "flyStereo/utility.h"
#include "flyStereo/visualization/draw_to_image.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

static constexpr unsigned int max_consecutive_missed_frames = 10;

template <typename IpBackend>
Pipeline<IpBackend>::Pipeline(const YAML::Node &params, const YAML::Node &stereo_calibration)
    : params_(params),
      image_processor_(15, stereo_calibration,
                       utility::eulerAnglesToRotationMatrix(params["R_imu_cam0"].as<std::vector<double>>())),
      vio_(StereoCalibration(stereo_calibration),
           utility::eulerAnglesToRotationMatrix(params["R_imu_cam0"].as<std::vector<double>>()),
           cv::Vec3d{params["vio_calibration_rvec"].as<std::array<double, 3>>().data()},
           cv::Vec3d{params["vio_calibration_tvec"].as<std::array<double, 3>>().data()})
#ifdef WITH_VIZ
      ,
      visualization_(utility::eulerAnglesToRotationMatrix(params["R_imu_cam0"].as<std::vector<double>>()))
#endif
{
}

template <typename IpBackend>
Pipeline<IpBackend>::~Pipeline() {
  // Shutdown the pipeline, this will stop the threads
  is_running_ = false;
  if (process_thread_.joinable()) {
    process_thread_.join();
  }
  // if (imu_thread_.joinable()) {
  //   imu_thread_.join();
  // }
}

template <typename IpBackend>
fs::path Pipeline<IpBackend>::CreateLogDir(const fs::path &root_log_dir) {
  // First make sure our logging directory exists
  fs::path log_dir;
  int run_number = 1;
  do {
    std::stringstream run_str;
    run_str << std::internal << "run" << std::setfill('0') << std::setw(3) << run_number++;
    log_dir = root_log_dir / run_str.str();
  } while (fs::exists(log_dir));

  fs::create_directory(log_dir);

  return log_dir;
}

template <typename IpBackend>
void Pipeline<IpBackend>::InitSpdFileLog(const fs::path &log_dir) {
  int max_bytes = 1048576 * 20;  // Max 20 MB
  int max_files = 20;

  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  sinks.back()->set_level(spdlog::level::info);
  sinks.push_back(
      std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_dir / "console_log.txt", max_bytes, max_files));
  sinks.back()->set_level(spdlog::level::debug);
  auto flyMS_log = std::make_shared<spdlog::logger>("flyStereo_log", std::begin(sinks), std::end(sinks));

  // Register the logger to the global level
  flyMS_log->set_level(spdlog::level::debug);
  spdlog::register_logger(flyMS_log);
  spdlog::set_default_logger(flyMS_log);
}

template <typename IpBackend>
void Pipeline<IpBackend>::Init() {
  // Block until we receive a start command
  if (params_["wait_for_start_command"].as<bool>()) {
    while (is_running_.load()) {
      if (mavlink_reader_.WaitForStartCmd() == true) {
        break;
      }
    }
  }

  // If configured, create a logging directory
  if (params_["record_mode"] && params_["record_mode"]["enable"].as<bool>()) {
    auto log_dir = CreateLogDir(params_["record_mode"]["log_root_dir"].as<std::string>());
    InitSpdFileLog(log_dir);
    sql_sink_.Init(log_dir);
    record_mode_ = true;
  }
  if (params_["replay_mode"] && params_["replay_mode"]["enable"].as<bool>()) {
    auto tmp_src = std::make_unique<SqlSrc>();
    tmp_src->Init(params_["replay_mode"]["replay_dir"].as<std::string>());

    data_src_ = std::move(tmp_src);
  } else {
    // Initialize the sensor suite
    // mavlink_reader_.Init(params_);
    // camera_src_.Init();

    data_src_ = std::make_unique<OakD>(30, false);
  }

  image_processor_.Init();

  // imu_thread_ = std::thread(&Pipeline<IpBackend>::imu_thread, this);
  is_running_ = true;
  process_thread_ = std::thread(&Pipeline<IpBackend>::run, this);
}

template <typename IpBackend>
void Pipeline<IpBackend>::shutdown() {
  is_running_ = false;
}

// // Thread to collect the imu data and disperse it to all objects that need it
// template <typename IpBackend>
// void Pipeline<IpBackend>::imu_thread() {
//   mavlink_reader_.ResetShutdownCmds();
//   while (is_running_.load()) {
//     mavlink_imu_t attitude;
//     // Get the next attitude message, block until we have one
//     if (mavlink_reader_.GetAttitudeMsg(&attitude, true)) {
//       // Send the imu message to the image processor
//       camera_src_.ReceiveImu(attitude);
//     }

//     // Check if we have been given the shutdown command
//     if (mavlink_reader_.CheckForShutdownCmd()) {
//       is_running_ = false;
//     }
//   }
// }

template <typename IpBackend>
void Pipeline<IpBackend>::run() {
  auto consecutive_missed_frames = 0u;

  ImageSink cam0_sink(params_["ImageSinkCam0"]);
  ImageSink cam1_sink(params_["ImageSinkCam1"]);

  while (is_running_.load()) {
    // Read the frames and check for errors
    cv::Mat_<uint8_t> raw_frame_cam0_t1, raw_frame_cam1_t1;
    std::vector<mavlink_imu_t> imu_msgs;
    uint64_t current_time;

    int ret_val = data_src_->GetSynchronizedData(raw_frame_cam0_t1, raw_frame_cam1_t1, imu_msgs, current_time);

    typename IpBackend::image_type d_frame_cam0_t1 = raw_frame_cam0_t1;
    typename IpBackend::image_type d_frame_cam1_t1 = raw_frame_cam1_t1;

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

    if (record_mode_) {
      LogParams<typename IpBackend::image_type> log_params;

      log_params.timestamp_frame = current_time;
      log_params.frame0 = d_frame_cam0_t1;
      log_params.frame1 = d_frame_cam1_t1;
      log_params.imu_msgs = imu_msgs;

      sql_sink_.ProcessFrame(log_params);
    }
    // Process the frames
    TrackedImagePoints<IpBackend> tracked_image_points =
        image_processor_.process_image(std::move(d_frame_cam0_t1), std::move(d_frame_cam1_t1), imu_msgs, current_time);

    auto [body_pose, inliers] = vio_.ProcessPoints(tracked_image_points);

#ifdef WITH_VIZ
    visualization_.ReceiveData(body_pose, inliers);
#endif
    // // mavlink_reader_.SendVioMsg(vio_data);

    /*********************************************************************
     * Output Images to the sink, if requested
     *********************************************************************/

    // if (cam0_sink) {
    //   UMat<cv::Vec3b> show_frame_color(d_frame_cam0_t1.size());
    //   UMat<uint8_t> show_frame(d_frame_cam0_t1);
    //   cv::cvtColor(show_frame.frame(), show_frame_color.frame(), cv::COLOR_GRAY2BGR);
    //   if (draw_points_to_frame_) {
    //     DrawPoints(tracked_image_points.cam0_t1, show_frame_color.frame());
    //   }
    //   // DrawPoints(debug_pts, show_frame);
    //   cam0_sink.SendFrame(show_frame_color);
    // }

    // if (cam1_sink) {
    //   UMat<cv::Vec3b> show_frame_color(d_frame_cam1_t1.size());
    //   UMat<uint8_t> show_frame = d_frame_cam1_t1.frame().clone();
    //   cv::cvtColor(show_frame.frame(), show_frame_color.frame(), cv::COLOR_GRAY2BGR);
    //   if (draw_points_to_frame_) {
    //     DrawPoints(tracked_image_points.cam1_t1, show_frame_color.frame());
    //   }
    //   cam1_sink.SendFrame(show_frame_color);
    // }
  }
}

template class Pipeline<CvBackend>;
#ifdef WITH_VPI
template class Pipeline<VpiBackend>;
#endif
