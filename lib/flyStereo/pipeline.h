#pragma once

#include <atomic>
#include <filesystem>
#include <thread>

#include "flyStereo/image_processing/image_processor.h"
#include "flyStereo/sensor_io/mavlink_reader.h"
#include "flyStereo/sensor_io/oakd.h"
#include "flyStereo/sensor_io/sql_sink.h"
#include "flyStereo/sensor_io/sql_src.h"
#include "flyStereo/vio.h"
#ifdef WITH_VIZ
#include "flyStereo/visualization/visualization.h"
#endif
#include "yaml-cpp/yaml.h"

namespace fs = std::filesystem;

template <typename IpBackend>
class Pipeline {
 public:
  Pipeline(const YAML::Node &params, const YAML::Node &stereo_calibration);
  ~Pipeline();
  void Init();

  bool is_running() { return is_running_.load(); }
  void shutdown();

 private:
  // threads
  void run();
  void imu_thread();
  void tracked_features_thread();

  /**
   * @brief Create a unique logging direcotry for this run. The date and time is typically unknown on the embedded
   * hardware, so we increment a directory with the pattern 'runXXX'
   *
   * @param root_log_dir The root directory to create the log directory in
   * @return fs::path The path to the new log directory
   */
  static fs::path CreateLogDir(const fs::path &root_log_dir);

  /**
   * @brief Initialize spdlog to create a file logger in the log directory. It will create a file called
   * 'console_log.txt'
   *
   * @param log_dir The directory to create the log file in
   */
  static void InitSpdFileLog(const fs::path &log_dir);

  std::thread process_thread_;
  // std::thread imu_thread_;
  std::atomic<bool> is_running_;

  const YAML::Node params_;

  MavlinkReader mavlink_reader_;
  // ArducamSystem<typename IpBackend::image_type> camera_src_;

  ImageProcessor<IpBackend> image_processor_;
  Vio<IpBackend> vio_;

  std::unique_ptr<StereoSystemSrcInterface> data_src_;

  SqlSink<typename IpBackend::image_type> sql_sink_;

#ifdef WITH_VIZ
  Visualization visualization_;
#endif
  bool draw_points_to_frame_ = true;

  bool record_mode_ = false;
  bool replay_mode_ = false;
};
