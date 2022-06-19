#pragma once

#include <atomic>
#include <thread>

#include "flyStereo/image_processing/cv_backend.h"
#include "flyStereo/image_processing/image_processor.h"
#include "flyStereo/image_processing/vpi_backend.h"
#include "flyStereo/sensor_io/mavlink_reader.h"
#include "flyStereo/sensor_io/sensor_interface.h"
#include "flyStereo/vio.h"
#include "yaml-cpp/yaml.h"

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

  std::thread process_thread_;
  std::thread imu_thread_;
  std::atomic<bool> is_running_;

  const YAML::Node params_;

  MavlinkReader mavlink_reader_;
  SensorInterface<IpBackend> sensor_interface_;
  ImageProcessor<IpBackend> image_processor_;
  Vio<IpBackend> vio_;

  bool draw_points_to_frame_ = true;
};
