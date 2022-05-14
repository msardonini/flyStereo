#pragma once

#include <atomic>
#include <thread>

#include "flyStereo/image_processing/image_processor.h"
#include "flyStereo/sensor_io/mavlink_reader.h"
#include "flyStereo/sensor_io/sensor_interface.h"
#include "flyStereo/vio.h"
#include "yaml-cpp/yaml.h"

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
  SensorInterface sensor_interface_;
  ImageProcessor<> image_processor_;
  Vio vio_;

  bool draw_points_to_frame_ = true;
};
