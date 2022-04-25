#pragma once

// System includes

// Package includes
#include "flyStereo/interface.h"
#include "flyStereo/sensor_io/camera.h"
#include "flyStereo/sensor_io/camera_trigger.h"
#include "flyStereo/sensor_io/mavlink_reader.h"
#include "flyStereo/sql_logger.h"
#include "yaml-cpp/yaml.h"

class SensorInterface {
 public:
  SensorInterface();
  ~SensorInterface();

  SensorInterface(const SensorInterface &) = delete;
  SensorInterface(SensorInterface &&) = delete;

  int GetSynchronizedData(cv::cuda::GpuMat &d_frame_cam0, cv::cuda::GpuMat &d_frame_cam1,
                          std::vector<mavlink_imu_t> &imu_data, uint64_t &current_frame_time);

  int Init(YAML::Node input_params);

  void ReceiveImu(mavlink_imu_t imu_msg);

  int AssociateImuData(std::vector<mavlink_imu_t> &imu_msgs, uint64_t &current_frame_time);

  static int GenerateImuXform(const std::vector<mavlink_imu_t> &imu_msgs, const cv::Matx33f R_imu_cam0,
                              const cv::Matx33f R_imu_cam1, cv::Matx33f &rotation_t0_t1_cam0,
                              const uint64_t current_frame_time, cv::Matx33f &rotation_t0_t1_cam1);

  std::unique_ptr<CameraTrigger> camera_trigger_;
  std::unique_ptr<Camera> cam0_;
  std::unique_ptr<Camera> cam1_;

 private:
  bool first_iteration_ = true;

  // first value in the pair contains the current value, second is the value from the
  // previous iteration
  using TrigTimestamp = std::pair<int, uint64_t>;
  std::pair<TrigTimestamp, TrigTimestamp> triggers_;

  // IMU objects
  std::queue<mavlink_imu_t> imu_queue_;
  std::mutex imu_queue_mutex_;
  uint64_t min_camera_dt_ms_;

  // Var used to keep the timing during replay
  uint64_t last_replay_time_ = 0;
  uint64_t last_replay_time_recorded_ = 0;
  float replay_speed_multiplier_ = 1;

  // Config params
  bool record_mode_ = false;
  bool replay_mode_ = false;
  std::unique_ptr<SqlLogger> sql_logger_;
};
