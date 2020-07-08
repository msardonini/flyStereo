#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>

#include "yaml-cpp/yaml.h"
#include "fly_stereo/mavlink/fly_stereo/mavlink.h"

#ifndef INCLUDE_FLY_STEREO_MAVLINK_READER_H_
#define INCLUDE_FLY_STEREO_MAVLINK_READER_H_

class MavlinkReader {
 public:
  MavlinkReader();
  ~MavlinkReader();

  int Init(YAML::Node input_params);

  bool GetAttitudeMsg(mavlink_imu_t* attitude, bool block = false);
  void SendCounterReset();
 private:
  void SerialReadThread();
  std::atomic<bool> is_running_;
  std::thread reader_thread_;
  std::mutex queue_mutex_;
  int serial_dev_ = 0;
  std::queue<mavlink_imu_t> output_queue_;
  std::condition_variable cond_var_;

  int replay_file_fd_ = 0;

};

#endif  // INCLUDE_FLY_STEREO_MAVLINK_READER_H_
