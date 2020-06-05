#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>

#include "yaml-cpp/yaml.h"
#include "mavlink/common/mavlink.h"
#include "mavlink/common/mavlink_msg_attitude.h"

#ifndef INCLUDE_FLY_STEREO_MAVLINK_READER_H_
#define INCLUDE_FLY_STEREO_MAVLINK_READER_H_

class MavlinkReader {
 public:
  MavlinkReader();
  ~MavlinkReader();

  int Init(YAML::Node input_params);

  bool GetAttitudeMsg(mavlink_attitude_t* attitude, bool block = false);

 private:
  void SerialReadThread();
  std::atomic<bool> is_running_;
  std::thread reader_thread_;
  std::mutex queue_mutex_;
  int serial_dev_ = 0;
  std::queue<mavlink_attitude_t> output_queue_;
  std::condition_variable cond_var_;

};

#endif  // INCLUDE_FLY_STEREO_MAVLINK_READER_H_
