#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "fly_stereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "fly_stereo/vio.h"
#include "yaml-cpp/yaml.h"

class MavlinkReader {
 public:
  MavlinkReader();
  ~MavlinkReader();

  int Init(YAML::Node input_params);

  bool GetAttitudeMsg(mavlink_imu_t* attitude, bool block = false);
  bool WaitForStartCmd();
  bool CheckForShutdownCmd();
  void ResetShutdownCmds();
  void SendCounterReset();
  void SendVioMsg(const vio_t& vio);

 private:
  int SetSerialParams(int device);

  void SerialReadThread();
  std::atomic<bool> is_running_;
  std::thread reader_thread_;
  int serial_dev_ = 0;

  // Variables for imu message passing
  std::mutex queue_mutex_;
  std::queue<mavlink_imu_t> output_queue_;
  std::condition_variable cond_var_;

  // Variables for start command message passing
  std::mutex cmd_msg_mutex_;
  std::condition_variable cmd_msg_cond_var_;
  bool command_on_ = false;
  std::atomic<bool> command_shutdown_ = false;
};
