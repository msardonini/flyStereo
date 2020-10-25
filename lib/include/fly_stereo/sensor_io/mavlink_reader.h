#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>

#include "yaml-cpp/yaml.h"
#include "fly_stereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "fly_stereo/vio.h"

#ifndef INCLUDE_FLY_STEREO_MAVLINK_READER_H_
#define INCLUDE_FLY_STEREO_MAVLINK_READER_H_

class MavlinkReader {
 public:
  MavlinkReader();
  ~MavlinkReader();

  int Init(YAML::Node input_params);

  bool GetAttitudeMsg(mavlink_imu_t* attitude, bool block = false);
  bool WaitForStartCmd();
  bool WaitForShutdownCmd();
  void ResetShutdownCmds();
  void SendCounterReset();
  void SendVioMsg(const vio_t &vio);
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
  bool command_shutdown_ = false;
};

#endif  // INCLUDE_FLY_STEREO_MAVLINK_READER_H_
