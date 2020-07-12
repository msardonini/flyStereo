#ifndef INCLUDE_FLY_STEREO_CAMERA_TRIGGER_H_
#define INCLUDE_FLY_STEREO_CAMERA_TRIGGER_H_

#include <thread>
#include <mutex>
#include <atomic>
#include <queue>

#include "yaml-cpp/yaml.h"

class CameraTrigger {
 public:
  explicit CameraTrigger(YAML::Node input_params);
  ~CameraTrigger();

  CameraTrigger() = delete;
  CameraTrigger(const CameraTrigger &cam) = delete;
  CameraTrigger(CameraTrigger &&cam) = delete;

  int Init();
  int TriggerCamera();
  std::queue<std::pair<uint32_t, uint64_t> > GetTriggerCount();

 private:
  void TriggerThread();
  int UpdateCounter();

  std::atomic<bool> is_running_;
  std::thread trigger_thread_;

  // File descriptors to interact with kernel
  int chip_fd_;
  int pin_fd_;

  // Pin and gpio chip numbers
  int pin_num_;
  int chip_num_;
  
  bool auto_trigger_async_ = false;
  bool replay_mode_ = false;
  double auto_trigger_async_rate_hz_ = 0.0;

  uint32_t trigger_count_ = 0;
  std::queue<std::pair<uint32_t, uint64_t> > time_counter_queue_;  
  std::mutex trigger_count_mutex_;
};


#endif  // INCLUDE_FLY_STEREO_CAMERA_TRIGGER_H_
