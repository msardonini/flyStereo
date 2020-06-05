#ifndef INCLUDE_FLY_STEREO_CAMERA_TRIGGER_H_
#define INCLUDE_FLY_STEREO_CAMERA_TRIGGER_H_

#include <thread>
#include <atomic>

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

 private:
  void TriggerThread();

  std::atomic<bool> is_running_;
  std::thread trigger_thread_;

  // File descriptors to interact with kernel
  int chip_fd_;
  int pin_fd_;

  // Pin and gpio chip numbers
  int pin_num_;
  int chip_num_;
  
  bool auto_trigger_async_ = false;
  double auto_trigger_async_rate_hz_ = 0.0;
};


#endif  // INCLUDE_FLY_STEREO_CAMERA_TRIGGER_H_
