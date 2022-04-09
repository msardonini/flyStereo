
#include "fly_stereo/sensor_io/camera_trigger.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>

// #define GPIOHANDLE_REQUEST_OUTPUT (1UL << 1)
constexpr unsigned long output_pin = 1UL << 1;
constexpr int num_frames_in_bufer = 2;

CameraTrigger::CameraTrigger(YAML::Node input_params) {
  is_running_.store(false);
  trigger_count_ = -num_frames_in_bufer;
  chip_num_ = input_params["chip_num"].as<int>();
  pin_num_ = input_params["pin_num"].as<int>();
  auto_trigger_async_ = input_params["auto_trigger_async"].as<bool>();
  replay_mode_ = input_params["replay_mode"].as<bool>();
  if (auto_trigger_async_) {
    auto_trigger_async_rate_hz_ = input_params["auto_trigger_async_rate_hz"].as<double>();
  }
}

CameraTrigger::~CameraTrigger() {
  close(chip_fd_);
  close(pin_fd_);

  is_running_.store(false);
  if (trigger_thread_.joinable()) {
    trigger_thread_.join();
  }
}

int CameraTrigger::Init() {
  if (replay_mode_) {
    return 0;
  }

  std::string chip_dev("/dev/gpiochip" + std::to_string(chip_num_));
  chip_fd_ = open(chip_dev.c_str(), O_RDWR);
  if (chip_fd_ <= 0) {
    std::cerr << "Failed to open chip\n";
    return -1;
  }

  // request the gpio pin from the kernel
  struct gpiohandle_request req;
  memset(&req, 0, sizeof(req));
  req.lineoffsets[0] = pin_num_;
  req.lines = 1;
  req.flags = output_pin;
  int ret = ioctl(chip_fd_, GPIO_GET_LINEHANDLE_IOCTL, &req);
  if (ret == -1) {
    perror("ERROR in rc_gpio_init");
    return -1;
  }

  if (req.fd == 0) {
    std::cerr << "ERROR in rc_gpio_init, ioctl gave NULL fd\n";
    return -1;
  }

  pin_fd_ = req.fd;

  if (auto_trigger_async_) {
    trigger_thread_ = std::thread(&CameraTrigger::TriggerThread, this);
  }
  return 0;
}

void CameraTrigger::TriggerThread() {
  is_running_.store(true);
  uint64_t delta_t_us = static_cast<uint64_t>(1.0E6 / auto_trigger_async_rate_hz_);
  while (is_running_.load()) {
    TriggerCamera();
    std::this_thread::sleep_for(std::chrono::microseconds(delta_t_us));
  }
}

std::pair<int, uint64_t> CameraTrigger::GetTriggerCount() {
  std::lock_guard<std::mutex> lock(trigger_count_mutex_);
  return time_counter_;
}

int CameraTrigger::TriggerCamera() {
  // If we are in replay mode, don't mess with the hardware and just update the counters
  if (replay_mode_) {
    uint64_t timestamp_us =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    return UpdateCounter(timestamp_us);
  }

  // The cameras get triggered by receiving a pulse 1us to 1ms width
  // We set the value to high, wait 2us, then set it low again
  struct gpiohandle_data data;
  data.values[0] = 1;
  int ret = ioctl(pin_fd_, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
  if (ret == -1) {
    perror("ERROR in rc_gpio_set_value");
    return -1;
  }

  usleep(2);

  data.values[0] = 0;
  ret = ioctl(pin_fd_, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
  if (ret == -1) {
    perror("ERROR in rc_gpio_set_value");
    return -1;
  }
  uint64_t timestamp_us =
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
          .count();

  return UpdateCounter(timestamp_us);
}

int CameraTrigger::UpdateCounter(uint64_t trigger_time) {
  // Update our trigger counter
  std::lock_guard<std::mutex> lock(trigger_count_mutex_);
  time_counter_ = std::pair<int, uint64_t>(++trigger_count_, trigger_time);
  return 0;
}
