#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>

#include "spdlog/spdlog.h"

class TimedLogger {
 public:
  static TimedLogger& get_instance() {
    static TimedLogger logger;
    return logger;
  }

  /// \brief Disables the logger. Makes the program more efficient by no longer calling the clock
  void disable() { enabled_ = false; }

  // /// \brief Sets the logging level of the output statements
  // void set_log_level(spdlog::level level) {
  //   spdlog::set_level(level);
  // }

  void timed_log(int channel, std::string_view msg) {
    auto time_now = std::chrono::system_clock::now();

    std::scoped_lock lock(channel_mutex_);
    if (channels_.count(channel)) {
      auto& time_prev = channels_.find(channel)->second;
      spdlog::info("{} Took: {} Seconds", msg,
                   std::chrono::duration_cast<std::chrono::microseconds>(time_now - time_prev).count());
    } else {
      spdlog::info("{}", msg);
    }

    channels_.insert_or_assign(channel, time_now);
  }

 private:
  TimedLogger() = default;

  std::atomic<bool> enabled_ = true;
  std::mutex channel_mutex_;
  std::unordered_map<int, std::chrono::time_point<std::chrono::system_clock>> channels_;
};
