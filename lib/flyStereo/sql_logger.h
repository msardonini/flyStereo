#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "flyStereo/sensor_io/log_params.h"
#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "opencv2/core.hpp"
#include "yaml-cpp/yaml.h"

struct Sqlite3_params;

class SqlLogger {
 public:
  SqlLogger(const YAML::Node &input_params);

  ~SqlLogger();

  /**
   * @brief      Queue up a sample to be logged by a separate thread
   *
   * @param[in]  params  The parameters
   */
  int QueueEntry(const LogParams &params);

  /**
   * @brief      Queries an entry of associated imu and camera data. This is
   *             typically used for replay functionality
   *
   * @param      timestamp_frame      The timestamp of the frames
   * @param      frame0               The frame for camera 0
   * @param      frame1               The frame for camera 1
   * @param      imu_msgs             The imu msgs
   *
   * @return     0 on success, -1 on failure
   */
  int QueryEntry(uint64_t &timestamp_frame, cv::Mat &frame0, cv::Mat &frame1, std::vector<mavlink_imu_t> &imu_msgs);

 private:
  int LogEntry(const LogParams &params);
  int LogEntry(const uint64_t timestamp_frame, const cv::Mat &frame0, const cv::Mat &frame1,
               const std::vector<mavlink_imu_t> &imu_msgs);
  void LogThread();
  std::atomic<bool> is_running_;
  std::unique_ptr<Sqlite3_params> sql3_;
  bool record_mode_;
  bool replay_mode_;

  std::mutex queue_mutex_;
  std::thread queue_thread_;
  std::queue<LogParams> log_queue_;
};
