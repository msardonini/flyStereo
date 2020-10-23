#ifndef LIB_INCLUDE_FLY_STEREO_SQL_LOGGER_H_
#define LIB_INCLUDE_FLY_STEREO_SQL_LOGGER_H_

#include <memory>
#include <vector>
#include <mutex>
#include <queue>
#include <atomic>
#include <thread>

#include "yaml-cpp/yaml.h"
#include "fly_stereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "opencv2/core.hpp"

struct LogParams {
  uint64_t timestamp_flyms;
  uint64_t timestamp_flystereo;
  cv::Mat frame0;
  cv::Mat frame1;
  std::vector<mavlink_imu_t> imu_msgs;
};


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
  void QueueEntry(const LogParams &params);


  int QueryEntry(uint64_t &timestamp_flyms, uint64_t &timestamp_flystereo, cv::Mat &frame0,
    cv::Mat &frame1, std::vector<mavlink_imu_t> &imu_msgs);

 private:
  int LogEntry(const LogParams &params);
  int LogEntry(const uint64_t timestamp_flyms, const uint64_t timestamp_flystereo,
    const cv::Mat &frame0, const cv::Mat &frame1, const std::vector<mavlink_imu_t>  &imu_msgs);
  void LogThread();
  std::atomic<bool> is_running_;
  std::unique_ptr<Sqlite3_params> sql3_;
  bool record_mode_;
  bool replay_mode_;

  std::mutex queue_mutex_;
  std::thread queue_thread_;
  std::queue<LogParams> log_queue_;
};

#endif  // LIB_INCLUDE_FLY_STEREO_SQL_LOGGER_H_
