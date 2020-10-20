#ifndef LIB_INCLUDE_FLY_STEREO_SQL_LOGGER_H_
#define LIB_INCLUDE_FLY_STEREO_SQL_LOGGER_H_

#include <memory>

#include "yaml-cpp/yaml.h"
#include "fly_stereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "opencv2/core.hpp"

struct Sqlite3_params;

class SqlLogger {
 public:
  SqlLogger(const YAML::Node &input_params);

  ~SqlLogger();

  int LogEntry(uint64_t timestamp_flyms, uint64_t timestamp_flystereo, const cv::Mat &frame0,
    const cv::Mat &frame1, const std::vector<mavlink_imu_t>  &imu_msgs);

  int QueryEntry(uint64_t &timestamp_flyms, uint64_t &timestamp_flystereo, cv::Mat &frame0,
    cv::Mat &frame1, std::vector<mavlink_imu_t> &imu_msgs);

 private:
  std::unique_ptr<Sqlite3_params> sql3_;
  bool record_mode_;
  bool replay_mode_;
};

#endif  // LIB_INCLUDE_FLY_STEREO_SQL_LOGGER_H_
