#pragma once

#include <atomic>
#include <filesystem>

#include "flyStereo/sensor_io/sqlite3_params.h"
#include "flyStereo/sensor_io/stereo_sytem_sink_interface.h"

namespace fs = std::filesystem;

template <UMatDerivative ImageT>
class SqlSink : StereoSystemSinkInterface<ImageT> {
 public:
  SqlSink() = default;
  ~SqlSink();

  SqlSink(const fs::path &log_dir);

  void Init(const fs::path &log_dir);

  int ProcessFrame(const LogParams<ImageT> &params) override;

 private:
  int LogEntry(const LogParams<ImageT> &params);

  void LogThread();

  std::atomic<bool> is_running_;
  Sqlite3Params sql3_;

  std::mutex queue_mutex_;
  std::thread queue_thread_;
  std::queue<LogParams<ImageT>> log_queue_;
};

#include "flyStereo/sensor_io/sql_sink.tpp"
