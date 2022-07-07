#include "flyStereo/sensor_io/sql_sink.h"

#include <filesystem>

#include "spdlog/spdlog.h"

constexpr size_t MAX_QUEUE_SIZE = 1000;
constexpr uint32_t MAX_WAIT_TIME_FLUSH_BUFFERS_ON_EXIT_SECONDS = 10;

namespace fs = std::filesystem;

template <UMatDerivative ImageT>
SqlSink<ImageT>::SqlSink(const fs::path &log_dir) {
  Init(log_dir);
}

template <UMatDerivative ImageT>
SqlSink<ImageT>::~SqlSink() {
  // Allow the rest of the queued buffers to get logged
  std::unique_lock<std::mutex> lock(queue_mutex_);

  auto max_wait_time = std::chrono::seconds(MAX_WAIT_TIME_FLUSH_BUFFERS_ON_EXIT_SECONDS);
  auto start = std::chrono::system_clock::now();
  while (!log_queue_.empty()) {
    if (start + max_wait_time < std::chrono::system_clock::now()) {
      spdlog::error("Cannot save the buffer fast enough, exiting");
      break;
    }
    spdlog::info("Saving buffers to disk. {} Left ...", log_queue_.size());
    lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    lock.lock();
  }
  lock.unlock();

  is_running_.store(false);

  is_running_ = false;
  if (queue_thread_.joinable()) {
    queue_thread_.join();
  }
  if (sql3_.sq_stmt) {
    sqlite3_finalize(sql3_.sq_stmt);
  }
  if (sql3_.data_base_) {
    sqlite3_close(sql3_.data_base_);
  }
}

template <UMatDerivative ImageT>
void SqlSink<ImageT>::Init(const fs::path &log_dir) {
  auto filepath = log_dir / "database.dat";

  // Open the database
  int ret = sqlite3_open(filepath.c_str(), &sql3_.data_base_);
  if (ret) {
    spdlog::error("Can't open database: {}", sqlite3_errmsg(sql3_.data_base_));
  } else {
    spdlog::info("Opened database successfully for record");
  }

  std::string sql_cmd =
      "CREATE TABLE FLY_STEREO_DATA ("
      "`Timestamp_Flight_Controller` BIGINT,"
      "`Timestamp_Perception` BIGINT,"
      "`Image0` BLOB,"
      "`Image1` BLOB,"
      "`Imu_vector_len` INTEGER,"
      "`Imu_messages` BLOB);";
  // output vars
  ret = sqlite3_exec(sql3_.data_base_, sql_cmd.c_str(), nullptr, nullptr, nullptr);
  if (ret != SQLITE_OK) {
    spdlog::error("Error in sqlite3_exec, return code is: {}", ret);
  }

  is_running_.store(true);
  queue_thread_ = std::thread(&SqlSink<ImageT>::LogThread, this);
}

template <UMatDerivative ImageT>
void SqlSink<ImageT>::LogThread() {
  while (is_running_.load()) {
    if (queue_mutex_.try_lock()) {
      // We own the mutex, log the next element in the queue
      while (log_queue_.size() > MAX_QUEUE_SIZE) {
        spdlog::error("Can't keep up writing data, dropping a frame!!");
        log_queue_.pop();
      }
      if (!log_queue_.empty()) {
        // "LogEntry" can take some time and it will lock the main processing thread if we own the
        // mutex. Make a local copy and log outside of the mutex
        auto params = std::move(log_queue_.front());
        log_queue_.pop();
        queue_mutex_.unlock();
        LogEntry(params);
      }
      queue_mutex_.unlock();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

template <UMatDerivative ImageT>
int SqlSink<ImageT>::ProcessFrame(const LogParams<ImageT> &params) {
  // Push the data to the logging queue
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (log_queue_.size() >= MAX_QUEUE_SIZE) {
    spdlog::warn("Logging queue full! Cannot queue entry");
    return -1;
  }
  log_queue_.push(params);
  return 0;
}

template <UMatDerivative ImageT>
int SqlSink<ImageT>::LogEntry(const LogParams<ImageT> &params) {
  std::string sql_cmd = std::string("INSERT INTO FLY_STEREO_DATA VALUES (?,?,?,?,?,?)");

  int ret = sqlite3_prepare(sql3_.data_base_, sql_cmd.c_str(), -1, &sql3_.sq_stmt, nullptr);
  if (ret != SQLITE_OK) {
    spdlog::error("error in prepare, code: {}", ret);
    return -1;
  }

  ret = sqlite3_bind_int64(sql3_.sq_stmt, 1, params.timestamp_frame);
  if (ret != SQLITE_OK) {
    spdlog::error("error in bind, code: {}", ret);
    return -1;
  }
  ret = sqlite3_bind_int64(sql3_.sq_stmt, 2, 0);  // This is kept to keep backwards compatibility with previous data
  if (ret != SQLITE_OK) {
    spdlog::error("error in bind, code: {}", ret);
    return -1;
  }

  auto frame0 = params.frame0.frame();
  ret = sqlite3_bind_blob(sql3_.sq_stmt, 3, frame0.data, frame0.total() * frame0.elemSize(), nullptr);
  if (ret != SQLITE_OK) {
    spdlog::error("error in bind, code: {}", ret);
    return -1;
  }

  auto frame1 = params.frame1.frame();
  ret = sqlite3_bind_blob(sql3_.sq_stmt, 4, frame1.data, frame1.total() * frame1.elemSize(), nullptr);
  if (ret != SQLITE_OK) {
    spdlog::error("error in bind, code: {}", ret);
    return -1;
  }

  // Serialize the imu data
  size_t msg_size = 0;
  size_t msg_count = 0;
  uint8_t buf[16384];
  for (mavlink_imu_t imu_msg : params.imu_msgs) {
    mavlink_message_t msg;
    mavlink_msg_imu_encode(1, 200, &msg, &imu_msg);
    uint16_t len = mavlink_msg_to_send_buffer(buf + msg_size, &msg);
    msg_size += len;
    msg_count++;
    // If our buf gets to be 90% filled just break, this will rarely happen
    if (msg_size > 0.9 * sizeof(buf)) {
      break;
    }
  }

  ret = sqlite3_bind_int(sql3_.sq_stmt, 5, msg_count);
  if (ret != SQLITE_OK) {
    spdlog::error("error in bind, code: {}", ret);
    return -1;
  }

  ret = sqlite3_bind_blob(sql3_.sq_stmt, 6, buf, msg_size, nullptr);
  if (ret != SQLITE_OK) {
    spdlog::error("error in bind, code: {}", ret);
    return -1;
  }

  ret = sqlite3_step(sql3_.sq_stmt);
  if (ret != SQLITE_DONE) {
    spdlog::error("Error stepping, code: {}", ret);
  }

  sqlite3_reset(sql3_.sq_stmt);
  return 0;
}

#include "flyStereo/types/umat.h"
template class SqlSink<UMat<uint8_t>>;
#ifdef WITH_VPI
#include "flyStereo/types/umat_vpiimage.h"
template class SqlSink<UMatVpiImage>;
#endif
