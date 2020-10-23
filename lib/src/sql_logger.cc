#include "fly_stereo/sql_logger.h"

#include <string>

#include "sqlite3.h"
#include "spdlog/spdlog.h"


constexpr size_t MAX_QUEUE_SIZE = 100;

struct Sqlite3_params {
  sqlite3 *data_base_;
  sqlite3_stmt *sq_stmt;
};

SqlLogger::SqlLogger(const YAML::Node &input_params) {
  sql3_ = std::make_unique<Sqlite3_params>();

  if (input_params["record_mode"].as<bool>() && input_params["record_outputs"][
    "SQL_database"].as<bool>()) {
    record_mode_ = true;
    std::string filename = input_params["run_folder"].as<std::string>();
    filename += "/database.dat";
    // Open the database
    int ret = sqlite3_open(filename.c_str(), &sql3_->data_base_);
    if (ret) {
      spdlog::error("Can't open database: {}", sqlite3_errmsg(sql3_->data_base_));
    } else {
      spdlog::info("Opened database successfully for record");
    }

    std::string sql_cmd = "CREATE TABLE FLY_STEREO_DATA ("
      "`Timestamp_Flight_Controller` BIGINT,"
      "`Timestamp_Perception` BIGINT,"
      "`Image0` BLOB,"
      "`Image1` BLOB,"
      "`Imu_vector_len` INTEGER,"
      "`Imu_messages` BLOB);";
    // output vars
    ret = sqlite3_exec(sql3_->data_base_, sql_cmd.c_str(), nullptr, nullptr, nullptr);
    if (ret != SQLITE_OK) {
      spdlog::error("Error in sqlite3_exec, return code is: {}", ret);
    }

    is_running_.store(true);
    queue_thread_ = std::thread(&SqlLogger::LogThread, this);
  } else if (input_params["replay_mode"].as<bool>()) {
    replay_mode_ = true;
    std::string filename = input_params["replay_dir"].as<std::string>() + "/database.dat";

    // Open the database
    int ret = sqlite3_open(filename.c_str(), &sql3_->data_base_);
    if (ret) {
      spdlog::error("Can't open database: {}", sqlite3_errmsg(sql3_->data_base_));
    } else {
      spdlog::info("Opened database successfully for replay");
    }

    std::string sql_cmd = "SELECT * FROM FLY_STEREO_DATA";
    ret = sqlite3_prepare(sql3_->data_base_, sql_cmd.c_str(), sql_cmd.length(), &sql3_->sq_stmt,
      nullptr);
    if (ret != SQLITE_OK) {
      spdlog::error("error in prepare, code: {}", ret);
    }
  }
}

SqlLogger::~SqlLogger() {
  is_running_.store(false);
  sqlite3_finalize(sql3_->sq_stmt);
  sqlite3_close(sql3_->data_base_);

  if (queue_thread_.joinable()) {
    queue_thread_.join();
  }
}

void SqlLogger::LogThread() {
  while (is_running_.load()) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    if (log_queue_.size() > MAX_QUEUE_SIZE) {
      spdlog::error("Can't keep up writing data, dropping a frame!!");
      log_queue_.pop();
    }
    if (!log_queue_.empty()) {
      LogEntry(log_queue_.front());
      log_queue_.pop();
    } else {
      lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void SqlLogger::QueueEntry(const LogParams &params) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  log_queue_.push(params);
}

int SqlLogger::LogEntry(const LogParams &params) {
  return LogEntry(params.timestamp_flyms, params.timestamp_flystereo, params.frame0, params.frame1,
    params.imu_msgs);
}

int SqlLogger::LogEntry(const uint64_t timestamp_flyms, const uint64_t timestamp_flystereo,
  const cv::Mat &frame0, const cv::Mat &frame1, const std::vector<mavlink_imu_t> &imu_msgs) {
  if (!record_mode_) {
    spdlog::error("LogEntry called when not in record mode");
    return -1;
  }

  std::string sql_cmd = std::string("INSERT INTO FLY_STEREO_DATA VALUES (?,?,?,?,?,?)");

  int ret = sqlite3_prepare(sql3_->data_base_, sql_cmd.c_str(), -1, &sql3_->sq_stmt, nullptr);
  if (ret != SQLITE_OK) {
    spdlog::error("error in prepare, code: {}", ret);
    return -1;
  }

  ret = sqlite3_bind_int64(sql3_->sq_stmt, 1, timestamp_flyms);
  if(ret != SQLITE_OK) {
    spdlog::error("error in bind, code: {}", ret);
    return -1;
  }
  ret = sqlite3_bind_int64(sql3_->sq_stmt, 2, timestamp_flystereo);
  if(ret != SQLITE_OK) {
    spdlog::error("error in bind, code: {}", ret);
    return -1;
  }
  ret = sqlite3_bind_blob(sql3_->sq_stmt, 3, frame0.data, frame0.total() * frame0.elemSize(),
    nullptr);
  if(ret != SQLITE_OK) {
    spdlog::error("error in bind, code: {}", ret);
    return -1;
  }
  ret = sqlite3_bind_blob(sql3_->sq_stmt, 4, frame1.data, frame1.total() * frame1.elemSize(),
    nullptr);
  if(ret != SQLITE_OK) {
    spdlog::error("error in bind, code: {}", ret);
    return -1;
  }

  // Serialize the imu data
  size_t msg_size = 0;
  size_t msg_count = 0;
  uint8_t buf[16384];
  for (mavlink_imu_t imu_msg : imu_msgs) {
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

  ret = sqlite3_bind_int(sql3_->sq_stmt, 5, msg_count);
  if(ret != SQLITE_OK) {
    spdlog::error("error in bind, code: {}", ret);
    return -1;
  }

  ret = sqlite3_bind_blob(sql3_->sq_stmt, 6, buf, msg_size, nullptr);
  if (ret != SQLITE_OK) {
    spdlog::error("error in bind, code: {}", ret);
    return -1;
  }

  ret = sqlite3_step(sql3_->sq_stmt);
  if (ret != SQLITE_DONE) {
    spdlog::error("Error stepping, code: {}", ret);
  }

  sqlite3_reset(sql3_->sq_stmt);
  return 0;
}

int SqlLogger::QueryEntry(uint64_t &timestamp_flyms, uint64_t &timestamp_flystereo,
  cv::Mat &frame0, cv::Mat &frame1, std::vector<mavlink_imu_t> &imu_msgs) {
  if (!replay_mode_) {
    spdlog::error("QueryEntry called when not in replay mode");
  }

  int ret = sqlite3_step(sql3_->sq_stmt);
  if (ret != SQLITE_ROW && ret != SQLITE_DONE) {
    spdlog::error("Failed Query, val: {}", ret);
    return -1;
  }

  timestamp_flyms = sqlite3_column_int64(sql3_->sq_stmt, 0);
  timestamp_flystereo = sqlite3_column_int64(sql3_->sq_stmt, 1);
  frame0 = cv::Mat(720, 1280, CV_8UC1, const_cast<void*>(sqlite3_column_blob(sql3_->sq_stmt, 2))).
    clone();
  frame1 = cv::Mat(720, 1280, CV_8UC1, const_cast<void*>(sqlite3_column_blob(sql3_->sq_stmt, 3))).
    clone();

  size_t imu_msg_count = sqlite3_column_int(sql3_->sq_stmt, 4);
  const uint8_t* imu_blob = reinterpret_cast<const uint8_t*>(sqlite3_column_blob(
    sql3_->sq_stmt, 5));
  size_t imu_blob_size = sqlite3_column_bytes(sql3_->sq_stmt, 5);

  imu_msgs.clear(); imu_msgs.reserve(imu_msg_count);
  for (size_t i = 0; i < imu_blob_size; i++) {
    mavlink_status_t mav_status;
    mavlink_message_t mav_message;
    uint8_t msg_received = mavlink_parse_char(MAVLINK_COMM_1, imu_blob[i], &mav_message,
      &mav_status);

    if (msg_received) {
      if (mav_message.msgid == MAVLINK_MSG_ID_IMU) {
        mavlink_imu_t attitude_msg;
        mavlink_msg_imu_decode(&mav_message, &attitude_msg);
        imu_msgs.push_back(attitude_msg);
      }
    }
  }
  return 0;
}
