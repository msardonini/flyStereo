#include "flyStereo/sensor_io/sql_src.h"

#include <filesystem>

#include "spdlog/spdlog.h"

namespace fs = std::filesystem;

SqlSrc::SqlSrc(fs::path &replay_data_dir) { Init(replay_data_dir); }

SqlSrc::~SqlSrc() {
  if (sql3_.sq_stmt) {
    sqlite3_finalize(sql3_.sq_stmt);
  }
  if (sql3_.data_base_) {
    sqlite3_close(sql3_.data_base_);
  }
}

void SqlSrc::Init(const fs::path &replay_data_dir) {
  auto replay_db_filepath = replay_data_dir / "database.dat";

  // Open the database
  int ret = sqlite3_open(replay_db_filepath.c_str(), &sql3_.data_base_);
  if (ret) {
    spdlog::error("Can't open database: {}", sqlite3_errmsg(sql3_.data_base_));
  } else {
    spdlog::info("Opened database successfully for replay");
  }

  std::string sql_cmd = "SELECT * FROM FLY_STEREO_DATA";
  ret = sqlite3_prepare(sql3_.data_base_, sql_cmd.c_str(), sql_cmd.length(), &sql3_.sq_stmt, nullptr);
  if (ret != SQLITE_OK) {
    spdlog::error("error in prepare, code: {}", ret);
  }
}

int SqlSrc::GetSynchronizedData(cv::Mat_<uint8_t> &frame_cam0, cv::Mat_<uint8_t> &frame_cam1,
                                std::vector<mavlink_imu_t> &imu_msgs, uint64_t &timestamp_frame) {
  int ret = sqlite3_step(sql3_.sq_stmt);
  if (ret != SQLITE_ROW) {
    if (ret == SQLITE_DONE) {
      spdlog::info("Finished Replay");
      return 1;
    } else {
      spdlog::error("Failed Query, val: {}", ret);
      return -1;
    }
  }

  timestamp_frame = sqlite3_column_int64(sql3_.sq_stmt, 0);
  frame_cam0 = cv::Mat(720, 1280, CV_8UC1, const_cast<void *>(sqlite3_column_blob(sql3_.sq_stmt, 2))).clone();
  frame_cam1 = cv::Mat(720, 1280, CV_8UC1, const_cast<void *>(sqlite3_column_blob(sql3_.sq_stmt, 3))).clone();

  size_t imu_msg_count = sqlite3_column_int(sql3_.sq_stmt, 4);
  const uint8_t *imu_blob = reinterpret_cast<const uint8_t *>(sqlite3_column_blob(sql3_.sq_stmt, 5));
  size_t imu_blob_size = sqlite3_column_bytes(sql3_.sq_stmt, 5);

  imu_msgs.clear();
  imu_msgs.reserve(imu_msg_count);
  for (size_t i = 0; i < imu_blob_size; i++) {
    mavlink_status_t mav_status;
    mavlink_message_t mav_message;
    uint8_t msg_received = mavlink_parse_char(MAVLINK_COMM_1, imu_blob[i], &mav_message, &mav_status);

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
