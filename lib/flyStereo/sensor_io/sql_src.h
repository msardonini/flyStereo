#pragma once

#include <filesystem>

#include "flyStereo/sensor_io/sqlite3_params.h"
#include "flyStereo/sensor_io/stereo_system_src_interface.h"
#include "flyStereo/types/umat.h"

namespace fs = std::filesystem;

template <UMatDerivative ImageT>
class SqlSrc : public StereoSystemSrcInterface<ImageT> {
 public:
  SqlSrc() = default;
  ~SqlSrc();
  SqlSrc(fs::path &replay_data_dir);

  void Init(const fs::path &replay_data_dir);

  int GetSynchronizedData(ImageT &d_frame_cam0, ImageT &d_frame_cam1, std::vector<mavlink_imu_t> &imu_data,
                          uint64_t &current_frame_time) override;

 private:
  Sqlite3Params sql3_;
};

#include "flyStereo/sensor_io/sql_src.tpp"
