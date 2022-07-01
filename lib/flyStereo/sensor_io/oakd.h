#pragma once
#include <memory>
#include <queue>

#include "depthai/depthai.hpp"
#include "flyStereo/sensor_io/stereo_system_src_interface.h"

template <UMatDerivative ImageT>
class OakD : public StereoSystemSrcInterface<ImageT> {
 public:
  OakD();

  void Init();

  int GetSynchronizedData(ImageT &d_frame_cam0, ImageT &d_frame_cam1, std::vector<mavlink_imu_t> &imu_data,
                          uint64_t &current_frame_time) override;

 private:
  dai::Pipeline pipeline_;
  std::unique_ptr<dai::Device> device_;

  std::shared_ptr<dai::DataOutputQueue> queue_left_;
  std::shared_ptr<dai::DataOutputQueue> queue_right_;
  std::shared_ptr<dai::node::XLinkOut> xout_left_;
  std::shared_ptr<dai::node::XLinkOut> xout_right_;
  std::shared_ptr<dai::node::MonoCamera> mono_left_;
  std::shared_ptr<dai::node::MonoCamera> mono_right_;
  std::shared_ptr<dai::node::IMU> imu_;
  std::shared_ptr<dai::node::XLinkOut> xout_imu_;
  std::shared_ptr<dai::DataOutputQueue> queue_imu_;
  std::queue<dai::IMUPacket> local_queue_imu_;
};

#include "flyStereo/sensor_io/oakd.tpp"
