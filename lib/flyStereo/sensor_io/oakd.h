#pragma once
#include <chrono>
#include <memory>
#include <queue>

#include "depthai/depthai.hpp"
#include "flyStereo/sensor_io/stereo_system_src_interface.h"

template <UMatDerivative ImageT>
class OakD : public StereoSystemSrcInterface<ImageT> {
 public:
  OakD();

  int GetSynchronizedData(ImageT &left_image, ImageT &right_image, std::vector<mavlink_imu_t> &imu_data_a,
                          uint64_t &current_frame_time) override;

 private:
  void Init();

  void publish_stereo_image_thread();

  void publish_imu_thread();

  std::vector<dai::IMUPacket> GetImuData(std::chrono::milliseconds timeout, bool &has_timed_out);

  std::pair<std::shared_ptr<dai::ImgFrame>, std::shared_ptr<dai::ImgFrame>> GetStereoImagePair(
      std::chrono::milliseconds timeout, bool &has_timed_out);

  dai::Pipeline pipeline_;
  std::unique_ptr<dai::Device> device_;

  std::shared_ptr<dai::node::StereoDepth> stereo_;
  std::shared_ptr<dai::DataOutputQueue> queue_left_;
  std::shared_ptr<dai::DataOutputQueue> queue_left_rect_;
  std::shared_ptr<dai::DataOutputQueue> queue_right_;
  std::shared_ptr<dai::DataOutputQueue> queue_right_rect_;
  std::shared_ptr<dai::node::XLinkOut> xout_left_;
  std::shared_ptr<dai::node::XLinkOut> xout_left_rect_;
  std::shared_ptr<dai::node::XLinkOut> xout_right_;
  std::shared_ptr<dai::node::XLinkOut> xout_right_rect_;
  std::shared_ptr<dai::node::MonoCamera> mono_left_;
  std::shared_ptr<dai::node::MonoCamera> mono_right_;
  std::shared_ptr<dai::node::IMU> imu_;
  std::shared_ptr<dai::node::XLinkOut> xout_imu_;
  std::shared_ptr<dai::DataOutputQueue> queue_imu_;
  std::queue<dai::IMUPacket> local_queue_imu_;

  std::chrono::time_point<std::chrono::steady_clock> start_time_;
};
