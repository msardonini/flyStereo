#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <queue>
#include <thread>

#include "depthai/depthai.hpp"
#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "flyStereo/sensor_io/stereo_system_src_interface.h"
#include "opencv2/core/core.hpp"

class OakD : public StereoSystemSrcInterface {
 public:
  OakD(int fps, bool rectify);

  virtual ~OakD() override = default;

  int GetSynchronizedData(cv::Mat_<uint8_t> &left_image, cv::Mat_<uint8_t> &right_image,
                          std::vector<mavlink_imu_t> &imu_data_a, uint64_t &current_frame_time) override;

 protected:
  void Init(int fps = 30, bool rectify = true);

  std::vector<dai::IMUPacket> GetImuData(std::chrono::milliseconds timeout, bool &has_timed_out);

  std::pair<std::shared_ptr<dai::ImgFrame>, std::shared_ptr<dai::ImgFrame>> GetStereoImagePair(
      std::chrono::milliseconds timeout, bool &has_timed_out);

  // Objects needed to interface with the dai camera
  dai::Pipeline pipeline_;
  std::unique_ptr<dai::Device> device_;
  std::shared_ptr<dai::node::StereoDepth> stereo_;
  std::shared_ptr<dai::DataOutputQueue> queue_left_;
  std::shared_ptr<dai::DataOutputQueue> queue_right_;
  std::shared_ptr<dai::node::XLinkOut> xout_left_;
  std::shared_ptr<dai::node::XLinkOut> xout_right_;
  std::shared_ptr<dai::node::MonoCamera> mono_left_;
  std::shared_ptr<dai::node::MonoCamera> mono_right_;
  std::shared_ptr<dai::node::IMU> imu_;
  std::shared_ptr<dai::node::XLinkOut> xout_imu_;
  std::shared_ptr<dai::DataOutputQueue> queue_imu_;

  std::queue<dai::IMUPacket> local_queue_imu_;  //< queue for imu data, used for synchronization with the camera

  std::chrono::time_point<std::chrono::steady_clock> start_time_;
};
