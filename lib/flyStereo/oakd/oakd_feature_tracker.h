#pragma once

#include "depthai/depthai.hpp"
#include "flyStereo/interface.h"

template <typename IpBackend>
class OakDFeatureTracker {
 public:
  OakDFeatureTracker();

  void Init();

  ~OakDFeatureTracker();

  // auto get_tracked_features() -> TrackedImagePoints<IpBackend>;
  auto get_tracked_features() -> void;

 private:
 private:
  dai::Pipeline pipeline_;
  std::unique_ptr<dai::Device> device_;

  std::shared_ptr<dai::node::MonoCamera> mono_left_;
  std::shared_ptr<dai::node::MonoCamera> mono_right_;
  std::shared_ptr<dai::node::FeatureTracker> feature_tracker_left_;
  std::shared_ptr<dai::node::FeatureTracker> feature_tracker_right_;
  std::shared_ptr<dai::node::IMU> imu_;
  std::shared_ptr<dai::node::XLinkOut> xout_left_;
  std::shared_ptr<dai::node::XLinkOut> xout_tracked_features_left_;
  std::shared_ptr<dai::node::XLinkOut> xout_right_;
  std::shared_ptr<dai::node::XLinkOut> xout_tracked_features_right_;
  std::shared_ptr<dai::node::XLinkOut> xout_imu_;
  std::shared_ptr<dai::node::XLinkIn> xin_tracked_features_config_;

  std::shared_ptr<dai::DataOutputQueue> queue_left_;
  std::shared_ptr<dai::DataOutputQueue> queue_left_features_;
  std::shared_ptr<dai::DataOutputQueue> queue_right_;
  std::shared_ptr<dai::DataOutputQueue> queue_right_features_;
  std::shared_ptr<dai::DataOutputQueue> queue_imu_;
  std::shared_ptr<dai::DataInputQueue> queue_tracked_features_config_;
  std::queue<dai::IMUPacket> local_queue_imu_;
};

#include "oakd_feature_tracker.tpp"
