#include "flyStereo/oakd/oakd_feature_tracker.h"

template <typename IpBackend>
OakDFeatureTracker<IpBackend>::OakDFeatureTracker() {}

template <typename IpBackend>
OakDFeatureTracker<IpBackend>::~OakDFeatureTracker() {}

template <typename IpBackend>
void OakDFeatureTracker<IpBackend>::Init() {
  // Define sources and outputs
  mono_left_ = pipeline_.create<dai::node::MonoCamera>();
  mono_right_ = pipeline_.create<dai::node::MonoCamera>();
  feature_tracker_left_ = pipeline_.create<dai::node::FeatureTracker>();
  feature_tracker_right_ = pipeline_.create<dai::node::FeatureTracker>();
  xout_left_ = pipeline_.create<dai::node::XLinkOut>();
  xout_tracked_features_left_ = pipeline_.create<dai::node::XLinkOut>();
  xout_right_ = pipeline_.create<dai::node::XLinkOut>();
  xout_tracked_features_right_ = pipeline_.create<dai::node::XLinkOut>();
  xin_tracked_features_config_ = pipeline_.create<dai::node::XLinkIn>();

  // Define sources and outputs
  imu_ = pipeline_.create<dai::node::IMU>();
  xout_imu_ = pipeline_.create<dai::node::XLinkOut>();
  xout_imu_->setStreamName("imu");

  // enable ROTATION_VECTOR at 400 hz rate
  imu_->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 200);
  imu_->enableIMUSensor(dai::IMUSensor::GYROSCOPE_CALIBRATED, 200);

  // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline
  // with a lot of input/output connections above this threshold packets will be sent in batch of X, if the host is not
  // blocked and USB bandwidth is available
  imu_->setBatchReportThreshold(20);
  // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
  // if lower or equal to batchReportThreshold then the sending is always blocking on device
  // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple
  // nodes
  imu_->setMaxBatchReports(200);

  // Link plugins IMU -> XLINK
  imu_->out.link(xout_imu_->input);

  xout_left_->setStreamName("left");
  xout_tracked_features_left_->setStreamName("tracked_features_left");
  xout_right_->setStreamName("right");
  xout_tracked_features_right_->setStreamName("tracked_features_right");
  xin_tracked_features_config_->setStreamName("tracked_features_config");

  // Properties
  mono_left_->setBoardSocket(dai::CameraBoardSocket::LEFT);
  mono_left_->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
  mono_right_->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  mono_right_->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

  // Linking
  mono_left_->out.link(feature_tracker_left_->inputImage);
  feature_tracker_left_->passthroughInputImage.link(xout_left_->input);
  feature_tracker_left_->outputFeatures.link(xout_tracked_features_left_->input);
  xin_tracked_features_config_->out.link(feature_tracker_left_->inputConfig);

  mono_right_->out.link(feature_tracker_right_->inputImage);
  feature_tracker_right_->passthroughInputImage.link(xout_right_->input);
  feature_tracker_right_->outputFeatures.link(xout_tracked_features_right_->input);
  xin_tracked_features_config_->out.link(feature_tracker_right_->inputConfig);

  // By default the least mount of resources are allocated
  // increasing it improves performance when optical flow is enabled
  auto numShaves = 2;
  auto numMemorySlices = 2;
  feature_tracker_left_->setHardwareResources(numShaves, numMemorySlices);
  feature_tracker_right_->setHardwareResources(numShaves, numMemorySlices);

  auto featureTrackerConfig = feature_tracker_right_->initialConfig.get();

  device_ = std::make_unique<dai::Device>(pipeline_);

  // Output queues will be used to get the grayscale frames from the outputs defined above
  queue_left_ = device_->getOutputQueue("left", 8, false);
  queue_left_features_ = device_->getOutputQueue("tracked_features_left", 8, false);
  queue_right_ = device_->getOutputQueue("right", 8, false);
  queue_right_features_ = device_->getOutputQueue("tracked_features_right", 8, false);
  queue_imu_ = device_->getOutputQueue("imu", 8, false);
  queue_tracked_features_config_ = device_->getInputQueue("tracked_features_config");
}

template <typename IpBackend>
// TrackedImagePoints<IpBackend> OakDFeatureTracker<IpBackend>::get_tracked_features() {
void OakDFeatureTracker<IpBackend>::get_tracked_features() {
  // auto inPassthroughFrameLeft = passthroughImageLeftQueue->get<dai::ImgFrame>();
  // cv::Mat passthroughFrameLeft = inPassthroughFrameLeft->getFrame();
  // cv::Mat leftFrame;
  // cv::cvtColor(passthroughFrameLeft, leftFrame, cv::COLOR_GRAY2BGR);

  // auto inPassthroughFrameRight = passthroughImageRightQueue->get<dai::ImgFrame>();
  // cv::Mat passthroughFrameRight = inPassthroughFrameRight->getFrame();
  // cv::Mat rightFrame;
  // cv::cvtColor(passthroughFrameRight, rightFrame, cv::COLOR_GRAY2BGR);

  auto trackedFeaturesLeft = queue_left_features_->get<dai::TrackedFeatures>()->trackedFeatures;
  auto trackedFeaturesRight = queue_right_features_->get<dai::TrackedFeatures>()->trackedFeatures;

  std::cout << trackedFeaturesLeft.size() << " " << trackedFeaturesRight.size() << std::endl;

  // std::cout << "left features: " << trackedFeaturesLeft.size() << std::endl;
  // for (auto& feature : trackedFeaturesLeft) {
  //   std::cout << feature.position.x << " " << feature.position.y << " " << feature.id << " " << feature.trackingError
  //             << std::endl;
  // }

  // std::cout << "right features: " << trackedFeaturesRight.size() << std::endl;
  // for (auto& feature : trackedFeaturesRight) {
  //   std::cout << feature.position.x << " " << feature.position.y << " " << feature.id << " " << feature.trackingError
  //             << std::endl;
  // }

  using IdT = decltype(dai::TrackedFeature::id);
  std::map<IdT, dai::Point2f> left_ids;
  std::for_each(trackedFeaturesLeft.begin(), trackedFeaturesLeft.end(),
                [&left_ids](const dai::TrackedFeature& feature) {
                  left_ids.insert(std::make_pair(feature.id, feature.position));
                });

  std::map<IdT, dai::Point2f> right_ids;
  std::for_each(trackedFeaturesRight.begin(), trackedFeaturesRight.end(),
                [&right_ids](const dai::TrackedFeature& feature) {
                  right_ids.insert(std::make_pair(feature.id, feature.position));
                });

  std::set<IdT> ids_intersect;
  for (auto& id : left_ids) {
    if (right_ids.count(id.first)) {
      ids_intersect.insert(id.first);
    }
  }

  std::cout << "intersection: " << ids_intersect.size() << std::endl;

  typename IpBackend::array_type left_features(cv::Size(1, ids_intersect.size()));
  typename IpBackend::array_type right_features(cv::Size(1, ids_intersect.size()));

  {
    auto counter = 0;
    for (auto& id : ids_intersect) {
      left_features.frame()(counter)(0) = left_ids[id].x;
      left_features.frame()(counter)(1) = left_ids[id].y;

      right_features.frame()(counter)(0) = right_ids[id].x;
      right_features.frame()(counter)(1) = right_ids[id].y;
      counter++;
    }
  }
}
