#include "flyStereo/sensor_io/oakd.h"

#include "spdlog/spdlog.h"

static std::atomic<bool> lrcheck{true};
static std::atomic<bool> extended{false};
static std::atomic<bool> subpixel{false};
static constexpr auto timeout_period = std::chrono::milliseconds(1000);
static constexpr auto stereo_sync_time_threshold_us =
    25;  //< If left/right images have a timestamp difference of more than this, we will
         // assume that the images are not synchronized.
std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};
static constexpr int image_queue_size = 1;

OakD::OakD(int fps, bool rectify) { Init(fps, rectify); }

void OakD::Init(int fps, bool rectify) {
  // Define sources and outputs
  mono_left_ = pipeline_.create<dai::node::MonoCamera>();
  mono_right_ = pipeline_.create<dai::node::MonoCamera>();
  xout_left_ = pipeline_.create<dai::node::XLinkOut>();
  xout_right_ = pipeline_.create<dai::node::XLinkOut>();

  xout_left_->setStreamName("left");
  xout_right_->setStreamName("right");

  // Define sources and outputs
  imu_ = pipeline_.create<dai::node::IMU>();
  xout_imu_ = pipeline_.create<dai::node::XLinkOut>();
  xout_imu_->setStreamName("imu");

  // Hardware Settings
  mono_left_->setFps(fps);
  mono_right_->setFps(fps);

  imu_->enableIMUSensor(dai::IMUSensor::GYROSCOPE_CALIBRATED, 200);
  imu_->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 200);

  // it's recommended to set both setBatchReportThreshold and setMaxBatchReports
  // to 20 when integrating in a pipeline with a lot of input/output connections
  // above this threshold packets will be sent in batch of X, if the host is not
  // blocked and USB bandwidth is available
  imu_->setBatchReportThreshold(1);
  // maximum number of IMU packets in a batch, if it's reached device will block
  // sending until host can receive it if lower or equal to batchReportThreshold
  // then the sending is always blocking on device useful to reduce device's CPU
  // load  and number of lost packets, if CPU load is high on device side due to
  // multiple nodes
  imu_->setMaxBatchReports(20);

  // Link plugins IMU -> XLINK
  imu_->out.link(xout_imu_->input);

  // Properties
  mono_left_->setBoardSocket(dai::CameraBoardSocket::LEFT);
  mono_left_->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
  mono_right_->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  mono_right_->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

  // StereoDepth
  if (rectify) {
    stereo_ = pipeline_.create<dai::node::StereoDepth>();
    stereo_->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo_->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    // stereo_->setInputResolution(1280, 720);
    stereo_->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
    stereo_->setLeftRightCheck(lrcheck);
    stereo_->setExtendedDisparity(extended);
    stereo_->setSubpixel(subpixel);

    stereo_->rectifiedLeft.link(xout_left_->input);
    stereo_->rectifiedRight.link(xout_right_->input);
    mono_left_->out.link(stereo_->left);
    mono_right_->out.link(stereo_->right);
  } else {
    mono_left_->out.link(xout_left_->input);
    mono_right_->out.link(xout_right_->input);
  }

  device_ = std::make_unique<dai::Device>(pipeline_);

  auto calibrationHandler = device_->readCalibration();

  // Output queues will be used to get the grayscale frames from the outputs
  // defined above
  queue_left_ = device_->getOutputQueue("left", image_queue_size, false);
  queue_right_ = device_->getOutputQueue("right", image_queue_size, false);
  queue_imu_ = device_->getOutputQueue("imu", 30, false);

  spdlog::info("Device USB status: {}", usbStrings[static_cast<int32_t>(device_->getUsbSpeed())]);

  start_time_ = std::chrono::steady_clock::now();
}

std::vector<dai::IMUPacket> OakD::GetImuData(std::chrono::milliseconds timeout, bool &has_timed_out) {
  auto imu_data = queue_imu_->get<dai::IMUData>(timeout, has_timed_out);

  if (has_timed_out) {
    has_timed_out = true;
    return std::vector<dai::IMUPacket>();
  } else {
    return std::move(imu_data->packets);
  }
}

std::pair<std::shared_ptr<dai::ImgFrame>, std::shared_ptr<dai::ImgFrame>> OakD::GetStereoImagePair(
    std::chrono::milliseconds timeout, bool &has_timed_out) {
  auto counter = 0l;
  while (!queue_left_->has<dai::ImgFrame>() || !queue_right_->has<dai::ImgFrame>()) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
    counter++;
    if (counter > std::chrono::duration_cast<std::chrono::microseconds>(timeout_period).count()) {
      spdlog::error("Timeout while waiting for image queues");
      has_timed_out = true;
      return std::make_pair(std::shared_ptr<dai::ImgFrame>(), std::shared_ptr<dai::ImgFrame>());
    }
  }

  auto left_seq_num = queue_left_->front<dai::ImgFrame>()->getSequenceNum();
  auto right_seq_num = queue_right_->front<dai::ImgFrame>()->getSequenceNum();
  if (left_seq_num != right_seq_num) {
    spdlog::warn("Images not synchronized! Left seq: {} Right seq: {}", left_seq_num, right_seq_num);
    if (left_seq_num > right_seq_num) {
      queue_right_->get();
    } else {
      queue_left_->get();
    }
    return {};
  }

  bool has_timed_out_check = false;
  auto in_left = queue_left_->get<dai::ImgFrame>(timeout, has_timed_out_check);
  has_timed_out |= has_timed_out_check;
  auto in_right = queue_right_->get<dai::ImgFrame>(timeout, has_timed_out_check);
  has_timed_out |= has_timed_out_check;

  if (has_timed_out) {
    has_timed_out = true;
    return {};
  } else {
    return std::make_pair(in_left, in_right);
  }
}

int OakD::GetSynchronizedData(cv::Mat_<uint8_t> &left_image, cv::Mat_<uint8_t> &right_image,
                              std::vector<mavlink_imu_t> &imu_data_a, uint64_t &current_frame_time) {
  bool has_timed_out = false;
  bool has_timed_out_check = false;

  auto stereo_pair = this->GetStereoImagePair(timeout_period, has_timed_out_check);
  has_timed_out |= has_timed_out_check;

  while (this->queue_imu_->has<dai::IMUData>() && !has_timed_out) {
    auto imu_data = this->GetImuData(timeout_period, has_timed_out_check);
    std::for_each(imu_data.begin(), imu_data.end(), [&](auto &val) { local_queue_imu_.push(std::move(val)); });
    has_timed_out |= has_timed_out_check;
  }

  if (has_timed_out) {
    spdlog::warn("Error poll timed out!");
    return -1;
  } else if (!stereo_pair.first || !stereo_pair.second) {
    spdlog::warn("Empty frame!");
    return -1;
  }

  auto left_cam_time = stereo_pair.first->getTimestamp();
  left_image = stereo_pair.first->getCvFrame();
  right_image = stereo_pair.second->getCvFrame();

  // Get all the IMU data that is in the time range of the current frame
  imu_data_a.clear();
  while (!local_queue_imu_.empty()) {
    auto imu_time = local_queue_imu_.front().gyroscope.timestamp.get();
    if (imu_time > left_cam_time) {
      break;
    }

    dai::IMUPacket &imu_packet = local_queue_imu_.front();

    mavlink_imu_t imu_msg;
    imu_msg.timestamp_us = imu_packet.gyroscope.timestamp.sec * 1000000 + imu_packet.gyroscope.timestamp.nsec / 1000;
    imu_msg.gyroXYZ[0] = imu_packet.gyroscope.x;
    imu_msg.gyroXYZ[1] = imu_packet.gyroscope.y;
    imu_msg.gyroXYZ[2] = imu_packet.gyroscope.z;
    imu_msg.accelXYZ[0] = imu_packet.acceleroMeter.x;
    imu_msg.accelXYZ[1] = imu_packet.acceleroMeter.y;
    imu_msg.accelXYZ[2] = imu_packet.acceleroMeter.z;

    imu_data_a.push_back(imu_msg);
    local_queue_imu_.pop();
  }

  // current_frame_time = std::chrono::duration_cast<std::chrono::microseconds>(left_cam_time - start_time_).count();
  current_frame_time = 0;

  // auto handler = dai::CalibrationHandler();
  // auto left_d =
  // handler.getDistortionCoefficients(dai::CameraBoardSocket::LEFT); auto right
  // = handler.getDistortionCoefficients(dai::CameraBoardSocket::RIGHT);

  // std::for_each(left_d.begin(), left_d.end(), [](auto val) {std::cout << ", "
  // << val;}); std::cout << std::endl; std::for_each(right.begin(),
  // right.end(), [](auto val) {std::cout << ", " << val;});

  return 0;
}
