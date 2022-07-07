#include "Eigen/Dense"
#include "depthai/device/CalibrationHandler.hpp"
#include "flyStereo/types/umat.h"

template <UMatDerivative ImageT>
OakD<ImageT>::OakD() {}

template <UMatDerivative ImageT>
void OakD<ImageT>::Init() {
  // Define sources and outputs
  mono_left_ = pipeline_.create<dai::node::MonoCamera>();
  mono_right_ = pipeline_.create<dai::node::MonoCamera>();
  xout_left_ = pipeline_.create<dai::node::XLinkOut>();
  xout_right_ = pipeline_.create<dai::node::XLinkOut>();

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
  xout_right_->setStreamName("right");

  // Properties
  mono_left_->setBoardSocket(dai::CameraBoardSocket::LEFT);
  mono_left_->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
  mono_right_->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  mono_right_->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

  // Linking
  mono_right_->out.link(xout_right_->input);
  mono_left_->out.link(xout_left_->input);

  device_ = std::make_unique<dai::Device>(pipeline_);

  // Output queues will be used to get the grayscale frames from the outputs defined above
  queue_left_ = device_->getOutputQueue("left", 1, false);
  queue_right_ = device_->getOutputQueue("right", 1, false);
  queue_imu_ = device_->getOutputQueue("imu", 1, false);

  start_time_ = std::chrono::steady_clock::now();
}

template <UMatDerivative ImageT>
int OakD<ImageT>::GetSynchronizedData(ImageT &d_frame_cam0, ImageT &d_frame_cam1,
                                      std::vector<mavlink_imu_t> &imu_data_a, uint64_t &current_frame_time) {
  auto in_left = queue_left_->get<dai::ImgFrame>();
  auto in_right = queue_right_->get<dai::ImgFrame>();

  d_frame_cam0 = in_left->getCvFrame();
  d_frame_cam1 = in_right->getCvFrame();

  auto left_cam_time = in_left->getTimestamp();

  auto imu_data = queue_imu_->get<dai::IMUData>();

  // std::cout << imu_data->packets.size() << "     " << sizeof(imu_data->packets.front()) << std::endl;

  std::for_each(imu_data->packets.begin(), imu_data->packets.end(),
                [&](auto &val) { local_queue_imu_.push(std::move(val)); });

  imu_data_a.clear();

  while (!local_queue_imu_.empty()) {
    // auto &imu_vals = local_queue_imu_.front().rotationVector;
    auto &gyro_vals = local_queue_imu_.front().gyroscope;
    auto &accel_vals = local_queue_imu_.front().acceleroMeter;
    auto imu_time = gyro_vals.timestamp.get();

    if (imu_time > left_cam_time) {
      break;
    }

    mavlink_imu_t imu_msg;
    imu_msg.time_since_trigger_us = 0;
    imu_msg.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(imu_time - start_time_).count();
    imu_msg.gyroXYZ[0] = gyro_vals.y;
    imu_msg.gyroXYZ[1] = gyro_vals.x;
    imu_msg.gyroXYZ[2] = -gyro_vals.z;
    imu_msg.accelXYZ[0] = accel_vals.y;
    imu_msg.accelXYZ[1] = accel_vals.x;
    imu_msg.accelXYZ[2] = -accel_vals.z;
    imu_data_a.emplace_back(imu_msg);
    local_queue_imu_.pop();

    // Eigen::Quaternionf quat(gyro_vals.real, gyro_vals.i, gyro_vals.j, gyro_vals.k);
    // auto euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    // for (auto i = 0; i < 3; i++) {
    //   imu_msg.gyroXYZ[i] = euler[i];
    // }
  }

  current_frame_time = std::chrono::duration_cast<std::chrono::microseconds>(left_cam_time - start_time_).count();

  // auto handler = dai::CalibrationHandler();
  // auto left_d = handler.getDistortionCoefficients(dai::CameraBoardSocket::LEFT);
  // auto right = handler.getDistortionCoefficients(dai::CameraBoardSocket::RIGHT);

  // std::for_each(left_d.begin(), left_d.end(), [](auto val) {std::cout << ", " << val;});
  // std::cout << std::endl;
  // std::for_each(right.begin(), right.end(), [](auto val) {std::cout << ", " << val;});

  return 0;
}
