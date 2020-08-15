#ifndef LIB_INCLUDE_FLY_STEREO_SENSOR_IO_SENSOR_INTERFACE_H_
#define LIB_INCLUDE_FLY_STEREO_SENSOR_IO_SENSOR_INTERFACE_H_

// System includes

// Package includes
#include "yaml-cpp/yaml.h"
#include "fly_stereo/sensor_io/camera.h"
#include "fly_stereo/sensor_io/camera_trigger.h"
#include "fly_stereo/sensor_io/mavlink_reader.h"
#include "fly_stereo/interface.h"

class SensorInterface {
 public:
  SensorInterface();
  ~SensorInterface();

  SensorInterface(const SensorInterface&) = delete;
  SensorInterface(SensorInterface&&) = delete;

  void DrawPoints(const std::vector<cv::Point2f> &mypoints, cv::Mat &myimage);

  int GetSynchronizedData(cv::cuda::GpuMat &d_frame_cam0,
    cv::cuda::GpuMat &d_frame_cam1, std::vector<mavlink_imu_t> &imu_data,
    uint64_t &current_frame_time);
  
  int Init(YAML::Node input_params);

  int ReceiveImu(mavlink_imu_t imu_msg);

  int AssociateImuData(std::vector<mavlink_imu_t> &imu_msgs, uint64_t &current_time);

  int GenerateImuXform(const std::vector<mavlink_imu_t> &imu_msgs,
    const cv::Matx33f R_imu_cam0, const cv::Matx33f R_imu_cam1, cv::Matx33f &rotation_t0_t1_cam0,
    const uint64_t current_frame_time, cv::Matx33f &rotation_t0_t1_cam1);

  int image_counter_ = 0;
  std::unique_ptr<CameraTrigger> camera_trigger_;
  std::unique_ptr<Camera> cam0_;
  std::unique_ptr<Camera> cam1_;

 private:
  // IMU objects
  std::queue<mavlink_imu_t> imu_queue_;
  std::mutex imu_queue_mutex_;
  uint64_t last_imu_ts_us_;
};



#endif  // LIB_INCLUDE_FLY_STEREO_SENSOR_IO_SENSOR_INTERFACE_H_