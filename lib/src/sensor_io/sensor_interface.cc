

#include "fly_stereo/sensor_io/sensor_interface.h"

#include <iostream>

#include "spdlog/spdlog.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "fly_stereo/utility.h"

SensorInterface::SensorInterface() {}

SensorInterface::~SensorInterface() {
  if (camera_trigger_) {
    camera_trigger_->TriggerCamera();
  }
}

int SensorInterface::Init(YAML::Node input_params) {
  if (input_params["replay_mode"] && input_params["replay_mode"]["enable"].as<bool>()) {
    replay_speed_multiplier_ = input_params["replay_mode"]["replay_speed_multiplier"].as<float>();
    replay_mode_ = true;
  }
  if (input_params["record_mode"] && input_params["record_mode"]["enable"].as<bool>() &&
    input_params["record_mode"]["outputs"]["SQL_database"].as<bool>()) {
    record_mode_ = true;
  }
  if (record_mode_ || replay_mode_) {
    sql_logger_ = std::make_unique<SqlLogger>(input_params);
  }

  YAML::Node sensor_params = input_params["sensor_interface"];

  cam0_ = std::make_unique<Camera> (sensor_params["Camera0"], replay_mode_);
  if(cam0_->Init()) {
    return -1;
  }
  cam1_ = std::make_unique<Camera> (sensor_params["Camera1"], replay_mode_);
  if (cam1_->Init()) {
    return -1;
  }
  camera_trigger_ = std::make_unique<CameraTrigger> (sensor_params["CameraTrigger"]);

  if (camera_trigger_->Init()) {
    return -1;
  }

  min_camera_dt_ms_ = sensor_params["min_camera_dt_ms"].as<uint64_t>();
  return 0;
}

int SensorInterface::GetSynchronizedData(cv::cuda::GpuMat &d_frame_cam0,
    cv::cuda::GpuMat &d_frame_cam1, std::vector<mavlink_imu_t> &imu_data,
    uint64_t &current_frame_time) {

  // Time checks for performance monitoring
  std::chrono::time_point<std::chrono::system_clock> t_start = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> t_trig;
  std::chrono::time_point<std::chrono::system_clock> t_frame;
  std::chrono::time_point<std::chrono::system_clock> t_assoc;
  std::chrono::time_point<std::chrono::system_clock> t_log1;
  std::chrono::time_point<std::chrono::system_clock> t_log2;


  if (replay_mode_ && sql_logger_) {
    cv::Mat frame0, frame1;
    uint64_t timestamp_flyMS, timestamp_flyStereo;
    int ret = sql_logger_->QueryEntry(timestamp_flyMS, timestamp_flyStereo, frame0, frame1,
      imu_data);
    d_frame_cam0.upload(frame0);
    d_frame_cam1.upload(frame1);

    // Add a delay equivalent to what was experienced during the recording
    if (last_replay_time_ != 0) {
      uint64_t delta_t_recording = timestamp_flyMS - last_replay_time_recorded_;
      uint64_t delta_t_processing = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() - last_replay_time_;

      int64_t sleep_time = (delta_t_recording - delta_t_processing) / replay_speed_multiplier_;
      if (sleep_time > 0) {
        // Do not sleep for more than 5 seconds
        if (sleep_time < 5E6) {
          std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
        }
      } else {
        spdlog::warn("Replay did not execute fast enough!");
      }
    }

    last_replay_time_recorded_ = timestamp_flyMS;
    last_replay_time_ = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    return ret;
  }

  // Check we aren't going to trigger the camera too soon. This can cause the program to hang
  uint64_t time_now_us = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  // sleep for the remaining time if we are here too soon
  if (triggers_.first.second - time_now_us < min_camera_dt_ms_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(min_camera_dt_ms_ - (triggers_.first.
      second - time_now_us)));
  }

  // Trigger the camera and set the time we did this
  camera_trigger_->TriggerCamera();

  t_trig = std::chrono::system_clock::now();

  triggers_.second = triggers_.first;
  triggers_.first = camera_trigger_->GetTriggerCount();

  int ret_frame0 = cam0_->GetFrame(d_frame_cam0);
  int ret_frame1 = cam1_->GetFrame(d_frame_cam1);
  if (ret_frame0 < 0 || ret_frame1 < 0) {
    spdlog::error("Error getting frame from camera");
    return -1;
  } else if (ret_frame0 > 0 || ret_frame1 > 0) {
    spdlog::warn("Timeout getting frame");
    return 2;
  }
  t_frame = std::chrono::system_clock::now();

  imu_data.clear();
  int ret = AssociateImuData(imu_data, current_frame_time);

  t_assoc = std::chrono::system_clock::now();
  if (record_mode_ && sql_logger_) {
    cv::Mat frame0, frame1;
    d_frame_cam0.download(frame0);
    d_frame_cam1.download(frame1);
    spdlog::info("number of imu messages! {}", imu_data.size());

    LogParams params;
    if (imu_data.size() > 0) {
      params.timestamp_flyms = imu_data.front().timestamp_us - imu_data.front().
        time_since_trigger_us ;
      } else {
        params.timestamp_flyms = 0;
      }

    params.timestamp_flystereo = triggers_.first.second;
    params.frame0 = frame0;
    params.frame1 = frame1;
    params.imu_msgs = imu_data;

    t_log1 = std::chrono::system_clock::now();
    sql_logger_->QueueEntry(params);
  }
  t_log2 = std::chrono::system_clock::now();

    spdlog::info("SI dts, trig: {}, frame: {}, assoc: {}, log1 {}, log2 {}", (t_trig - t_start).count() / 1E6, (t_frame - t_trig).count() / 1E6, (t_assoc - t_frame).count() / 1E6, (t_log1 - t_assoc).count() / 1E6, (t_log2 - t_log1).count() / 1E6);

  return ret;
}

void SensorInterface::DrawPoints(const std::vector<cv::Point2f> &mypoints,
    cv::Mat &myimage) {
  int myradius = 5;
  for (size_t i = 0; i < mypoints.size(); i++) {
    cv::Scalar color;
    if (i == 0) {
      color = CV_RGB(0, 0, 255);
    } else {
      color = CV_RGB(0, 0, 0);
    }
    circle(myimage, cv::Point(mypoints[i].x, mypoints[i].y), myradius, color, -1, 8, 0);
  }
}


void SensorInterface::ReceiveImu(mavlink_imu_t imu_msg) {
  std::lock_guard<std::mutex> lock(imu_queue_mutex_);
  imu_queue_.push(imu_msg);
}

int SensorInterface::AssociateImuData(std::vector<mavlink_imu_t> &imu_msgs,
  uint64_t &current_frame_time) {
  // Guard against the imu queue
  std::lock_guard<std::mutex> lock(imu_queue_mutex_);

  // All the imu points before the first image will not be useful, delete them
  if (first_iteration_) {
    while (!imu_queue_.empty() && imu_queue_.front().trigger_count != 1) {
      imu_queue_.pop();
    }
    first_iteration_ = false;
  }
  // The first image will not have relvant imu data
  if (triggers_.first.first <= 1) {
    return 1;
  }

  if (imu_queue_.size() == 0) {
    std::cerr << "Imu queue empty!" << std::endl;
    return 1;
  }

  // The objective is to get all the imu measurements that correspond to this frame,
  // The imu messages recorded between the current and previous frame will have the label of the
  // previous image
  int image_of_interest = triggers_.second.first;

  // Reserve some memory in our vector to prevent copies. We should expect this to fill up to
  //  roughly (IMU acquisition rate / image acquisition rate )
  imu_msgs.reserve(20);

  while (!imu_queue_.empty() && static_cast<int>(imu_queue_.front().trigger_count) <=
    image_of_interest) {
    if (static_cast<int>(imu_queue_.front().trigger_count) == image_of_interest) {
      imu_msgs.push_back(imu_queue_.front());
    }
    imu_queue_.pop();
  }

  if (imu_queue_.empty()) {
    current_frame_time = 0;
  } else{
    mavlink_imu_t &msg = imu_queue_.front();
    current_frame_time = msg.timestamp_us - msg.time_since_trigger_us;
  }
  return 0;
}

int SensorInterface::GenerateImuXform(const std::vector<mavlink_imu_t> &imu_msgs,
  const cv::Matx33f R_imu_cam0, const cv::Matx33f R_imu_cam1, cv::Matx33f &rotation_t0_t1_cam0,
  const uint64_t current_frame_time, cv::Matx33f &rotation_t0_t1_cam1) {
  // The first image will not have relvant imu data
  if (imu_msgs.size() == 0) {
    rotation_t0_t1_cam0 = cv::Matx33f::eye();
    rotation_t0_t1_cam1 = cv::Matx33f::eye();
    return 0;
  }

  // Integrate the roll/pitch/yaw values
  cv::Vec3f delta_rpy_imu = {0.0f, 0.0f, 0.0f};
  if (imu_msgs.size() == 1) {
    uint64_t delta_t;
    if (current_frame_time == 0) {
      delta_t = imu_msgs[0].time_since_trigger_us;
    } else {
      delta_t = current_frame_time - (imu_msgs[0].timestamp_us - imu_msgs[0].
        time_since_trigger_us);
    }
    delta_rpy_imu += cv::Vec3f(imu_msgs[0].gyroXYZ[0], imu_msgs[0].gyroXYZ[1],
      imu_msgs[0].gyroXYZ[2]) * static_cast<float>(delta_t) / 1.0E6f;
  } else {
    for (size_t i = 0; i < imu_msgs.size(); i++) {
      uint64_t delta_t;
      // Handle the edge cases where i = 0, or i is max
      if (i == 0) {
        delta_t = imu_msgs[i].time_since_trigger_us;
      } else if (i == imu_msgs.size() - 1) {
        // If we don't have the current time, just use the same dt at the last iteration
        if (current_frame_time != 0) {
          delta_t = current_frame_time - imu_msgs[i - 1].timestamp_us;
        }
      } else {
        delta_t = imu_msgs[i].timestamp_us - imu_msgs[i - 1].timestamp_us;
      }

      delta_rpy_imu += cv::Vec3f(imu_msgs[i].gyroXYZ[0], imu_msgs[i].gyroXYZ[1], imu_msgs[i].
        gyroXYZ[2]) * (static_cast<float>(delta_t) / 1.0E6f);
    }
  }
  rotation_t0_t1_cam0 = utility::eulerAnglesToRotationMatrix<float>(R_imu_cam0 * delta_rpy_imu);
  rotation_t0_t1_cam1 = utility::eulerAnglesToRotationMatrix<float>(R_imu_cam1 * delta_rpy_imu);
  // std::cout << "delta_rpy_imu: " << delta_rpy_imu << std::endl;
  return 0;
}
