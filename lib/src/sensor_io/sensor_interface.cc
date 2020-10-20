

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
  if (input_params["replay_mode"].as<bool>()) {
    replay_mode_ = true;
  }
  if (input_params["record_mode"].as<bool>()) {
    record_mode_ = true;
  }
  if (record_mode_ || replay_mode_) {
    sql_logger_ = std::make_unique<SqlLogger>(input_params);
  }

  YAML::Node sensor_params = input_params["sensor_interface"];

  cam0_ = std::make_unique<Camera> (sensor_params["Camera0"]);
  if(cam0_->Init()) {
    return -1;
  }
  cam1_ = std::make_unique<Camera> (sensor_params["Camera1"]);
  if (cam1_->Init()) {
    return -1;
  }
  camera_trigger_ = std::make_unique<CameraTrigger> (sensor_params["CameraTrigger"]);

  if (camera_trigger_->Init()) {
    return -1;
  }

  time_sync_frame_ = sensor_params["time_sync_frame"].as<unsigned int>();
  min_camera_dt_ms_ = sensor_params["min_camera_dt_ms"].as<uint64_t>();
  time_assoc_thresh_us_ = sensor_params["time_assoc_thresh_us"].as<int64_t>();

  return 0;
}

int SensorInterface::GetSynchronizedData(cv::cuda::GpuMat &d_frame_cam0,
    cv::cuda::GpuMat &d_frame_cam1, std::vector<mavlink_imu_t> &imu_data,
    uint64_t &current_frame_time) {

  if (replay_mode_ && sql_logger_) {
    cv::Mat frame0, frame1;
    uint64_t timestamp_flyMS, timestamp_flyStereo;
    int ret = sql_logger_->QueryEntry(timestamp_flyMS, timestamp_flyStereo, frame0, frame1,
      imu_data);
    d_frame_cam0.upload(frame0);
    d_frame_cam1.upload(frame1);
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

  imu_data.clear();
  int ret = AssociateImuData(imu_data, current_frame_time);

  if (record_mode_ && sql_logger_) {
    cv::Mat frame0, frame1;
    d_frame_cam0.download(frame0);
    d_frame_cam1.download(frame1);
    spdlog::info("number of imu messages! {}", imu_data.size());
    sql_logger_->LogEntry(2, 3, frame0, frame1, imu_data);
  }

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

  // Save the timestamp of the first image so we can use it to compare deltas in the future
  if (time_first_trigger_flystereo_ == 0 && triggers_.first.first == time_sync_frame_) {
    time_first_trigger_flystereo_ = triggers_.second.second;
    spdlog::info("time flyStereo start {} ", time_first_trigger_flystereo_);
  }
  if (time_first_trigger_flyMS_ == 0 && !imu_queue_.empty() &&
      imu_queue_.front().trigger_count <= time_sync_frame_ - 1) {
    // Pop all the values prior to the one we will use for the timestamp
    while (!imu_queue_.empty() && imu_queue_.front().trigger_count < time_sync_frame_ - 1) {
      imu_queue_.pop();
    }
    if (!imu_queue_.empty() && imu_queue_.front().trigger_count == time_sync_frame_ - 1) {
      time_first_trigger_flyMS_ = imu_queue_.front().timestamp_us - imu_queue_.front().
        time_since_trigger_us;
      spdlog::info("time flyMS start {}", time_first_trigger_flyMS_);
    } else {
      spdlog::info("trigger not at {}", time_sync_frame_ - 1);
    }
  }

  if (time_first_trigger_flyMS_ == 0 || time_first_trigger_flystereo_ == 0) {
    spdlog::info("not initialized, returning");
    return 1;
  }

  if (imu_queue_.size() == 0) {
    std::cerr << "Imu queue empty!" << std::endl;
    return 1;
  }

  // The objective is to get all the imu measurements that correspond to this frame,
  // take all the imu measurements that correspond to the current frame minus 1
  int image_of_interest;
  // Use the timestamps relative to the first image to figure out which trigger matches the
  // correct image
  while (imu_queue_.size() > 0) {
    uint64_t delta_t_flyMS = imu_queue_.front().timestamp_us - imu_queue_.front().
      time_since_trigger_us - time_first_trigger_flyMS_;
    uint64_t delta_t_flystereo = triggers_.second.second - time_first_trigger_flystereo_;

    const int64_t delta_t_dt = static_cast<int64_t>(delta_t_flystereo - delta_t_flyMS);
    // Check to make sure the times are within our threshold
    if (delta_t_dt > time_assoc_thresh_us_) {
      // The current imu messages are behind the frame, delete them
      spdlog::warn("Unsuccessful association, dropping imu messages");
      int current_trig_count = imu_queue_.front().trigger_count;
      while (!imu_queue_.empty() && current_trig_count == imu_queue_.front().trigger_count) {
        imu_queue_.pop();
      }
      if (imu_queue_.empty()) {
        spdlog::warn("imu queue empty, resetting");
        return 1;
      }
    } else if (-delta_t_dt > time_assoc_thresh_us_) {
      // The current frame is a timestep behind the imu data, return
      spdlog::warn("Unsuccessful association, dropping image frame");
      return 1;
    } else {  // Successful association
      image_of_interest = imu_queue_.front().trigger_count;
      break;
    }
  }

  // Reserve some memory in our vector to prevent copies. We should expect this to fill up to
  //  roughly (IMU acquisition rate / image acquisition rate )
  imu_msgs.reserve(20);
  while (!imu_queue_.empty() && imu_queue_.front().trigger_count <= image_of_interest) {
    if (imu_queue_.front().trigger_count == image_of_interest) {
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
  cv::Vec3f delta_rpw = {0.0f, 0.0f, 0.0f};
  if (imu_msgs.size() == 1) {
    uint64_t delta_t = current_frame_time - (imu_msgs[0].timestamp_us - imu_msgs[0].
      time_since_trigger_us);
    delta_rpw += cv::Vec3f(imu_msgs[0].gyroXYZ[0], imu_msgs[0].gyroXYZ[1], imu_msgs[0].gyroXYZ[2])
      * static_cast<float>(delta_t) / 1.0E6f;
    spdlog::info("delta t {}, {}, {}", static_cast<float>(delta_t) / 1.0E6f, current_frame_time,(imu_msgs[0].timestamp_us - imu_msgs[0].time_since_trigger_us));
  } else {
    for (int i = 0; i < imu_msgs.size(); i++) {
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

      delta_rpw += cv::Vec3f(imu_msgs[i].gyroXYZ[0], imu_msgs[i].gyroXYZ[1],
        imu_msgs[i].gyroXYZ[2]) * (static_cast<float>(delta_t) / 1.0E6f);
    }
  }
  rotation_t0_t1_cam0 = utility::eulerAnglesToRotationMatrix<float>(R_imu_cam0 * delta_rpw);
  rotation_t0_t1_cam1 = utility::eulerAnglesToRotationMatrix<float>(R_imu_cam1 * delta_rpw);
  // std::cout << "delta_rpw: " << delta_rpw << std::endl;
  return 0;
}
