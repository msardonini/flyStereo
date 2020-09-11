

#include "fly_stereo/sensor_io/sensor_interface.h"

#include <iostream>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

SensorInterface::SensorInterface() {}

SensorInterface::~SensorInterface() {
  if (camera_trigger_) {
    camera_trigger_->TriggerCamera();
  }
}

int SensorInterface::Init(YAML::Node input_params) {
  cam0_ = std::make_unique<Camera> (input_params["Camera0"]);
  if(cam0_->Init()) {
    return -1;
  }
  cam1_ = std::make_unique<Camera> (input_params["Camera1"]);
  if (cam1_->Init()) {
    return -1;
  }
  camera_trigger_ = std::make_unique<CameraTrigger> (input_params["CameraTrigger"]);

  if (camera_trigger_->Init()) {
    return -1;
  }

  min_camera_dt_ms_ = input_params["min_camera_dt_ms"].as<uint64_t>();

  return 0;
}

int SensorInterface::GetSynchronizedData(cv::cuda::GpuMat &d_frame_cam0,
    cv::cuda::GpuMat &d_frame_cam1, std::vector<mavlink_imu_t> &imu_data,
    uint64_t &current_frame_time) {
  // Check we aren't going to trigger the camera too soon. This can cause the program to hang
  std::chrono::steady_clock::time_point time_now = std::chrono::steady_clock::now();
  uint64_t delta_t = std::chrono::duration_cast<std::chrono::milliseconds>(
    time_now - last_trigger_time_).count();
  // sleep for the remaining time if we are here too soon
  if (delta_t < min_camera_dt_ms_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(min_camera_dt_ms_ - delta_t));
  }

  // Trigger the camera and set the time we did this
  camera_trigger_->TriggerCamera();
  last_trigger_time_ = std::chrono::steady_clock::now();

  int ret_frame = cam0_->GetFrame(d_frame_cam0);
  ret_frame |= cam1_->GetFrame(d_frame_cam1);
  if (ret_frame) {
      image_counter_++;
      return -1;
  }

  uint64_t ts_frame0 = cam0_->GetTimestampNs();
  uint64_t ts_frame1 = cam1_->GetTimestampNs();
  double time_diff = static_cast<double>(abs(static_cast<int64_t>(ts_frame1 - ts_frame0))) / 1.0E9;

  // std::cout << "TIME DIFF: " << time_diff << std::endl << std::endl;

  imu_data.clear();
  int ret = AssociateImuData(imu_data, current_frame_time);
  image_counter_++;
  return ret;
}

void SensorInterface::DrawPoints(const std::vector<cv::Point2f> &mypoints,
    cv::Mat &myimage) {
  int myradius=5;
  for (int i = 0; i < mypoints.size(); i++) {
    cv::Scalar color;
    if (i == 0) {
      color = CV_RGB(0, 0, 255);
    } else {
      color = CV_RGB(0, 0, 0);
    }
    circle(myimage, cv::Point(mypoints[i].x, mypoints[i].y), myradius, color, -1, 8, 0);
  }
}


int SensorInterface::ReceiveImu(mavlink_imu_t imu_msg) {
  std::lock_guard<std::mutex> lock(imu_queue_mutex_);
  imu_queue_.push(imu_msg);
}

int SensorInterface::AssociateImuData(std::vector<mavlink_imu_t> &imu_msgs,
  uint64_t &current_frame_time) {
  std::queue<std::pair<uint32_t, uint64_t> > trigger_queue = camera_trigger_->GetTriggerCount();

  // The first image will not have relvant imu data
  if (image_counter_ <= 5) {
    return 1;
  }

  if (imu_queue_.size() == 0) {
    std::cerr << "Imu queue empty!" << std::endl;
    return 1;
  }

  // Reserve some memory in our vector to prevent copies. We should expect this to fill up to
  //  roughly (IMU acquisition rate / image acquisition rate )
  imu_msgs.reserve(20);


  while(trigger_queue.size() > 1) {
    std::cerr << "Error! Mismatch between number of triggers and proccesed frames" << std::endl;
    trigger_queue.pop();
  }

  if (image_counter_ != std::get<0>(trigger_queue.front())) {
    std::cerr << "Error! Mismatch between number of triggers and proccesed frames, image: "
      << image_counter_ << ", from_imu: " << std::get<0>(trigger_queue.front()) << std::endl;
    return -1;
  }

  // The objective is to get all the imu measurements that correspond to this frame,
  // take all the imu measurements that correspond to the current frame minus 1
  int image_of_interest = image_counter_ - 5;
  std::lock_guard<std::mutex> lock(imu_queue_mutex_);

  // Do a sanity check that we didn't miss a bunch of IMU messages, or sometimes a message from a
  // previous recording makes it into the front of our queueu
  if (image_of_interest + 3 < imu_queue_.front().trigger_count) {
    std::cerr << "Error! Missed IMU messages" << std::endl;
    imu_queue_.pop();
  }
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

  // Continue processesing now we have the imu data from the appropriate image frame
  // We need at least 2 messages in order to run the integration
  if (imu_msgs.size() < 2) {
    std::cerr << "Error! Not enough imu messages to run integration" << std::endl;
    return -1;
  }

  // Integrate the roll/pitch/yaw values
  cv::Vec3f delta_rpw = {0.0f, 0.0f, 0.0f};
  if (imu_msgs.size() == 1) {
    uint64_t delta_t = current_frame_time - (imu_msgs[0].timestamp_us - imu_msgs[0].
      time_since_trigger_us);
    delta_rpw += cv::Vec3f(imu_msgs[0].gyroXYZ[0], imu_msgs[0].gyroXYZ[1], imu_msgs[0].gyroXYZ[2])
      * static_cast<float>(delta_t) / 1.0E6f;
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
  cv::Rodrigues(R_imu_cam0 * delta_rpw, rotation_t0_t1_cam0);
  cv::Rodrigues(R_imu_cam1 * delta_rpw, rotation_t0_t1_cam1);
  // std::cout << "delta_rpw: " << delta_rpw << std::endl;
  return 0;
}
