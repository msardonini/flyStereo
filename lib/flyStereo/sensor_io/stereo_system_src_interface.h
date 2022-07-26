#pragma once

#include <vector>

#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "flyStereo/utility.h"

class StereoSystemSrcInterface {
 public:
  virtual ~StereoSystemSrcInterface() = default;

  virtual int GetSynchronizedData(cv::Mat_<uint8_t> &d_frame_cam0, cv::Mat_<uint8_t> &d_frame_cam1,
                                  std::vector<mavlink_imu_t> &imu_data, uint64_t &current_frame_time) = 0;

  static int GenerateImuXform(const std::vector<mavlink_imu_t> &imu_msgs, const cv::Matx33f R_imu_cam0,
                              const cv::Matx33f R_imu_cam1, cv::Matx33f &rotation_t0_t1_cam0,
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
        delta_t = current_frame_time - (imu_msgs[0].timestamp_us - imu_msgs[0].time_since_trigger_us);
      }
      delta_rpy_imu += cv::Vec3f(imu_msgs[0].gyroXYZ[0], imu_msgs[0].gyroXYZ[1], imu_msgs[0].gyroXYZ[2]) *
                       static_cast<float>(delta_t) / 1.0E6f;
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

        delta_rpy_imu += cv::Vec3f(imu_msgs[i].gyroXYZ[0], imu_msgs[i].gyroXYZ[1], imu_msgs[i].gyroXYZ[2]) *
                         (static_cast<float>(delta_t) / 1.0E6f);
      }
    }

    // std::cout << " delta_rpy_imu: " << delta_rpy_imu << std::endl;
    rotation_t0_t1_cam0 = utility::eulerAnglesToRotationMatrix(R_imu_cam0 * delta_rpy_imu);
    rotation_t0_t1_cam1 = utility::eulerAnglesToRotationMatrix(R_imu_cam1 * delta_rpy_imu);
    // std::cout << "delta_rpy_imu: " << delta_rpy_imu << std::endl;
    return 0;
  }
};
