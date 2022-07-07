#include <iostream>

// Includes common necessary includes for development using depthai library
#include "System.h"
#include "flyStereo/sensor_io/oakd.h"

int main(int argc, char* argv[]) {
  std::string trajectory_output_file("/tmp/trajectory.txt");
  std::string vocab_file("/root/flyStereo/external/ORB_SLAM3/Vocabulary/ORBvoc.txt");
  std::string yaml_config_file("/root/flyStereo/external/ORB_SLAM3/Examples/Stereo/oak.yaml");
  // std::string yaml_config_file("/root/flyStereo/external/ORB_SLAM3/Examples/Stereo-Inertial/RealSense_D435i.yaml");
  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(vocab_file, argv[1], ORB_SLAM3::System::STEREO, true, 0, trajectory_output_file);

  OakD<UMat<uint8_t>> oak;
  oak.Init();

  auto time_start = std::chrono::steady_clock::now();

  while (!SLAM.isShutDown()) {
    // Grab new data from the sensor
    UMat<uint8_t> left, right;
    std::vector<mavlink_imu_t> imu_data;
    uint64_t frame_time;

    int ret = oak.GetSynchronizedData(left, right, imu_data, frame_time);

    std::vector<ORB_SLAM3::IMU::Point> imu_points_orb;

    for (auto& pt : imu_data) {
      imu_points_orb.emplace_back(pt.accelXYZ[0], pt.accelXYZ[1], pt.accelXYZ[2], pt.gyroXYZ[0], pt.gyroXYZ[1],
                                  pt.gyroXYZ[2], pt.timestamp_us / 1.0E6);
    }

    auto pose = SLAM.TrackStereo(left.frame(), right.frame(), frame_time / 1.0E6, imu_points_orb);

    std::cout << pose.matrix3x4() << std::endl;
  }
  return 0;
}
