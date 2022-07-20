#include <iostream>

// Includes common necessary includes for development using depthai library
#include "System.h"
#include "flyStereo/sensor_io/oakd.h"
#include "flyStereo/sensor_io/sql_src.h"

int main(int argc, char* argv[]) {
  std::string trajectory_output_file("/tmp/trajectory.txt");
  std::string vocab_file("/root/flyStereo/external/ORB_SLAM3/Vocabulary/ORBvoc.txt");
  std::string yaml_config_file("/root/flyStereo/external/ORB_SLAM3/Examples/Stereo/oakd2.yaml");
  // std::string yaml_config_file("/root/flyStereo/external/ORB_SLAM3/Examples/Stereo-Inertial/RealSense_D435i.yaml");
  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(vocab_file, yaml_config_file, ORB_SLAM3::System::IMU_STEREO, true, 0, trajectory_output_file);
  std::cout << " image scale " << SLAM.GetImageScale() << std::endl;

  std::unique_ptr<StereoSystemSrcInterface<UMat<uint8_t>>> src_interface;
  if (argc > 1) {
    std::filesystem::path replay_data_dir(argv[1]);
    src_interface = std::make_unique<SqlSrc<UMat<uint8_t>>>(replay_data_dir);
  } else {
    // Default to Oak
    src_interface = std::make_unique<OakD<UMat<uint8_t>>>();
  }

  auto time_start = std::chrono::steady_clock::now();
  auto time_update = time_start;

  double last_time = 0;

  while (!SLAM.isShutDown()) {
    // Grab new data from the sensor
    UMat<uint8_t> left, right;
    std::vector<mavlink_imu_t> imu_data;
    uint64_t frame_time;

    int ret = src_interface->GetSynchronizedData(left, right, imu_data, frame_time);
    if (ret > 0) {  // We've reached the end of the stream
      break;
    }

    if (imu_data.empty()) {
      continue;
    }

    std::vector<ORB_SLAM3::IMU::Point> imu_points_orb;
    for (auto& pt : imu_data) {
      imu_points_orb.emplace_back(pt.accelXYZ[0], pt.accelXYZ[1], pt.accelXYZ[2], pt.gyroXYZ[0], pt.gyroXYZ[1],
                                  pt.gyroXYZ[2], pt.timestamp_us / 1.0E6);
    }

    // std::cout << "accel " << imu_points_orb.front().a(0) << " " << imu_points_orb.front().a(1) << " "
    //           << imu_points_orb.front().a(2) << std::endl;

    auto pose = SLAM.TrackStereo(left.frame(), right.frame(), frame_time / 1.0E6, imu_points_orb);

    // std::cout << "FPS " << 1.0 / ((frame_time / 1.0E6) - last_time) << std::endl;
    // std::cout << "num imu msgs " << imu_data.size() << std::endl;
    time_update = std::chrono::steady_clock::now();
    last_time = (frame_time / 1.0E6);

    // std::cout << pose.matrix3x4() << std::endl;
  }
  cv::waitKey(0);
  return 0;
}
