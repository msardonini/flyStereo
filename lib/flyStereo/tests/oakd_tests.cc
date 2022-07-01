#include "flyStereo/sensor_io/oakd.h"
#include "gtest/gtest.h"
#include "opencv2/highgui.hpp"

TEST(OAKDTEST, HELLOWORLD) {
  OakD<UMat<uint8_t>> oak;
  oak.Init();

  UMat<uint8_t> d_frame_cam0;
  UMat<uint8_t> d_frame_cam1;
  std::vector<mavlink_imu_t> imu_data;
  uint64_t current_frame_time;

  while (true) {
    oak.GetSynchronizedData(d_frame_cam0, d_frame_cam1, imu_data, current_frame_time);

    std::cout << "size " << imu_data.size() << std::endl;
    cv::imshow("left", d_frame_cam0.frame());
    cv::imshow("right", d_frame_cam1.frame());

    int key = cv::waitKey(1);
    if (key == 'q' || key == 'Q') {
      return;
    }
  }
}
