
#include <sys/time.h>
#include <errno.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <experimental/filesystem>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/cuda.hpp"
#include "yaml-cpp/yaml.h"
#include "fly_stereo/sensor_io/sensor_interface.h"

char get_user_input(int timeout_s, int timout_us) {
  fd_set fdset;
  struct timeval timeout;
  int  rc;
  int  val;

  timeout.tv_sec = 0;
  timeout.tv_usec = 50000;

  FD_ZERO(&fdset);
  FD_SET(0, &fdset);

  rc = select(1, &fdset, NULL, NULL, &timeout);
  if (rc == -1) {
    std::cerr<< "Error in select " << strerror(errno) << std::endl;
    return 0;
  } else if (rc == 0) {
    return 0;
  } else {
    if (FD_ISSET(0, &fdset)) {
      val = getchar();
      return val;
    }
  }
}

int main(int argc, char *argv[]) {
  // Argument params
  int image_counter = 0;
  bool user_input = false;

  std::string config_file, save_dir;
  int opt;
  while((opt = getopt(argc, argv, "c:n:s:")) != -1) {
    switch(opt) {
      case 'c':
        config_file = std::string(optarg);
        break;
      case 's':
        save_dir = std::string(optarg);
        break;
      case 'n':
        image_counter = std::stoi(optarg);
        break;
      case 'u':
        user_input = true;
      case '?':
        printf("unknown option: %c\n", optopt);
        break;
    }
  }

  if (config_file.empty()) {
    std::cerr << "Need to provide -c config_file" << std::endl;
    return -1;
  } else if (save_dir.empty()) {
    std::cerr << "Need to provide -s save_dir" << std::endl;
    return -1;
  }

  YAML::Node params = YAML::LoadFile(config_file)["fly_stereo"];

  std::unique_ptr<SensorInterface> sensor_interface = std::make_unique<SensorInterface>();
  sensor_interface->Init(params);

  cv::cuda::GpuMat d_frame_cam0, d_frame_cam1;
  std::experimental::filesystem::path save_dir_fp(save_dir);
  std::experimental::filesystem::create_directory(save_dir_fp);
  int counter = 1;
  while (true) {
    std::vector<mavlink_imu_t> imu_data;
    uint64_t current_frame_time;
    int ret = sensor_interface->GetSynchronizedData(d_frame_cam0, d_frame_cam1, imu_data, current_frame_time);
    if (ret < 0) {
      std::cerr << "Error reading frame!" << std::endl;
      return -1;
    }

    sensor_interface->cam0_->SendFrame(d_frame_cam0);
    sensor_interface->cam1_->SendFrame(d_frame_cam1);

    bool save_img = false;
    if (user_input) {
      char ret = get_user_input(0, 50000);
      if (ret == 0x20) {
        save_img = true;
      } else if (ret != 0) {
        std::cout << "ret is: " << ret << std::endl;
      }
    } else {
      // Default behavior is just to save an image every ~5 seconds
      if (counter % 100 == 0)
        save_img = true;
    }

    if (save_img) {
      std::cout << "Saving Image pair: " << ++image_counter << std::endl;
      std::experimental::filesystem::path file0("cam0_" + std::to_string(
        image_counter) + ".png");
      std::experimental::filesystem::path file1("cam1_" + std::to_string(
        image_counter) + ".png");
      std::experimental::filesystem::path save_path_0 = save_dir / file0;
      std::experimental::filesystem::path save_path_1 = save_dir / file1;

      cv::Mat frame0, frame1;
      d_frame_cam0.download(frame0);
      d_frame_cam1.download(frame1);
      cv::imwrite(save_path_0.c_str(), frame0);
      cv::imwrite(save_path_1.c_str(), frame1);
    }
    counter++;
  }

  return 0;
}

