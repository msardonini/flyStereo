
#include <sys/time.h>
#include <errno.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <experimental/filesystem>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "yaml-cpp/yaml.h"
#include "fly_stereo/sensor_io/camera.h"
#include "fly_stereo/sensor_io/camera_trigger.h"

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
  std::string config_file;
  int image_counter = 0;
  bool user_input = false;

  int opt;
  while((opt = getopt(argc, argv, "c:n:")) != -1) {
    switch(opt) {
      case 'c':
        config_file = std::string(optarg);
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

  YAML::Node params = YAML::LoadFile(config_file)["acquire_calib_images"];
  Camera cam0(params["Camera0"]);
  Camera cam1(params["Camera1"]);
  CameraTrigger camera_trigger(params["CameraTrigger"]);
  if (cam0.Init()) {
    std::cerr << "failed to init camera 0" << std::endl;
    return -1;
  }

  if (cam1.Init()) {
    std::cerr << "failed to init camera 1" << std::endl;
    return -1;
  }


  if (camera_trigger.Init()) {
    std::cerr << "failed to init camera trigger" << std::endl;
    return -1;
  }
  // if (cam0.Init() || cam1.Init() || camera_trigger.Init()) {
  //   std::cerr << "failed to init cameras" << std::endl;
  //   return -1;
  // }

  cv::Mat frame_cam0, frame_cam1;
  std::experimental::filesystem::path save_dir(params["save_dir"].as<std::
    string>());
  std::experimental::filesystem::create_directory(save_dir);
  int counter = 1;
  while (true) {
    if (cam0.GetFrame(frame_cam0) || cam1.GetFrame(frame_cam1)) {
      std::cerr << "Error reading frame!" << std::endl;
      return -1;
    }

    cam0.SendFrame(frame_cam0);
    cam1.SendFrame(frame_cam1);


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
      if (counter % 60 == 0)
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

      cv::imwrite(save_path_0.c_str(), frame_cam0);
      cv::imwrite(save_path_1.c_str(), frame_cam1);
    }
    counter++;
  }

  return 0;  
}

