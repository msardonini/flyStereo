
#include <getopt.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

#include "yaml-cpp/yaml.h"
#include "fly_stereo/image_processor.h"
#include "fly_stereo/mavlink_reader.h"
#include "fly_stereo/camera_trigger.h"

std::atomic<bool> is_running(true);

void SignalHandler(int signal_num) {
  std::cout << "Received control+c, shutting down" << std::endl;
  is_running.store(false);
}

// Thread to collect the imu data and disperse it to all objects that need it
void imu_thread(YAML::Node imu_reader_params, ImageProcessor *image_processor) {
  MavlinkReader mavlink_reader;
  mavlink_reader.Init(imu_reader_params);

  while(is_running.load()) {
    mavlink_attitude_t attitude;
    // Get the next attitude message, block until we have one
    if(mavlink_reader.GetAttitudeMsg(&attitude, true)) {

      // Send the imu message to the image processor
      image_processor->ReceiveImu(attitude);
    }
  }
}

int main(int argc, char* argv[]) {
  signal(SIGINT, SignalHandler);

  // Argument params
  std::string config_file;

  int opt;
  while((opt = getopt(argc, argv, "c:")) != -1) {
    switch(opt) {
      case 'c':
        config_file = std::string(optarg);
        break;
      case '?':
        printf("unknown option: %c\n", optopt);
        break;
    }
  }

  if (config_file.empty()) {
    std::cerr << "Required argument, -c" << std::endl;
    return -1;
  }

  YAML::Node fly_stereo_params = YAML::LoadFile(config_file)["fly_stereo"];

  ImageProcessor image_processor(fly_stereo_params["image_processor"]);
  image_processor.Init();

  std::thread imu_thread_obj(imu_thread, fly_stereo_params["mavlink_reader"], &image_processor);

  while (is_running.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Clean up the imu thread
  if (imu_thread_obj.joinable()) {
    imu_thread_obj.join();
  }

  std::cout << "Shutting down main " << std::endl;
  return 0;
}
