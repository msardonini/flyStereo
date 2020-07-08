
#include <getopt.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

#include "yaml-cpp/yaml.h"
#include "fly_stereo/image_processor.h"
#include "fly_stereo/mavlink_reader.h"
#include "fly_stereo/camera_trigger.h"
#include "fly_stereo/msckf_vio/msckf_vio.h"

std::atomic<bool> is_running(true);

void SignalHandler(int signal_num) {
  std::cout << "Received control+c, shutting down" << std::endl;
  is_running.store(false);
}

// Thread to collect the imu data and disperse it to all objects that need it
void imu_thread(YAML::Node imu_reader_params, ImageProcessor *image_processor,
    msckf_vio::MsckfVio *msckf_vio) {
  MavlinkReader mavlink_reader;
  mavlink_reader.Init(imu_reader_params);

  while(is_running.load()) {
    mavlink_imu_t attitude;
    // Get the next attitude message, block until we have one
    if(mavlink_reader.GetAttitudeMsg(&attitude, true)) {
      // Send the imu message to the image processor
      image_processor->ReceiveImu(attitude);

      // Send the imu message to the vio object
      msckf_vio->imuCallback(attitude);
    }
  }
}

void tracked_features_thread(ImageProcessor *image_processor, msckf_vio::MsckfVio *msckf_vio) {
  while(is_running.load()) {
    ImagePoints pts;
    if(image_processor->GetTrackedPoints(&pts)) {
      // Send the vector of feature points to the vio object
      std::cout << "Sending Features to VIO!\n\n";
      // Send the imu message to the vio object
      msckf_vio->featureCallback(pts);
    }
  }
}


int main(int argc, char* argv[]) {
  std::cout << "PID of this process: " << getpid() << std::endl;
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

  msckf_vio::MsckfVio msckf_vio;
  msckf_vio.loadParameters(fly_stereo_params["msckf_vio"]);

  std::thread imu_thread_obj(imu_thread, fly_stereo_params["mavlink_reader"], &image_processor,
    &msckf_vio);
  std::thread features_thread_obj(tracked_features_thread, &image_processor, &msckf_vio);


  while (is_running.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Clean up the threads
  if (imu_thread_obj.joinable()) {
    imu_thread_obj.join();
  }
  if (features_thread_obj.joinable()) {
    features_thread_obj.join();
  }

  std::cout << "Shutting down main " << std::endl;
  return 0;
}
