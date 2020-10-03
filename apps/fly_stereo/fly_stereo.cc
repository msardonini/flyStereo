
#include <getopt.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

#include "yaml-cpp/yaml.h"
#include "fly_stereo/vio.h"
#include "fly_stereo/image_processor.h"
#include "fly_stereo/sensor_io/mavlink_reader.h"

std::atomic<bool> is_running(true);

void SignalHandler(int signal_num) {
  std::cout << "Received control+c, shutting down" << std::endl;
  is_running.store(false);
}

// Thread to collect the imu data and disperse it to all objects that need it
void imu_thread(MavlinkReader *mavlink_reader, ImageProcessor *image_processor) {
  while(is_running.load()) {
    mavlink_imu_t attitude;
    // Get the next attitude message, block until we have one
    if(mavlink_reader->GetAttitudeMsg(&attitude, true)) {
      // Send the imu message to the image processor
      image_processor->ReceiveImu(attitude);

      // // Send the imu message to the vio object
      // msckf_vio->imuCallback(attitude);
    }
  }
}

void tracked_features_thread(ImageProcessor *image_processor, Vio *vio, MavlinkReader *mavlink_reader) {
  while(is_running.load()) {
    ImagePoints pts;
    if(image_processor->GetTrackedPoints(&pts)) {
      vio_t vio_data;
      // Send the features to the vio object
      vio->ProcessPoints(pts, vio_data);

      mavlink_reader->SendVioMsg(vio_data);
    }
  }
}

void UpdateLogDirectory(YAML::Node &node) {
  std::string log_location = node["log_dir"].as<std::string>();
  // First make sure our logging directory exists
  int run_number = 1;
  std::stringstream run_str;
  run_str << std::internal << std::setfill('0') << std::setw(3) << run_number;

  std::string run_folder(log_location + std::string("/run") + run_str.str());

  //Find the next run number folder that isn't in use
  struct stat st = {0};
  while (!stat(run_folder.c_str(), &st)) {
    run_str.str(std::string());
    run_str << std::internal << std::setfill('0') << std::setw(3) << ++run_number;
    run_folder = (log_location + std::string("/run") + run_str.str());
  }
  //Make a new folder to hold the logged data
  mkdir(run_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  std::string symbol("<log_dir>");

  std::string imu_filepath = node["mavlink_reader"]["replay_imu_data_file"].as<std::string>();
  size_t pos = imu_filepath.find(symbol);
  imu_filepath.replace(pos, symbol.length(), run_folder);
  node["mavlink_reader"]["replay_imu_data_file"] = imu_filepath;

  std::string cam0_fp = node["image_processor"]["Camera0"]["sink_pipeline"].as<std::string>();
  pos = cam0_fp.find(symbol);
  cam0_fp.replace(pos, symbol.length(), run_folder);
  node["image_processor"]["Camera0"]["sink_pipeline"] = cam0_fp;

  std::string cam1_fp = node["image_processor"]["Camera1"]["sink_pipeline"].as<std::string>();
  pos = cam1_fp.find(symbol);
  cam1_fp.replace(pos, symbol.length(), run_folder);
  node["image_processor"]["Camera1"]["sink_pipeline"] = cam1_fp;
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


  // Parse the yaml file with our configuration parameters
  YAML::Node fly_stereo_params = YAML::LoadFile(config_file)["fly_stereo"];

  // If we are running in logging mode, change the log directory in the config file
  if (fly_stereo_params["record_mode"].as<bool>()) {
    UpdateLogDirectory(fly_stereo_params);
  }

  // Initialze the mavlink reader object
  MavlinkReader mavlink_reader;
  mavlink_reader.Init(fly_stereo_params["mavlink_reader"]);

  if (fly_stereo_params["wait_for_start_command"].as<bool>()) {  
    while(is_running.load()) {
      if(mavlink_reader.WaitForStartCmd() == true) {
        break;
      }
    }
  }

  ImageProcessor image_processor(fly_stereo_params["image_processor"], 
    fly_stereo_params["stereo_calibration"]);
  image_processor.Init();

  std::thread imu_thread_obj(imu_thread, &mavlink_reader,
    &image_processor);


  Vio vio(fly_stereo_params["vio"], fly_stereo_params["stereo_calibration"]);
  std::thread features_thread_obj(tracked_features_thread, &image_processor, &vio,
    &mavlink_reader);

  while (is_running.load()) {
    // If we are receiving start/finish commands then wait for the signal
    if (fly_stereo_params["wait_for_start_command"].as<bool>()) {  
      if(mavlink_reader.WaitForShutdownCmd() == true) {
        is_running.store(false);
      }
    } else {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  // Clean up the threads
  if (imu_thread_obj.joinable()) {
    imu_thread_obj.join();
  }
  // Clean up the threads
  if (features_thread_obj.joinable()) {
    features_thread_obj.join();
  }

  std::cout << "Shutting down main " << std::endl;
  return 0;
}
