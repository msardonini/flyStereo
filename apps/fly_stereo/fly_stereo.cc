
#include <getopt.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <fstream>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"
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

void tracked_features_thread(ImageProcessor *image_processor, Vio *vio, MavlinkReader
  *mavlink_reader) {

  std::chrono::time_point<std::chrono::system_clock> t_start = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> t_ip = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> t_vio = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> t_mav = std::chrono::system_clock::now();

  while(is_running.load()) {
    t_start = std::chrono::system_clock::now();
    ImagePoints pts;
    if(image_processor->GetTrackedPoints(&pts)) {
      t_ip = std::chrono::system_clock::now();
      vio_t vio_data;
      // Send the features to the vio object
      vio->ProcessPoints(pts, vio_data);
      t_vio = std::chrono::system_clock::now();

      mavlink_reader->SendVioMsg(vio_data);
      t_mav = std::chrono::system_clock::now();


    spdlog::trace("dts ms: ip: {}, vio: {}, mav: {} ", (t_ip - t_start).count() / 1E6,
      (t_vio - t_ip).count() / 1E6, (t_mav - t_vio).count() / 1E6);

    }
  }
}

void UpdateLogDirectory(YAML::Node &node) {
  std::string log_location = node["record_mode"]["log_root_dir"].as<std::string>();
  // First make sure our logging directory exists
  int run_number = 1;
  std::stringstream run_str;
  run_str << std::internal << std::setfill('0') << std::setw(3) << run_number;

  std::string log_dir(log_location + std::string("/run") + run_str.str());

  //Find the next run number folder that isn't in use
  struct stat st = {0};
  while (!stat(log_dir.c_str(), &st)) {
    run_str.str(std::string());
    run_str << std::internal << std::setfill('0') << std::setw(3) << ++run_number;
    log_dir = (log_location + std::string("/run") + run_str.str());
  }
  //Make a new folder to hold the logged data
  mkdir(log_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  // Save the run folder to our YAML params
  node["record_mode"]["log_dir"] = log_dir;
}

int InitializeSpdLog(const std::string &log_dir) {
  int max_bytes = 1048576 * 20;  // Max 20 MB
  int max_files = 20;

  std::vector<spdlog::sink_ptr> sinks;
  // Only use the console sink if we are in debug mode

  sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  sinks.back()->set_level(spdlog::level::info);
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt> (
    log_dir + "/console_log.txt", max_bytes, max_files));
  sinks.back()->set_level(spdlog::level::trace);
  auto flyMS_log = std::make_shared<spdlog::logger>("flyStereo_log", std::begin(sinks),
    std::end(sinks));

  // Register the logger to the global level
  flyMS_log->set_level(spdlog::level::trace);
  spdlog::register_logger(flyMS_log);
  spdlog::set_default_logger(flyMS_log);
}

// Application Entry Point
int main(int argc, char* argv[]) {
  pid_t pid = getpid();
  spdlog::info("PID of this process: {}", pid);
  // Check if a PID file exists for this program, if so, shut down
  std::string pid_file("/home/msardonini/.pid/flyStereo.pid");
  if (access(pid_file.c_str(), F_OK ) != -1) {
    spdlog::error("PID file found, program already running. Exiting now");
    return -1;
  } else {
    std::ofstream pid_file_stream(pid_file, std::ofstream::out);
    pid_file_stream << pid;
  }

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

  // Initialze the mavlink reader object
  MavlinkReader mavlink_reader;
  mavlink_reader.Init(fly_stereo_params);

  if (fly_stereo_params["wait_for_start_command"].as<bool>()) {
    while(is_running.load()) {
      if(mavlink_reader.WaitForStartCmd() == true) {
        break;
      }
    }
  }
  // If we are running in logging mode, change the log directory in the config file
  if (fly_stereo_params["record_mode"] && fly_stereo_params["record_mode"]["enable"].as<bool>()) {
    UpdateLogDirectory(fly_stereo_params);
    InitializeSpdLog(fly_stereo_params["record_mode"]["log_dir"].as<std::string>());
  }

  ImageProcessor image_processor(fly_stereo_params, fly_stereo_params["stereo_calibration"]);
  image_processor.Init();

  std::thread imu_thread_obj(imu_thread, &mavlink_reader, &image_processor);

  Vio vio(fly_stereo_params, fly_stereo_params["stereo_calibration"]);
  std::thread features_thread_obj(tracked_features_thread, &image_processor, &vio,
    &mavlink_reader);

  // If we have received shutdown commands prior to starting, we do not want to shut down.
  // Reset them
  mavlink_reader.ResetShutdownCmds();
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
  // Remove the PID file
  remove(pid_file.c_str());
  return 0;
}
