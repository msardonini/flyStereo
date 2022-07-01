
#include <getopt.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

#include "flyStereo/image_processing/cv_backend.h"
#include "flyStereo/image_processing/image_processor.h"
#include "flyStereo/pipeline.h"
#include "flyStereo/sensor_io/mavlink_reader.h"
#include "flyStereo/vio.h"
#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"

static Pipeline<CvBackend> *pipeline_global = nullptr;

void SignalHandler(int signal_num) {
  std::cout << "Received control+c, shutting down" << std::endl;

  if (pipeline_global) {
    pipeline_global->shutdown();
  } else {
    std::cerr << "Error! No pipeline to shutdown" << std::endl;
  }
}

// Application Entry Point
int main(int argc, char *argv[]) {
  pid_t pid = getpid();
  spdlog::info("PID of this process: {}", pid);
  // Check if a PID file exists for this program, if so, shut down
  std::string pid_file("/home/msardonini/.pid/flyStereo.pid");
  if (access(pid_file.c_str(), F_OK) != -1) {
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
  while ((opt = getopt(argc, argv, "c:")) != -1) {
    switch (opt) {
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
  YAML::Node fly_stereo_params = YAML::LoadFile(config_file)["flyStereo"];

  // Initialize the pipeline
  // Pipeline<CvBackend> pipeline(fly_stereo_params, fly_stereo_params["stereo_calibration"]);
  Pipeline<CvBackend> pipeline(fly_stereo_params, fly_stereo_params["stereo_calibration"]);
  pipeline.Init();
  pipeline_global = &pipeline;

  // If we have received shutdown commands prior to starting, we do not want to shut down.
  // Reset them
  while (pipeline.is_running()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::cout << "Shutting down main " << std::endl;
  // Remove the PID file
  remove(pid_file.c_str());
  return 0;
}
