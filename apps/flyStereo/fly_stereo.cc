
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

#include "flyStereo/image_processing/image_processor.h"
#include "flyStereo/pipeline.h"
#include "flyStereo/sensor_io/mavlink_reader.h"
#include "flyStereo/vio.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"

static Pipeline *pipeline_global = nullptr;

void SignalHandler(int signal_num) {
  std::cout << "Received control+c, shutting down" << std::endl;

  if (pipeline_global) {
    pipeline_global->shutdown();
  } else {
    std::cerr << "Error! No pipeline to shutdown" << std::endl;
  }
}

void UpdateLogDirectory(YAML::Node &node) {
  std::string log_location = node["record_mode"]["log_root_dir"].as<std::string>();
  // First make sure our logging directory exists
  int run_number = 1;
  std::stringstream run_str;
  run_str << std::internal << std::setfill('0') << std::setw(3) << run_number;

  std::string log_dir(log_location + std::string("/run") + run_str.str());

  // Find the next run number folder that isn't in use
  struct stat st = {0};
  while (!stat(log_dir.c_str(), &st)) {
    run_str.str(std::string());
    run_str << std::internal << std::setfill('0') << std::setw(3) << ++run_number;
    log_dir = (log_location + std::string("/run") + run_str.str());
  }
  // Make a new folder to hold the logged data
  mkdir(log_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  // Save the run folder to our YAML params
  node["record_mode"]["log_dir"] = log_dir;
}

void InitializeSpdLog(const std::string &log_dir) {
  int max_bytes = 1048576 * 20;  // Max 20 MB
  int max_files = 20;

  std::vector<spdlog::sink_ptr> sinks;
  // Only use the console sink if we are in debug mode

  sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  sinks.back()->set_level(spdlog::level::info);
  sinks.push_back(
      std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_dir + "/console_log.txt", max_bytes, max_files));
  sinks.back()->set_level(spdlog::level::trace);
  auto flyMS_log = std::make_shared<spdlog::logger>("flyStereo_log", std::begin(sinks), std::end(sinks));

  // Register the logger to the global level
  flyMS_log->set_level(spdlog::level::trace);
  spdlog::register_logger(flyMS_log);
  spdlog::set_default_logger(flyMS_log);
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

  // If we are running in logging mode, change the log directory in the config file
  if (fly_stereo_params["record_mode"] && fly_stereo_params["record_mode"]["enable"].as<bool>()) {
    UpdateLogDirectory(fly_stereo_params);
    InitializeSpdLog(fly_stereo_params["record_mode"]["log_dir"].as<std::string>());
  }

  // Initialize the pipeline
  Pipeline pipeline(fly_stereo_params, fly_stereo_params["stereo_calibration"]);
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
