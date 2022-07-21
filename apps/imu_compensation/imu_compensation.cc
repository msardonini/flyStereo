
#include <getopt.h>
#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <iostream>

#include "flyStereo/image_processing/cv_backend.h"
#include "flyStereo/sensor_io/arducam_system.h"
#include "flyStereo/sensor_io/image_sink.h"
#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "flyStereo/sensor_io/oakd.h"
#include "flyStereo/visualization/draw_to_image.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/imgproc.hpp"
#include "yaml-cpp/yaml.h"

std::atomic<bool> is_running(true);

void SignalHandler(int signal_num) {
  std::cout << "Received control+c, shutting down" << std::endl;
  is_running.store(false);
}

int UpdatePointsViaImu(const std::vector<cv::Point3d> &current_pts, const cv::Matx33d &rotation,
                       const cv::Matx33d &camera_matrix, std::vector<cv::Point3d> &updated_pts,
                       bool project_forward = true) {
  if (current_pts.size() == 0) {
    return -1;
  }

  cv::Matx33d H;
  if (project_forward) {
    H = camera_matrix * rotation * camera_matrix.inv();
  } else {
    H = camera_matrix * rotation.t() * camera_matrix.inv();
  }

  updated_pts.resize(current_pts.size());
  for (int i = 0; i < current_pts.size(); i++) {
    updated_pts[i] = H * current_pts[i];
  }
  return 0;
}

// // Thread to collect the imu data and disperse it to all objects that need it
// void imu_thread(YAML::Node imu_reader_params, ArducamSystem<CvBackend::image_type> *arducam_system) {
//   MavlinkReader mavlink_reader;
//   mavlink_reader.Init(imu_reader_params);

//   while (is_running.load()) {
//     mavlink_imu_t attitude;
//     // Get the next attitude message, block until we have one
//     if (mavlink_reader.GetAttitudeMsg(&attitude, true)) {
//       // Send the imu message to the image processor
//       arducam_system->ReceiveImu(attitude);
//     }
//   }
// }

int main(int argc, char *argv[]) {
  std::cout << "PID of this process: " << getpid() << std::endl;
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

  YAML::Node imu_comp_params = YAML::LoadFile(config_file)["flyStereo"];

  // ArducamSystem<CvBackend::image_type> arducam_system(imu_comp_params);
  OakD camera_src(30, false);

  // Image Sinks
  ImageSink cam0_sink(imu_comp_params["ImageSinkCam0"]);
  ImageSink cam1_sink(imu_comp_params["ImageSinkCam1"]);

  // std::thread imu_thread_obj(imu_thread, imu_comp_params["mavlink_reader"], &camera_src);

  // Draw a box of points on the image
  std::vector<cv::Point3d> debug_pts_cam0;

  const float grid_size = 0.33;
  debug_pts_cam0.push_back(cv::Point3d(-grid_size, -grid_size, 1));
  debug_pts_cam0.push_back(cv::Point3d(-grid_size, 0.0, 1));
  debug_pts_cam0.push_back(cv::Point3d(-grid_size, grid_size, 1));
  debug_pts_cam0.push_back(cv::Point3d(0.0, -grid_size, 1));
  debug_pts_cam0.push_back(cv::Point3d(0.0, 0.0, 1));
  debug_pts_cam0.push_back(cv::Point3d(0.0, grid_size, 1));
  debug_pts_cam0.push_back(cv::Point3d(grid_size, -grid_size, 1));
  debug_pts_cam0.push_back(cv::Point3d(grid_size, 0.0, 1));
  debug_pts_cam0.push_back(cv::Point3d(grid_size, grid_size, 1));

  std::vector<cv::Point3d> debug_pts_cam1 = debug_pts_cam0;

  // cv::cuda::GpuMat d_debug_pts; d_debug_pts.upload(debug_pts);
  //   d_debug_pts.download(debug_pts);

  cv::Mat_<uint8_t> d_frame_cam0, d_frame_cam1;
  std::vector<mavlink_imu_t> imu_msgs;

  // Get the config params for the rotation of IMU to cameras
  YAML::Node stereo_calibration = imu_comp_params["stereo_calibration"];
  std::vector<double> interface_vec = stereo_calibration["R"]["data"].as<std::vector<double>>();
  cv::Matx33d R_cam0_cam1 = cv::Matx33d(interface_vec.data());

  interface_vec = stereo_calibration["K0"]["data"].as<std::vector<double>>();
  cv::Matx33d K_cam0 = cv::Matx33d(interface_vec.data());
  interface_vec = stereo_calibration["K1"]["data"].as<std::vector<double>>();
  cv::Matx33d K_cam1 = cv::Matx33d(interface_vec.data());
  std::vector<double> D_cam0 = stereo_calibration["D0"]["data"].as<std::vector<double>>();
  std::vector<double> D_cam1 = stereo_calibration["D1"]["data"].as<std::vector<double>>();

  cv::Matx33d R_imu_cam0;
  std::vector<double> imu_cam0_vec = imu_comp_params["R_imu_cam0"].as<std::vector<double>>();
  cv::Vec3d angles_imu_cam0 = {imu_cam0_vec[0], imu_cam0_vec[1], imu_cam0_vec[2]};
  cv::Rodrigues(angles_imu_cam0, R_imu_cam0);
  cv::Matx33d R_imu_cam1 = R_imu_cam0 * R_cam0_cam1;

  cv::Matx33f R_t0_t1_cam0, R_t0_t1_cam1;
  uint64_t current_frame_time;
  while (is_running.load()) {
    imu_msgs.clear();
    if (camera_src.GetSynchronizedData(d_frame_cam0, d_frame_cam1, imu_msgs, current_frame_time) != 0) {
      continue;
    }
    current_frame_time = 0;
    if (camera_src.GenerateImuXform(imu_msgs, R_imu_cam0, R_imu_cam1, R_t0_t1_cam0, current_frame_time, R_t0_t1_cam1) !=
        0) {
      continue;
    }

    cv::Mat show_frame_cam0, show_frame_cam1;
    UpdatePointsViaImu(debug_pts_cam0, R_t0_t1_cam0, cv::Matx33d::eye(), debug_pts_cam0, false);
    UpdatePointsViaImu(debug_pts_cam1, R_t0_t1_cam1, cv::Matx33d::eye(), debug_pts_cam1, false);

    if (imu_msgs.size() > 0) {
      std::cout << "imu message: " << imu_msgs[0].gyroXYZ[0] << " " << imu_msgs[0].gyroXYZ[1] << " "
                << imu_msgs[0].gyroXYZ[2] << " " << std::endl;
      // std::cout << "imu message size: " << imu_msgs.size() << std::endl;
      // std::cout << "xform " << R_t0_t1_cam0 << std::endl;
      // std::cout << "debug_pts_cam0 " << debug_pts_cam0[0] << std::endl;
    }

    cv::Vec3d tvec(0.0, 0.0, 0.0);
    cv::Vec3d rvec(0.0, 0.0, 0.0);
    if (cam0_sink) {
      std::vector<cv::Point2d> show_pts;
      cv::projectPoints(debug_pts_cam0, rvec, tvec, K_cam0, D_cam0, show_pts);
      std::vector<cv::Point2f> show_pts_f(show_pts.begin(), show_pts.end());
      UMat<cv::Vec3b> show_frame_color;
      UMat<uint8_t> show_frame = d_frame_cam0.clone();
      cv::cvtColor(show_frame.frame(), show_frame_color.frame(), cv::COLOR_GRAY2BGR);
      DrawPoints(show_pts_f, show_frame_color.frame());
      cam0_sink.SendFrame(show_frame_color.frame());
    }

    if (cam1_sink) {
      std::vector<cv::Point2d> show_pts;
      cv::projectPoints(debug_pts_cam1, rvec, tvec, K_cam1, D_cam1, show_pts);
      std::vector<cv::Point2f> show_pts_f(show_pts.begin(), show_pts.end());
      UMat<cv::Vec3b> show_frame_color;
      UMat<uint8_t> show_frame = d_frame_cam1.clone();
      cv::cvtColor(show_frame.frame(), show_frame_color.frame(), cv::COLOR_GRAY2BGR);
      DrawPoints(show_pts_f, show_frame_color.frame());
      cam1_sink.SendFrame(show_frame_color.frame());
    }

    // Sleep to limit the FPS
    // std::this_thread::sleep_for(std::chrono::microseconds(10000));
  }

  // if (imu_thread_obj.joinable()) {
  //   imu_thread_obj.join();
  // }
  return 0;
}
