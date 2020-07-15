
#include <getopt.h>
#include <unistd.h>
#include <iostream>
#include <atomic>
#include <signal.h>

#include "yaml-cpp/yaml.h"
#include "fly_stereo/sensor_io/sensor_interface.h"
#include "fly_stereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/imgproc.hpp"


std::atomic<bool> is_running(true);

void SignalHandler(int signal_num) {
  std::cout << "Received control+c, shutting down" << std::endl;
  is_running.store(false);
}


int UpdatePointsViaImu(const std::vector<cv::Point2f> &current_pts, const cv::Matx33d &rotation,
  const cv::Matx33d &camera_matrix, std::vector<cv::Point2f> &updated_pts) {
  if (current_pts.size() == 0) {
    return -1;
  }

  cv::Matx33f H = camera_matrix * rotation * camera_matrix.inv();

  updated_pts.resize(current_pts.size());
  for (int i = 0; i < current_pts.size(); i++) {
    cv::Vec3f temp_current(current_pts[i].x, current_pts[i].y, 1.0f);
    cv::Vec3f temp_updated = H * temp_current;
    updated_pts[i].x = temp_updated[0] / temp_updated[2];
    updated_pts[i].y = temp_updated[1] / temp_updated[2];
  }
  return 0;
}


// Thread to collect the imu data and disperse it to all objects that need it
void imu_thread(YAML::Node imu_reader_params, SensorInterface *sensor_interface) {
  MavlinkReader mavlink_reader;
  mavlink_reader.Init(imu_reader_params);

  while(is_running.load()) {
    mavlink_imu_t attitude;
    // Get the next attitude message, block until we have one
    if(mavlink_reader.GetAttitudeMsg(&attitude, true)) {
      // Send the imu message to the image processor
      sensor_interface->ReceiveImu(attitude);

      // // Send the imu message to the vio object
      // msckf_vio->imuCallback(attitude);
    }
  }
}

int main(int argc, char *argv[]) {
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

  YAML::Node imu_comp_params = YAML::LoadFile(config_file)["fly_stereo"];

  SensorInterface sensor_interface;
  sensor_interface.Init(imu_comp_params["image_processor"]);

  std::thread imu_thread_obj(imu_thread, imu_comp_params["mavlink_reader"], &sensor_interface);

  // Draw a box of points on the image
  std::vector<cv::Point2f> debug_pts_cam0;

  debug_pts_cam0.push_back(cv::Point2f(320, 180));
  debug_pts_cam0.push_back(cv::Point2f(640, 180));
  debug_pts_cam0.push_back(cv::Point2f(960, 180));
  debug_pts_cam0.push_back(cv::Point2f(320, 360));
  debug_pts_cam0.push_back(cv::Point2f(640, 360));
  debug_pts_cam0.push_back(cv::Point2f(960, 360));
  debug_pts_cam0.push_back(cv::Point2f(320, 540));
  debug_pts_cam0.push_back(cv::Point2f(640, 540));
  debug_pts_cam0.push_back(cv::Point2f(960, 540));

  std::vector<cv::Point2f> debug_pts_cam1 = debug_pts_cam0;

  // cv::cuda::GpuMat d_debug_pts; d_debug_pts.upload(debug_pts);
  //   d_debug_pts.download(debug_pts);

  cv::cuda::GpuMat d_frame_cam0, d_frame_cam1;
  std::vector<mavlink_imu_t> imu_msgs;

  // Get the config params for the rotation of IMU to cameras
  YAML::Node stereo_calibration = imu_comp_params["image_processor"]["stereo_calibration"];
  std::vector<float> interface_vec = stereo_calibration["R"]["data"].as<std::vector<float>>();
  cv::Matx33f R_cam0_cam1 = cv::Matx33f(interface_vec.data());

  interface_vec = stereo_calibration["K1"]["data"].as<std::vector<float>>();
  cv::Matx33f K_cam0 = cv::Matx33f(interface_vec.data());
  interface_vec = stereo_calibration["K2"]["data"].as<std::vector<float>>();
  cv::Matx33f K_cam1 = cv::Matx33f(interface_vec.data());


  cv::Matx33f R_imu_cam0;
  std::vector<float> imu_cam0_vec = stereo_calibration["debug_vec"].as<std::vector<float>>();
  cv::Vec3f angles_imu_cam0 = {imu_cam0_vec[0], imu_cam0_vec[1], imu_cam0_vec[2]};
  cv::Rodrigues(angles_imu_cam0, R_imu_cam0);
  cv::Matx33f R_imu_cam1 = R_imu_cam0 * R_cam0_cam1;

  cv::Matx33f R_t0_t1_cam0, R_t0_t1_cam1;
  while (is_running.load()) {
    imu_msgs.clear();
    sensor_interface.GetSynchronizedData(d_frame_cam0, d_frame_cam1, imu_msgs);
    sensor_interface.GenerateImuXform(imu_msgs, R_imu_cam0, R_imu_cam1, R_t0_t1_cam0,
      R_t0_t1_cam1);

    if (imu_msgs.size() > 0)
      std::cout << imu_msgs.size() << std::endl;

    cv::Mat show_frame_cam0, show_frame_cam1;

    UpdatePointsViaImu(debug_pts_cam0, R_t0_t1_cam0, K_cam0, debug_pts_cam0);
    UpdatePointsViaImu(debug_pts_cam1, R_t0_t1_cam1, K_cam1, debug_pts_cam1);

    sensor_interface.DrawPoints(debug_pts_cam1, show_frame_cam1);


    if (sensor_interface.cam0_->OutputEnabled()) {
      cv::Mat show_frame, show_frame_color;
      d_frame_cam0.download(show_frame);
      cv::cvtColor(show_frame, show_frame_color, cv::COLOR_GRAY2BGR);
      sensor_interface.DrawPoints(debug_pts_cam0, show_frame_color);
      sensor_interface.cam0_->SendFrame(show_frame_color);
    }

    if (sensor_interface.cam1_->OutputEnabled()) {
      cv::Mat show_frame, show_frame_color;
      d_frame_cam1.download(show_frame);
      cv::cvtColor(show_frame, show_frame_color, cv::COLOR_GRAY2BGR);
      sensor_interface.DrawPoints(debug_pts_cam1, show_frame_color);
      sensor_interface.cam1_->SendFrame(show_frame_color);
    }
  }
  return 0;
}