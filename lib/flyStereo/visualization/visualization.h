#pragma once

#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/viz.hpp"
#include "yaml-cpp/yaml.h"

class Visualization {
 public:
  Visualization(const cv::Matx33d &R_imu_cam0);
  ~Visualization();

  int ReceiveData(const cv::Affine3d &body_pose, const std::vector<cv::Point3f> &inliers);

 private:
  cv::viz::Viz3d my_window_;
  cv::viz::WCube cube_widget_;
  std::vector<cv::Point3d> inliers_global_;
  std::vector<cv::Affine3d> trajectory_;

  cv::Affine3f cam_pose_;
  cv::Matx33d R_imu_cam0_;

  std::unique_ptr<cv::VideoWriter> writer_;
};
