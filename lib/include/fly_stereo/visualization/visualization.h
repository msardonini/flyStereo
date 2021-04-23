#ifndef LIB_INCLUDE_FLY_STEREO_VISUALIZATION_VISUALIZATION_H_
#define LIB_INCLUDE_FLY_STEREO_VISUALIZATION_VISUALIZATION_H_

#include <vector>

#include "yaml-cpp/yaml.h"
#include "opencv2/core.hpp"
#include "opencv2/viz.hpp"
#include "opencv2/videoio.hpp"
#include "Eigen/Dense"

class Visualization {
 public:
  Visualization(const YAML::Node &stereo_calibration);
  ~Visualization();

  int ReceiveData(const Eigen::Matrix4d &body_pose, const std::vector<cv::Point3d> &inliers);

 private:
  cv::viz::Viz3d my_window_;
  cv::viz::WCube cube_widget_;
  std::vector<cv::Point3d> inliers_global_;
  std::vector<cv::Affine3d> trajectory_;

  cv::Affine3f cam_pose_;
  cv::Matx33d R_imu_cam0_;

  std::unique_ptr<cv::VideoWriter> writer_;
};

#endif  // LIB_INCLUDE_FLY_STEREO_VISUALIZATION_VISUALIZATION_H_
