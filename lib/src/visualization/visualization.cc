#include "flyStereo/visualization/visualization.h"

#include <iostream>

#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"

Visualization::Visualization(const YAML::Node &stereo_calibration)
    : my_window_("Coordinate Frame"),
      cube_widget_(cv::Point3f(0.5, 0.5, 0.5), cv::Point3f(0.0, 0.0, 0.0), true, cv::viz::Color::blue()) {
  // viz::Viz3d myWindow("Coordinate Frame");
  my_window_.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  // viz::WLine axis(Point3f(-1.0f,-1.0f,-1.0f), Point3f(1.0f,1.0f,1.0f));
  // axis.setRenderingProperty(viz::LINE_WIDTH, 4.0);
  // myWindow.showWidget("Line Widget", axis);
  // viz::WCube cube_widget(Point3f(0.5,0.5,0.0), Point3f(0.0,0.0,-0.5), true, viz::Color::blue());
  cube_widget_.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
  my_window_.showWidget("Cube Widget", cube_widget_);
  // cv::Mat rot_vec = cv::Mat::zeros(1,3,CV_32F);
  // float translation_phase = 0.0, translation = 0.0;

  std::vector<double> interface_vec = stereo_calibration["R_imu_cam0"].as<std::vector<double>>();
  Eigen::Matrix3d R_imu_cam0_eigen = Eigen::AngleAxisd(interface_vec[0], Eigen::Vector3d::UnitX()) *
                                     Eigen::AngleAxisd(interface_vec[1], Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(interface_vec[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();

  cv::eigen2cv(R_imu_cam0_eigen, R_imu_cam0_);

  // Set the pose of the viz camera
  cv::Vec3f cam_pos(10.0f, -20.0f, 5.0f), cam_focal_point(9.9f, -19.0f, 4.75f), cam_y_dir(0.0f, 0.0f, -1.0f);
  cam_pose_ = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
}
Visualization::~Visualization() {}

int Visualization::ReceiveData(const Eigen::Matrix4d &body_pose, const std::vector<cv::Point3d> &inliers) {
  cv::Matx33d rot_mat;
  cv::Vec3d trans_vec;
  Eigen::Matrix3d body_rot_eigen = body_pose.block<3, 3>(0, 0);
  Eigen::Vector3d body_trans_eigen = body_pose.block<3, 1>(0, 3);

  cv::eigen2cv(body_rot_eigen, rot_mat);
  cv::eigen2cv(body_trans_eigen, trans_vec);

  cv::Affine3d body_pose_cv(rot_mat, trans_vec);
  trajectory_.push_back(body_pose_cv);

  for (size_t i = 0; i < inliers.size(); i++) {
    cv::Point3d point_body_frame = R_imu_cam0_ * inliers[i];
    inliers_global_.push_back(body_pose_cv * point_body_frame);
  }

  my_window_.setWidgetPose("Cube Widget", body_pose_cv.translate(cv::Vec3d(-.25, -.25, -.25)));
  my_window_.showWidget("trajectory", cv::viz::WTrajectory(trajectory_));
  my_window_.showWidget("points", cv::viz::WCloud(inliers_global_, cv::viz::Color::green()));

  cv::Size win_size(1280, 540);
  my_window_.setWindowSize(win_size);
  my_window_.setViewerPose(cam_pose_);

  my_window_.spinOnce(1, true);

  cv::Mat viz_frame = my_window_.getScreenshot();
  if (!writer_) {
    writer_ = std::make_unique<cv::VideoWriter>("viz.mjpg", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 11,
                                                viz_frame.size(), true);
  }

  *writer_ << viz_frame;
  return 0;
}
