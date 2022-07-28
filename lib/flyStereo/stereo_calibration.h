#pragma once

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "yaml-cpp/yaml.h"

struct StereoCalibration {
  /**
   * @brief Construct a Stereo Calibration object
   *
   * @param stereo_calibration YAML node containing the stereo calibration parameters
   */
  StereoCalibration(const YAML::Node &stereo_calibration)
      : StereoCalibration(cv::Matx33d(stereo_calibration["K0"]["data"].as<std::vector<double>>().data()),
                          cv::Matx33d(stereo_calibration["K1"]["data"].as<std::vector<double>>().data()),
                          stereo_calibration["D0"]["data"].as<std::vector<double>>(),
                          stereo_calibration["D1"]["data"].as<std::vector<double>>(),
                          cv::Matx33d(stereo_calibration["R"]["data"].as<std::vector<double>>().data()),
                          cv::Vec3d(stereo_calibration["T"]["data"].as<std::vector<double>>().data()),
                          cv::Size(stereo_calibration["image_size"].as<std::vector<int>>()[0],
                                   stereo_calibration["image_size"].as<std::vector<int>>()[1])) {}

  /**
   * @brief Construct a Stereo Calibration object
   *
   * @param K_cam0  Camera matrix for camera 0
   * @param K_cam1  Camera matrix for camera 1
   * @param D_cam0  Distortion coefficients for camera 0
   * @param D_cam1  Distortion coefficients for camera 1
   * @param R_cam0_cam1 Extrinsic rotation matrix between camera 0 and camera 1
   * @param T_cam0_cam1 Extrinsic translation vector between camera 0 and camera 1
   * @param image_size Size of the image used for calibration
   */
  StereoCalibration(cv::Matx33d K_cam0, cv::Matx33d K_cam1, std::vector<double> D_cam0, std::vector<double> D_cam1,
                    cv::Matx33d R_cam0_cam1, cv::Vec3d T_cam0_cam1, cv::Size image_size)
      : K_cam0(K_cam0),
        K_cam1(K_cam1),
        D_cam0(D_cam0),
        D_cam1(D_cam1),
        R_cam0_cam1(R_cam0_cam1),
        T_cam0_cam1(T_cam0_cam1),
        image_size(image_size) {
    // Calculate the essential matrix and fundamental matrix
    const cv::Matx33d T_cross_mat(0.0, -T_cam0_cam1[2], T_cam0_cam1[1], T_cam0_cam1[2], 0.0, -T_cam0_cam1[0],
                                  -T_cam0_cam1[1], T_cam0_cam1[0], 0.0);

    E = T_cross_mat * R_cam0_cam1;
    F = K_cam0.inv().t() * E * K_cam1.inv();

    cv::stereoRectify(K_cam0, D_cam0, K_cam1, D_cam1, cv::Size(1280, 720), R_cam0_cam1, T_cam0_cam1, R0, R1, P0, P1, Q,
                      cv::CALIB_ZERO_DISPARITY, -1.0, cv::Size(), &valid_roi_left, &valid_roi_right);
  }
  // Stereo Camera Parameters
  cv::Matx33d K_cam0;
  cv::Matx33d K_cam1;
  std::vector<double> D_cam0;
  std::vector<double> D_cam1;
  cv::Matx33d R_cam0_cam1;
  cv::Vec3d T_cam0_cam1;
  cv::Size image_size;
  cv::Matx33d E;
  cv::Matx33d F;

  // Stereo Rectification Parameters
  cv::Matx33d R0;
  cv::Matx33d R1;
  cv::Matx34d P0;
  cv::Matx34d P1;
  cv::Matx44d Q;
  cv::Rect2i valid_roi_left;
  cv::Rect2i valid_roi_right;
};

template <typename Dst>
struct StereoCalibrationConvert;

template <typename Dst>
Dst stereo_calibration_convert(const StereoCalibration &src) {
  return StereoCalibrationConvert<Dst>::convert(src);
}
