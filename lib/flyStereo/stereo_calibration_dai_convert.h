#pragma once

#include "depthai/depthai.hpp"
#include "flyStereo/stereo_calibration.h"

/**
 * @brief Template specialization for the StereoCalibrationConvert class, enabling the conversion of StereoCalibration
 * to dai::CalibrationHandler
 *
 */
template <>
struct StereoCalibrationConvert<dai::CalibrationHandler> {
 public:
  /**
   * @brief Convert a StereoCalibration object to a dai::CalibrationHandler
   *
   * @param stereo_cal The StereoCalibration object to convert
   * @return dai::CalibrationHandler
   */
  static dai::CalibrationHandler convert(const StereoCalibration &stereo_cal) {
    dai::CalibrationHandler calibration_handler;
    calibration_handler.setCameraIntrinsics(dai::CameraBoardSocket::LEFT, convert_to_vectors(stereo_cal.K_cam0), 1280,
                                            720);
    std::vector<float> dist_float_cam0(stereo_cal.D_cam0.begin(), stereo_cal.D_cam0.end());
    calibration_handler.setDistortionCoefficients(dai::CameraBoardSocket::LEFT, dist_float_cam0);

    calibration_handler.setCameraIntrinsics(dai::CameraBoardSocket::RIGHT, convert_to_vectors(stereo_cal.K_cam1), 1280,
                                            720);
    std::vector<float> dist_float_cam1(stereo_cal.D_cam1.begin(), stereo_cal.D_cam1.end());
    calibration_handler.setDistortionCoefficients(dai::CameraBoardSocket::RIGHT, dist_float_cam1);

    // std::vector<float> translation(stereo_cal.T_cam0_cam1.begin(), stereo_cal.T_cam0_cam1.end());
    calibration_handler.setCameraExtrinsics(dai::CameraBoardSocket::LEFT, dai::CameraBoardSocket::RIGHT,
                                            convert_to_vectors(stereo_cal.R_cam0_cam1),
                                            convert_to_vector(stereo_cal.T_cam0_cam1));

    calibration_handler.setStereoLeft(dai::CameraBoardSocket::LEFT, convert_to_vectors(stereo_cal.R0));
    calibration_handler.setStereoRight(dai::CameraBoardSocket::RIGHT, convert_to_vectors(stereo_cal.R1));

    return calibration_handler;
  }

 private:
  /**
   * @brief Helper function to convert a cv::Matx-like object to a std::vector<std::vector<float>>
   *
   * @tparam T cv::Matx-like object
   * @param input the input object
   * @return std::vector<std::vector<float>>
   */
  template <typename T>
  static std::vector<std::vector<float>> convert_to_vectors(const T &input) {
    std::vector<std::vector<float>> output;
    for (auto i = 0; i < input.rows; i++) {
      std::vector<float> row;
      for (auto j = 0; j < input.cols; j++) {
        row.push_back(input(i, j));
      }
      output.push_back(row);
    }
    return output;
  };

  /**
   * @brief Helper function to convert a cv::Matx-like object to a std::vector<float>
   *
   * @tparam T cv::Matx-like object
   * @param input the input object
   * @return std::vector<float>
   */
  template <typename T>
  static std::vector<float> convert_to_vector(T &input) {
    std::vector<float> output;
    for (auto i = 0; i < input.rows; i++) {
      output.push_back(input(i));
    }
    return output;
  };
};
