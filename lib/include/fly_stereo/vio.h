#ifndef LIB_INCLUDE_FLY_STEREO_VIO_H_
#define LIB_INCLUDE_FLY_STEREO_VIO_H_

// Package Includes
#include "opencv2/core.hpp"
#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"
#include "fly_stereo/interface.h"

class Vio {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor with config params
  Vio(const YAML::Node &input_params, const YAML::Node &stereo_calibration);
  // Destructor
  ~Vio();

  int ProcessPoints(const ImagePoints &pts, Eigen::Vector3d &pose);

 private:
  int BinFeatures(const ImagePoints &pts, std::map<int, std::vector<ImagePoint> > &grid);

  int CalculatePoseUpdate(const std::map<int, std::vector<ImagePoint> > &grid,
    Eigen::Matrix4d &pose_update);

  // Object to hold all of the binned features
  unsigned int image_width_;
  unsigned int image_height_;
  unsigned int bins_width_;
  unsigned int bins_height_;
  unsigned int max_pts_in_bin_;

  // All points are placed in bins, sections of the image. The key to this map is the row-major 
  // index of the bin. The value is a vector of image points, which holds points from both cameras
  // in the current frame and the previos one
  std::map<int, std::vector<ImagePoint> > grid;

  // Stereo camera calibration parameters
  cv::Matx33d K_cam0_;
  cv::Matx33d K_cam1_;
  std::vector<double> D_cam0_;
  std::vector<double> D_cam1_;
  cv::Matx33d R_cam0_cam1_;
  cv::Vec3d T_cam0_cam1_;
  cv::Matx34d P0_;
  cv::Matx34d P1_;

  // The estimated pose after VIO is run
  Eigen::MatrixXd pose_;

  // Output file stream for debugging output
  std::unique_ptr<std::ofstream> ofs_;

};


#endif  // LIB_INCLUDE_FLY_STEREO_VIO_H_
