/// \file Unit tests for image processing

#include <cstdlib>
#include <iostream>
#include <vector>

#include "flyStereo/utility.h"
#include "gtest/gtest.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

constexpr uint32_t kSeed = 40;  // Constant seed, lucky number

constexpr float height_off_ground_m = 3.3;
constexpr uint32_t num_pts_x = 100;
constexpr uint32_t num_pts_y = 100;
constexpr uint32_t area_width_m = 10;
constexpr uint32_t area_height_m = 10;
constexpr float ground_plane_z_noise_coeff = 0.01;

constexpr uint32_t num_images = 50;
constexpr float max_camera_rotation_rad = 0.1;

const cv::Size image_size(1000, 1000);

class ImageProcessingTestFixture : public ::testing::Test {
 public:
  ImageProcessingTestFixture() {
    // Generate a set of points to act as the ground plane
    std::srand(kSeed);
    ground_plane_points.reserve(num_pts_x * num_pts_y);
    for (auto i = 0; i < num_pts_x; i++) {
      for (auto j = 0; j < num_pts_y; j++) {
        ground_plane_points.emplace_back(static_cast<float>(i * area_width_m) / num_pts_x,
                                         static_cast<float>(j * area_height_m) / num_pts_y,
                                         height_off_ground_m + (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) *
                                                                   2.f * ground_plane_z_noise_coeff);
      }
    }

    // Generate a trajectory for the stereo camera
    camera_trajectory.reserve(num_images);
    camera_trajectory.push_back(cv::Affine3f::Identity());
    for (auto i = 1; i < num_images; i++) {
      // Generate a random rotation
      const float pitch = (static_cast<float>(std::rand() / RAND_MAX) - 0.5f) * 2.f * max_camera_rotation_rad;
      const float roll = (static_cast<float>(std::rand() / RAND_MAX) - 0.5f) * 2.f * max_camera_rotation_rad;
      const cv::Vec<float, 3> euler_angles(pitch, roll, 0.f);
      const auto rotation = utility::eulerAnglesToRotationMatrix<float>(euler_angles);

      // Move diagonally across the plane
      const auto max_distance = std::sqrt(area_width_m * area_width_m + area_height_m * area_height_m);
      const cv::Vec<float, 3> translation(-i * max_distance / num_images, -i * max_distance / num_images, 0.f);

      // std::cout << translation << std::endl;

      // Combine the rotation and translation into a homogeneous transformation matrix
      cv::Affine3f affine(rotation, translation);

      affine.concatenate(camera_trajectory.back());
      // affine = camera_trajectory.back() * affine;/
      camera_trajectory.emplace_back(affine);
    }

    // Project the points into the image frames and create input images
    points_cam0.reserve(num_images);
    points_cam1.reserve(num_images);
    for (auto i = 0; i < num_images; i++) {
      // Generate the xform to cam0
      auto& cam0_xform = camera_trajectory[i];

      std::vector<cv::Point2f> cam0_points;
      cv::projectPoints(ground_plane_points, cam0_xform.rvec(), cam0_xform.translation(), K_cam0, cv::Mat(),
                        cam0_points);

      if (i == 4) {
        std::cout << " tvecs " << cam0_xform.translation() << std::endl;
        std::cout << "world coordinate " << ground_plane_points.front() << std::endl;
        std::cout << "image point " << cam0_points.front() << std::endl;
      }
      auto cam1_xform = camera_trajectory[i];
      cam1_xform.concatenate(cv::Affine3f(R, T));
      std::vector<cv::Point2f> cam1_points;
      cv::projectPoints(ground_plane_points, cam1_xform.rvec(), cam1_xform.translation(), K_cam1, cv::Mat(),
                        cam1_points);

      auto is_out_of_frame = [&](cv::Point2f pt) {
        return !cv::Rect(0, 0, image_size.width, image_size.height).contains(pt);
      };

      // Remove the points that are out of frame
      std::erase_if(cam0_points, is_out_of_frame);
      std::erase_if(cam1_points, is_out_of_frame);

      points_cam0.emplace_back(cam0_points);
      points_cam1.emplace_back(cam1_points);
    }

    // Render images from the points
    for (auto i = 0; i < num_images; i++) {
      cv::Mat cam0_image = cv::Mat::zeros(image_size, CV_8UC1);
      cv::Mat cam1_image = cv::Mat::zeros(image_size, CV_8UC1);

      for (auto& pt : points_cam0[i]) {
        cv::circle(cam0_image, pt, 3, 255);
      }
      for (auto& pt : points_cam1[i]) {
        cv::circle(cam1_image, pt, 3, 255);
        // cam1_image.at<uint8_t>(pt) = 255;
      }

      images.emplace_back(cam0_image, cam1_image);
    }
  }

  cv::Mat K_cam0 = (cv::Mat_<float>(3, 3) << image_size.width / 2.f, 0.f, image_size.width / 2.f, 0.f,
                    image_size.height / 2.f, image_size.height / 2.f, 0.f, 0.f, 1.f);

  cv::Mat K_cam1 = K_cam0;  // Same camera matrix

  cv::Matx33f R = cv::Matx33f(1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f);
  cv::Vec3f T = cv::Vec3f(.05f, 0.f, 0.f);

  std::vector<cv::Point3f> ground_plane_points;
  std::vector<cv::Affine3f> camera_trajectory;

  std::vector<std::vector<cv::Point2f>> points_cam0;
  std::vector<std::vector<cv::Point2f>> points_cam1;

  std::vector<std::pair<cv::Mat, cv::Mat>> images;
};

TEST_F(ImageProcessingTestFixture, test2) { cv::imwrite("test.jpg", images[20].second); }