/// \file Unit tests for image processing

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <vector>

#include "flyStereo/image_processing/cv_backend.h"
#include "flyStereo/image_processing/cv_cpu_backend.h"
#include "flyStereo/image_processing/image_processor.h"

#ifdef WITH_VPI
#include "flyStereo/image_processing/vpi_backend.h"
#endif
#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "flyStereo/utility.h"
#include "gtest/gtest.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/viz.hpp"

constexpr uint32_t kSeed = 40;  // Constant seed, lucky number

constexpr int32_t grid_size_ground_plane_widgets = 10;
constexpr float height_off_ground_m = 1;
constexpr uint32_t num_pts_x = 33;
constexpr uint32_t num_pts_y = 33;
constexpr uint32_t area_width_m = 4;
constexpr uint32_t area_height_m = 4;
constexpr float ground_plane_noise_coeff = 0.01;

constexpr uint32_t num_images = 100;
constexpr float max_camera_rotation_rad = 0.0;

const uint32_t circle_radius = 2;
const cv::Size image_size(1280, 780);

// TODO: add min/max values for the noise
static inline float gen_noise(const float min_val, const float max_val) {
  return min_val + (static_cast<float>(std::rand()) / RAND_MAX) * (max_val - min_val);
}

static inline mavlink_imu_t gen_empty_msg() {
  mavlink_imu_t msg;
  msg.accelXYZ[0] = 0;
  msg.accelXYZ[1] = 0;
  msg.accelXYZ[2] = 0;
  msg.gyroXYZ[0] = 0;
  msg.gyroXYZ[1] = 0;
  msg.gyroXYZ[2] = 0;
  msg.timestamp_us = 0;
  msg.time_since_trigger_us = 0;
  msg.pitch = 0;
  msg.roll = 0;
  msg.yaw = 0;
  msg.trigger_count = 0;
  return msg;
}

static inline std::vector<cv::Point3f> get_ground_points_vec(const std::vector<std::pair<cv::Point3f, uint8_t>>& vals) {
  std::vector<cv::Point3f> pts;
  pts.reserve(vals.size());
  for (const auto& val : vals) {
    pts.push_back(val.first);
  }
  return pts;
}

template <typename IpBackend>
class ImageProcessingTestFixture : public ::testing::Test {
 public:
  ImageProcessingTestFixture() : my_window_("Test Window") {
    cv::viz::Camera cam0(K_cam0, image_size);
    my_window_.spinOnce(1, false);
    my_window_.setCamera(cam0);

    {
      const cv::Size size_in_viz(2, 2);
      auto ground_plane_image = cv::imread("/root/flyStereo/test.png");
      for (auto row = -grid_size_ground_plane_widgets / 2; row < grid_size_ground_plane_widgets / 2; ++row) {
        for (auto col = -grid_size_ground_plane_widgets / 2; col < grid_size_ground_plane_widgets / 2; ++col) {
          ground_plane_widgets_.emplace_back(ground_plane_image.clone(), size_in_viz);
          my_window_.showWidget(std::string("Ground Plane") + std::to_string(ground_plane_widgets_.size()),
                                ground_plane_widgets_.back());
          my_window_.setWidgetPose(
              std::string("Ground Plane") + std::to_string(ground_plane_widgets_.size()),
              cv::Affine3f(cv::Vec3f(0, 0, 0),
                           cv::Vec3f(col * size_in_viz.width, row * size_in_viz.height, height_off_ground_m)));
        }
      }
    }

    // Generate a set of points to act as the ground plane
    std::srand(kSeed);
    ground_plane_points.reserve(num_pts_x * num_pts_y);
    ground_plane_point_intensities.reserve(num_pts_x * num_pts_y);
    for (auto i = 0; i < num_pts_x; i++) {
      for (auto j = 0; j < num_pts_y; j++) {
        ground_plane_points.emplace_back(
            gen_noise(-ground_plane_noise_coeff, ground_plane_noise_coeff) +
                static_cast<float>(i * area_width_m) / num_pts_x,
            gen_noise(-ground_plane_noise_coeff, ground_plane_noise_coeff) +
                static_cast<float>(j * area_height_m) / num_pts_y,
            gen_noise(-ground_plane_noise_coeff, ground_plane_noise_coeff) + height_off_ground_m);
        ground_plane_point_intensities.emplace_back(gen_noise(50.f, 255.f));
      }
    }

    // Generate a trajectory for the stereo camera
    camera_trajectory.reserve(num_images);
    camera_trajectory.emplace_back(cv::Affine3f::Identity());  // First timestep is identity
    imu_msgs.reserve(num_images);
    mavlink_imu_t first_imu_msg = gen_empty_msg();
    first_imu_msg.timestamp_us = 1E6f;
    first_imu_msg.time_since_trigger_us = 0;
    imu_msgs.emplace_back(first_imu_msg);

    for (auto i = 1; i < num_images; i++) {
      // Generate a random rotation
      const float pitch = (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * 2.f * max_camera_rotation_rad;
      const float roll = (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * 2.f * max_camera_rotation_rad;
      const cv::Vec<float, 3> euler_angles(pitch, roll, 0.f);
      const auto rotation = utility::eulerAnglesToRotationMatrix(euler_angles);

      const cv::Vec<float, 3> translation(static_cast<double>(i * area_width_m) / num_images,
                                          static_cast<double>(i * area_height_m) / num_images, 0.f);

      // Combine the rotation and translation into a homogeneous transformation matrix
      cv::Affine3f affine(rotation, translation);

      camera_trajectory.emplace_back(affine);

      // Generate an IMU message
      mavlink_imu_t imu_msg = gen_empty_msg();
      imu_msg.timestamp_us = 1E6f * (i - 1);
      imu_msg.time_since_trigger_us = 0;
      imu_msg.gyroXYZ[0] = roll - imu_msgs.back().roll;
      imu_msg.gyroXYZ[1] = pitch - imu_msgs.back().pitch;
      imu_msg.gyroXYZ[2] = 0.f;
      imu_msgs.emplace_back(imu_msg);
    }

    // Project the points into the image frames and create input images
    points_cam0.reserve(num_images);
    points_cam1.reserve(num_images);
    for (auto i = 0; i < num_images; i++) {
      // Generate the xform to cam0
      auto cam0_xform_inv = camera_trajectory[i];
      std::vector<cv::Point2f> cam0_image_point_coords;
      cv::projectPoints(ground_plane_points, cam0_xform_inv.rvec(), cam0_xform_inv.translation(), K_cam0, cv::noArray(),
                        cam0_image_point_coords);

      auto cam1_xform = cam0_xform_inv.concatenate(cv::Affine3f(R, T).inv());

      std::vector<cv::Point2f> cam1_image_point_coords;
      cv::projectPoints(ground_plane_points, cam1_xform.rvec(), cam1_xform.translation(), K_cam1, cv::noArray(),
                        cam1_image_point_coords);

      // Zip up the points and intensities
      auto lambda_zip = [](cv::Point2f pt, uint8_t intensity) { return std::make_pair(pt, intensity); };

      std::vector<std::pair<cv::Point2f, uint8_t>> cam0_points_and_intensities;
      std::transform(cam0_image_point_coords.begin(), cam0_image_point_coords.end(),
                     ground_plane_point_intensities.begin(), std::back_inserter(cam0_points_and_intensities),
                     lambda_zip);
      std::vector<std::pair<cv::Point2f, uint8_t>> cam1_points_and_intensities;
      std::transform(cam1_image_point_coords.begin(), cam1_image_point_coords.end(),
                     ground_plane_point_intensities.begin(), std::back_inserter(cam1_points_and_intensities),
                     lambda_zip);

      // Remove the points that are out of frame
      auto is_out_of_frame = [&](std::pair<cv::Point2f, uint8_t> pt) {
        // Check if the point is out of frame, or within 1 radius of the edge
        return !cv::Rect(2 * circle_radius, 2 * circle_radius, image_size.width - 4 * circle_radius,
                         image_size.height - 4 * circle_radius)
                    .contains(pt.first);
      };
      std::erase_if(cam0_points_and_intensities, is_out_of_frame);
      std::erase_if(cam1_points_and_intensities, is_out_of_frame);

      // Save the points and intensities for later use
      points_cam0.emplace_back(cam0_points_and_intensities);
      points_cam1.emplace_back(cam1_points_and_intensities);

      // Generate Images with annotated points
      cv::Mat cam0_image = cv::Mat::zeros(image_size, CV_8UC1);
      cv::Mat cam1_image = cv::Mat::zeros(image_size, CV_8UC1);

      for (const auto& pt : points_cam0[i]) {
        cv::circle(cam0_image, pt.first, circle_radius, pt.second, circle_radius);
      }

      for (const auto& pt : points_cam1[i]) {
        cv::circle(cam1_image, pt.first, circle_radius, pt.second, circle_radius);
      }

      my_window_.setViewerPose(cam0_xform_inv);
      auto cam0_image_viz = my_window_.getScreenshot();
      cv::cvtColor(cam0_image_viz, cam0_image_viz, cv::COLOR_BGR2GRAY);

      // cv::viz::Camera cam1(K_cam1, image_size);
      // my_window_.setCamera(cam1);
      my_window_.setViewerPose(cam1_xform);
      auto cam1_image_viz = my_window_.getScreenshot();
      cv::cvtColor(cam1_image_viz, cam1_image_viz, cv::COLOR_BGR2GRAY);

      images.emplace_back(cam0_image_viz, cam1_image_viz);

      // images.emplace_back(cam0_image, cam1_image);
    }
  }

  cv::Matx33d K_cam0 = cv::Matx33d(image_size.width / 2.0, 0.0, image_size.width / 2.0, 0.0, image_size.height / 2.0,
                                   image_size.height / 2.0, 0.0, 0.0, 1.0);
  cv::Matx33d K_cam1 = K_cam0;  // Same camera matrix

  cv::Matx33d R = cv::Matx33d(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
  cv::Vec3d T = cv::Vec3d(-.05, 0.0, 0.0);

  std::vector<uint8_t> ground_plane_point_intensities;  //< Intensity of the ground plane points
  std::vector<cv::Point3f> ground_plane_points;         //< Points on the ground plane, in cam0 t=0 frame
  std::vector<mavlink_imu_t> imu_msgs;                  //< IMU messages, one for each image
  std::vector<cv::Affine3f> camera_trajectory;          //< Camera trajectory, this is what VIO should output
  std::vector<std::vector<std::pair<cv::Point2f, uint8_t>>>
      points_cam0;  //< Ground plane points in each image, in cam0 frame
  std::vector<std::vector<std::pair<cv::Point2f, uint8_t>>>
      points_cam1;  //< Ground plane points in each image, in cam1 frame
  std::vector<std::pair<typename IpBackend::image_type, typename IpBackend::image_type>>
      images;  //< Images, cam0 and cam1

  cv::viz::Viz3d my_window_;                           //< Vizualization window
  std::list<cv::viz::WImage3D> ground_plane_widgets_;  //< Vizualization widget
};

using ImageProcessorBackends = ::testing::Types<
#ifdef WITH_VPI
    VpiBackend,
#endif
    CvBackend, CvCpuBackend>;
TYPED_TEST_SUITE(ImageProcessingTestFixture, ImageProcessorBackends);

TYPED_TEST(ImageProcessingTestFixture, test2) {
  // Convert Images from cv::Mat to UMat
  // std::vector<std::pair<UMat<uint8_t>, UMat<uint8_t>>> images_umat;
  // images_umat.reserve(images.size());
  // std::transform(images.begin(), images.end(), std::back_inserter(images_umat),
  //                [](const std::pair<cv::Mat, cv::Mat>& image) {
  //                  return std::make_pair(UMat<uint8_t>(image.first), UMat<uint8_t>(image.second));
  //                });

  StereoCalibration stereo_calibration(this->K_cam0, this->K_cam1, {}, {}, this->R, this->T);
  ImageProcessor<TypeParam> image_processor(0, stereo_calibration, cv::Matx33d::eye());
  image_processor.Init();

  std::list<uint64_t> latencies;
  Vio<TypeParam> vio(stereo_calibration, cv::Matx33d::eye());
  std::vector<cv::Affine3d> calculated_trajectory;
  for (auto frame = 0; frame < num_images; frame++) {
    TrackedImagePoints<TypeParam> tracked_image_points;
    auto start = std::chrono::high_resolution_clock::now();
    auto first = this->images[frame].first;
    auto second = this->images[frame].second;
    image_processor.process_image(std::move(first), std::move(second),
                                  std::vector<mavlink_imu_t>{this->imu_msgs[frame]}, frame * 1E6, tracked_image_points);
    auto end = std::chrono::high_resolution_clock::now();
    latencies.push_back(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
    // image_processor.process_image(cam0_image_d, cam1_image_d, {}, frame * 1E6, output_points);
    // std::cout << "num pts " << tracked_image_points.ids.size() << std::endl;

    auto [camera_pose, inliers] = vio.ProcessPoints(tracked_image_points);
    calculated_trajectory.push_back(camera_pose);
    // std::cout << "position " << vio_output.position << std::endl;
    // std::cout << "gt position " << camera_trajectory[frame].translation() << std::endl;
    // cv::imshow("test.jpg", images[frame].second);
    // cv::waitKey(0);

    cv::viz::WTrajectory calculated_trajectory_widget(calculated_trajectory, cv::viz::WTrajectory::PATH, 1,
                                                      cv::viz::Color::green());
    cv::viz::WTrajectory ground_truth_trajectory(
        std::vector<cv::Affine3f>{this->camera_trajectory.begin(), this->camera_trajectory.begin() + frame + 1},
        cv::viz::WTrajectory::PATH, 2);
    this->my_window_.showWidget("ground_truth_trajectory", ground_truth_trajectory);
    this->my_window_.showWidget("calculcated_trajectory", calculated_trajectory_widget);
    this->my_window_.setViewerPose(cv::Affine3f(cv::Vec3f{0, 0, 0}, cv::Vec3f{2, 2, -3}));
    this->my_window_.spinOnce(1, true);
  }
  auto average_fps = 1.0E6 * latencies.size() / std::accumulate(latencies.begin(), latencies.end(), 0ul);
  std::cout << "average fps " << average_fps << std::endl;

  cv::Vec3d output_error;
  cv::subtract(this->camera_trajectory.back().translation(), calculated_trajectory.back().translation(), output_error);
  std::cout << "translation error " << output_error << std::endl;
}
