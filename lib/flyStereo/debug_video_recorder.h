#pragma once

#include <memory>

#include "flyStereo/types/umat.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#ifdef FLYSTEREO_VISUALIZE_DEBUG_VIDEO
constexpr bool FLYSTEREO_VISUALIZE_DEBUG_VIDEO_ENABLED = true;
#else
constexpr bool FLYSTEREO_VISUALIZE_DEBUG_VIDEO_ENABLED = false;
#endif

static std::unique_ptr<cv::VideoWriter> writer;
static std::vector<cv::Scalar> colors{CV_RGB(0, 0, 255), CV_RGB(0, 255, 0), CV_RGB(255, 0, 0), CV_RGB(0, 255, 255)};

/**
 * @brief Draws the points on the image as small circles
 *
 * @param image The image to draw points to
 * @param points The points to draw
 * @tparam point_type The datatype holding the points
 * @tparam enable true if this function is enabled, false otherwise
 */
template <UMatDerivative point_type, bool enable = FLYSTEREO_VISUALIZE_DEBUG_VIDEO_ENABLED>
void draw_points_to_frame(cv::Mat& image, std::initializer_list<point_type>& points) {
  if constexpr (!enable) {
    return;
  }

  if (points.size() > colors.size()) {
    throw std::runtime_error("Not enough colors to draw all the different points!");
  }

  int radius = 5;
  int counter = 0;
  auto draw_func = [&](cv::Mat& image, const point_type& points, cv::Scalar color) {
    const auto& points_mat = points.frame();
    std::for_each(points_mat.begin(), points_mat.end(), [&](const cv::Vec2f& point) {
      cv::circle(image, cv::Point(point[0], point[1]), radius, color, -1, cv::FILLED);
    });
  };

  std::for_each(points.begin(), points.end(),
                [&](const point_type& points_mat) { draw_func(image, points_mat, colors[counter++]); });
}

/**
 * @brief Exports the frame to optional output
 *
 * @param frame The frame to export
 * @param write_to_screen if true, the frame is written to the screen. Otherwise it's written to the video file
 * 'debug_video.avi'
 * @param name if write_to_screen is true, this is the name of the window to write to. Otherwise it's the name of the
 * file generated with the video. The avi exention is added automatically.
 * @tparam enable true if this function is enabled, false otherwise
 */
template <bool enable = FLYSTEREO_VISUALIZE_DEBUG_VIDEO_ENABLED>
void export_frame(const cv::Mat& frame, bool write_to_screen = true, std::string name = "debug_video") {
  if constexpr (!enable) {
    return;
  }
  if (write_to_screen) {
    cv::imshow(name, frame);
    cv::waitKey(1);
  } else {
    if (!writer) {
      writer = std::make_unique<cv::VideoWriter>(name + ".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 2,
                                                 frame.size(), true);
    }
    if (writer->isOpened()) {
      writer->write(frame);
    } else {
      std::cerr << "failed to open video!!\n";
    }
  }
}

/**
 * @brief Plots an overlay of the points on two images that are side by side. The points are drawn as small circles.
 *
 * @param image0 The first image, drawn on the left side of the overlay
 * @param points0 The points to draw on the first image
 * @param image1 The second image, drawn on the right side of the overlay
 * @param points1 The points to draw on the second image
 * @tparam point_type The data type of the points
 * @tparam enable true if this function is enabled, false otherwise
 */
template <UMatDerivative image_type, UMatDerivative point_type, bool enable = FLYSTEREO_VISUALIZE_DEBUG_VIDEO_ENABLED>
void plot_points_overlay_2x1(const image_type& image0, std::initializer_list<point_type> points0,
                             const image_type& image1, std::initializer_list<point_type> points1) {
  if constexpr (!enable) {
    return;
  }
  const auto image_size = image0.size();
  if (image_size != image1.size()) {
    return;
  }

  cv::Mat image0_color;
  cv::cvtColor(image0.frame(), image0_color, cv::COLOR_GRAY2BGR);
  draw_points_to_frame(image0_color, points0);

  cv::Mat image1_color;
  cv::cvtColor(image1.frame(), image1_color, cv::COLOR_GRAY2BGR);
  draw_points_to_frame(image1_color, points1);

  cv::Mat concat_frame;
  cv::hconcat(image0_color, image1_color, concat_frame);

  // Draw a vertical line to show the separation between the two cameras
  cv::line(concat_frame, cv::Point(concat_frame.cols / 2, 0), cv::Point(concat_frame.cols / 2, concat_frame.rows),
           cv::Scalar(0, 255, 0), 1, 8, 0);

  cv::imwrite("test.png", image0.frame());

  export_frame(concat_frame, true, "2x1");
}

template <UMatDerivative image_type, UMatDerivative point_type, bool enable = FLYSTEREO_VISUALIZE_DEBUG_VIDEO_ENABLED>
void plot_points_overlay_2x2(const image_type& image0, std::initializer_list<point_type> points0,
                             const image_type& image1, std::initializer_list<point_type> points1,
                             const image_type& image2, std::initializer_list<point_type> points2,
                             const image_type& image3, std::initializer_list<point_type> points3) {
  if constexpr (!enable) {
    return;
  }
  const auto image_size = image0.frame().size();
  if (image_size != image1.frame().size() || image_size != image2.frame().size() ||
      image_size != image3.frame().size()) {
    return;
  }

  cv::Mat image0_color;
  cv::cvtColor(image0.frame(), image0_color, cv::COLOR_GRAY2BGR);
  draw_points_to_frame(image0_color, points0);

  cv::Mat image1_color;
  cv::cvtColor(image1.frame(), image1_color, cv::COLOR_GRAY2BGR);
  draw_points_to_frame(image1_color, points1);

  cv::Mat image2_color;
  cv::cvtColor(image2.frame(), image2_color, cv::COLOR_GRAY2BGR);
  draw_points_to_frame(image2_color, points2);

  cv::Mat image3_color;
  cv::cvtColor(image3.frame(), image3_color, cv::COLOR_GRAY2BGR);
  draw_points_to_frame(image3_color, points3);

  // cv::Mat concat_frame(image0.size() * 2, CV_8uC3);
  cv::Mat hconcat0;
  cv::hconcat(image0_color, image1_color, hconcat0);
  cv::Mat hconcat1;
  cv::hconcat(image2_color, image3_color, hconcat1);
  cv::Mat concat_frame;
  cv::vconcat(hconcat0, hconcat1, concat_frame);

  // Draw a vertical line to show the separation between the two cameras
  cv::line(concat_frame, cv::Point(concat_frame.cols / 2, 0), cv::Point(concat_frame.cols / 2, concat_frame.rows),
           cv::Scalar(0, 255, 0), 1, 8, 0);

  // Draw a horizontal line to show the separation between the two cameras
  cv::line(concat_frame, cv::Point(0, concat_frame.rows / 2), cv::Point(concat_frame.cols, concat_frame.rows / 2),
           cv::Scalar(0, 255, 0), 1, 8, 0);

  export_frame(concat_frame, true, "2x2");
}
