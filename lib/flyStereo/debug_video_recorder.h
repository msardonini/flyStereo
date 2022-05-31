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

std::unique_ptr<cv::VideoWriter> writer;
std::vector<cv::Scalar> colors{CV_RGB(0, 0, 255), CV_RGB(0, 255, 0), CV_RGB(255, 0, 0), CV_RGB(0, 255, 255)};

template <bool enable = FLYSTEREO_VISUALIZE_DEBUG_VIDEO_ENABLED>
void draw_points_to_frame(cv::Mat& image, const UMat<cv::Vec2f>& points, cv::Scalar color, int radius = 5) {
  if constexpr (!enable) {
    return;
  }
  const auto& points_mat = points.frame();
  std::for_each(points_mat.begin(), points_mat.end(), [&](const cv::Vec2f& point) {
    cv::circle(image, cv::Point(point[0], point[1]), radius, color, -1, cv::FILLED);
  });
}

template <typename point_type, bool enable = FLYSTEREO_VISUALIZE_DEBUG_VIDEO_ENABLED>
void draw(cv::Mat& image, std::initializer_list<point_type>& points) {
  if constexpr (!enable) {
    return;
  }

  if (points.size() > colors.size()) {
    throw std::runtime_error("Not enough colors to draw all the different points!");
  }

  int counter = 0;
  std::for_each(points.begin(), points.end(),
                [&](const UMat<cv::Vec2f>& points_mat) { draw_points_to_frame(image, points_mat, colors[counter++]); });
}

template <bool enable = FLYSTEREO_VISUALIZE_DEBUG_VIDEO_ENABLED>
void export_frame(const cv::Mat& frame, bool write_to_screen = true) {
  if constexpr (!enable) {
    return;
  }
  if (write_to_screen) {
    cv::imshow("frame", frame);
    cv::waitKey(0);
  } else {
    if (!writer) {
      writer = std::make_unique<cv::VideoWriter>("./debug_video.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 2,
                                                 frame.size(), true);
    }
    if (writer->isOpened()) {
      writer->write(frame);
    } else {
      std::cerr << "failed to open video!!\n";
    }
  }
}

template <typename point_type, bool enable = FLYSTEREO_VISUALIZE_DEBUG_VIDEO_ENABLED>
void plot_points_overlay_2x1(const UMat<uint8_t>& image0, std::initializer_list<point_type> points0,
                             const UMat<uint8_t>& image1, std::initializer_list<point_type> points1) {
  if constexpr (!enable) {
    return;
  }

  auto image_points0 = std::make_pair(image0, points0);
  cv::Mat image0_color;
  cv::cvtColor(image0.frame(), image0_color, cv::COLOR_GRAY2BGR);

  auto image_points1 = std::make_pair(image1, points1);
  cv::Mat image1_color;
  cv::cvtColor(image1.frame(), image1_color, cv::COLOR_GRAY2BGR);

  draw(image0_color, points0);
  draw(image1_color, points1);

  cv::Mat concat_frame;
  cv::hconcat(image0_color, image1_color, concat_frame);

  // Draw a vertical line to show the separation between the two cameras
  cv::line(concat_frame, cv::Point(concat_frame.cols / 2, 0), cv::Point(concat_frame.cols / 2, concat_frame.rows),
           cv::Scalar(0, 255, 0), 1, 8, 0);

  export_frame(concat_frame);
}
