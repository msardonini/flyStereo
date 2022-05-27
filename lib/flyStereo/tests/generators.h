#pragma once

#include <tuple>

#include "flyStereo/types/umat.h"
#include "opencv2/core/core.hpp"

inline auto generate_image_points(const cv::Size& image_size, const int num_points, int translation_x = 5,
                                  int translation_y = 5) {
  UMat<cv::Vec2f> prev_pts(cv::Size(num_points, 1));
  UMat<cv::Vec2f> curr_pts(cv::Size(num_points, 1));

  // Generate random points to track

  cv::Mat_<uint8_t> curr_frame_mat = cv::Mat_<uint8_t>::zeros(image_size);
  cv::Mat_<uint8_t> prev_frame_mat = cv::Mat_<uint8_t>::zeros(image_size);
  for (auto i = 0; i < prev_pts.frame().cols; i++) {
    prev_pts.frame()(i) = cv::Vec2f(rand() % (prev_frame_mat.cols - 2 * translation_x),
                                    rand() % (prev_frame_mat.rows - 2 * translation_y));
    cv::Point2i pt_prev(prev_pts.frame()(i)[0], prev_pts.frame()(i)[1]);
    cv::circle(prev_frame_mat, pt_prev, 2, cv::Scalar(255), -1);

    curr_pts.frame()(i) = prev_pts.frame()(i) + cv::Vec2f(translation_x, translation_y);
    cv::Point2i pt_curr(curr_pts.frame()(i)[0], curr_pts.frame()(i)[1]);
    cv::circle(curr_frame_mat, pt_curr, 2, cv::Scalar(255), -1);
  }
  UMat<uint8_t> curr_frame(curr_frame_mat);
  UMat<uint8_t> prev_frame(prev_frame_mat);

  return std::make_tuple(prev_frame, curr_frame, prev_pts, curr_pts);
}
