#pragma once

#include <map>

#include "flyStereo/types/umat.h"
#include "flyStereo/types/umat_vpiarray.h"
#include "opencv2/core.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaoptflow.hpp"
#include "opencv2/features2d.hpp"

/**
 * @brief Removes points that do not have good status
 *
 * @param status The status of the points, typically returned from calcOpticalFlowPyrLK
 * @param points The points to modify
 */
inline void RemovePoints(const UMat<uint8_t> &status, const uint8_t success_value, UMat<cv::Vec2f> &points) {
  auto &points_frame = points.frame();
  auto &status_frame = status.frame();
  // Check to make sure the points match
  if (points_frame.size() != status_frame.size()) {
    throw std::runtime_error("The number of points and the number of status values do not match");
  }

  int point_index = 0;
  for (int i = 0; i < status_frame.cols; i++) {
    if (status_frame(0, i) == success_value) {
      points_frame(0, point_index)[0] = points_frame(0, i)[0];
      points_frame(0, point_index)[1] = points_frame(0, i)[1];
      point_index++;
    }
  }
  points.decrease_num_cols(point_index);
}

template <typename T>
inline void RemovePoints(const UMat<uint8_t> &status, const uint8_t success_value, std::vector<T> &points) {
  const auto &status_frame = status.frame();

  if (static_cast<int>(points.size()) != status_frame.cols) {
    throw std::runtime_error("The number of points and the number of status values do not match");
  }

  int point_index = 0;
  for (int i = 0; i < status_frame.cols; i++) {
    if (status_frame(0, i) == success_value) {
      points[point_index] = points[i];
      point_index++;
    }
  }
  points.resize(point_index);
}

template <typename... Args>
inline void RemovePoints(const UMat<uint8_t> &status, const uint8_t success_value, Args &... args) {
  (RemovePoints(status, success_value, args), ...);
}

inline void MarkPointsOutOfFrame(UMat<uint8_t> &status, const UMat<cv::Vec2f> &points, const cv::Size &frame_size,
                                 const uint8_t success_value) {
  const auto &points_frame = points.frame();
  auto &status_frame = status.frame();
  if (points_frame.size() != status_frame.size()) {
    throw std::runtime_error("The number of points and the number of status values do not match");
  }

  // Iterate over the rows TODO make lamba and use for_each
  for (int i = 0; i < points_frame.cols; i++) {
    const cv::Point2i pt(points_frame(0, i)[0], points_frame(0, i)[1]);
    const cv::Rect area(cv::Point2i(0, 0), frame_size);

    if (!area.contains(pt)) {
      status_frame(0, i) = !success_value;
    }
  }
}

inline UMat<cv::Vec2f> AppendUMatColwise(const UMat<cv::Vec2f> &mat1, const UMat<cv::Vec2f> &mat2) {
  const auto &mat1_f = mat1.frame();
  const auto &mat2_f = mat2.frame();

  // Edge case: if the matrices are empty, return an empty matrix
  if (mat1_f.cols == 0 && mat2_f.cols == 0) {
    return UMat<cv::Vec2f>(cv::Size(0, 1));
  }

  // Check edge cases where one of the UMats is empty
  if (mat1_f.cols == 0) {
    return mat2;
  } else if (mat2_f.cols == 0) {
    return mat1;
  }
  // Check the number of rows are the same
  if (mat1_f.rows != mat2_f.rows) {
    throw std::runtime_error("Cannot append cols, the number of rows do not match");
  }

  UMat<cv::Vec2f> new_umat(cv::Size2i(mat1_f.cols + mat2_f.cols, mat1_f.rows));

  cv::Range range_rows(0, mat1_f.rows);
  cv::Range range_cols1(0, mat1_f.cols);
  cv::Range range_cols2(mat1_f.cols, mat1_f.cols + mat2_f.cols);

  mat1_f.copyTo(new_umat.frame()(range_rows, range_cols1));
  mat2_f.copyTo(new_umat.frame()(range_rows, range_cols2));
  return new_umat;
}

inline bool BinAndMarkPoints(const UMat<cv::Vec2f> points, const std::vector<unsigned int> point_ids,
                             const cv::Size &framesize, const cv::Size bins_size, const unsigned int max_pts_in_bin,
                             UMat<uint8_t> &status, const uint8_t success_value) {
  const auto &points_f = points.frame();
  status = cv::Mat_<uint8_t>(1, points_f.cols, success_value);

  // Place all of our image points in our map
  // First, calculate the row-major index of the bin, this will be the map's key
  std::map<int, std::vector<std::tuple<unsigned int, unsigned int, cv::Point2f> > > grid;
  for (int i = 0; i < points_f.cols; i++) {
    unsigned int bin_row = points_f(0, i)[1] * bins_size.height / framesize.height;
    unsigned int bin_col = points_f(0, i)[0] * bins_size.width / framesize.width;

    unsigned int index = bins_size.width * bin_row + bin_col;
    grid[index].push_back({i, point_ids[i], cv::Point2f(points_f(0, i)[0], points_f(0, i)[1])});
  }

  // Now that the grid is full, delete points that exceed the threshold of points per bin
  for (auto it = grid.begin(); it != grid.end(); it++) {
    if (it->second.size() >= max_pts_in_bin) {
      // Sort each vector in the grid by the age of the point (done by ID) lower IDs are older points
      std::sort(it->second.begin(), it->second.end(),
                [](auto const &a, auto const &b) { return std::get<1>(a) < std::get<1>(b); });

      // Mark the points that should be deleted
      for (size_t i = max_pts_in_bin; i < it->second.size(); i++) {
        status.frame()(0, std::get<0>(it->second[i])) = !success_value;
      }
    }
  }
  return true;
}
