#ifndef LIB_INCLUDE_FLY_STEREO_OPTICAL_FLOW_POINTS_H_
#define LIB_INCLUDE_FLY_STEREO_OPTICAL_FLOW_POINTS_H_

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaoptflow.hpp"



// Define constant indexes for the use with this object
// t = tracked points vs. d = detected points
// c0 = cam0 vs. c1 = cam1
// t0 = time0 (previous timestep) vs t1 = time1 (current timestep)
constexpr unsigned int t_c0_t0 = 0; 
constexpr unsigned int t_c0_t1 = 1; 
constexpr unsigned int t_c1_t0 = 2; 
constexpr unsigned int t_c1_t1 = 3; 
constexpr unsigned int d_c0_t0 = 4; 
constexpr unsigned int d_c0_t1 = 5; 
constexpr unsigned int d_c1_t0 = 6; 
constexpr unsigned int d_c1_t1 = 7;

// Define constant indexes for the ids vectors
constexpr unsigned int ids_t0 = 0;
constexpr unsigned int ids_t1 = 1;


struct OpticalFlowPoints {
  OpticalFlowPoints() {}

  template <typename T>
  bool RemovePoints(const std::vector<uchar> &status, std::vector<T> &vec) {
    // Check to make sure the sizes match
    if (vec.size() != status.size()) {
      std::cerr << "Error! status and ids are not the same size" << std::endl;
      return false;
    }
    // Remove the poiints that have a bad status flag
    for (int j = status.size() - 1; j >= 0; j--) {
      if (status[j] == 0) {
        vec.erase(vec.begin() + j);
      }
    }
    return true;
  }

  bool RemoveOutliers(const std::vector<uchar> &status, const std::vector<unsigned int>
    &vecs_to_use, const std::vector<unsigned int> &id_vec_to_use = {}) {
    // We need a CPU vector for this operation, download it and run the remove function, and
    // re-upload it again
    for (int i = 0; i < vecs_to_use.size(); i++) {
      std::vector<cv::Point2f> points = GetCpu(vecs_to_use[i]);
      if(!RemovePoints<cv::Point2f>(status, points)) {
        return false;
      }
      LoadCpu(points, vecs_to_use[i]);
    }

    // If requested, remove the outliers for the ID vectors too
    for (unsigned int i = 0; i < id_vec_to_use.size(); i++) {
      if (!RemovePoints<unsigned int>(status, ids[id_vec_to_use[i]])) {
        return false;
      }
    }

    return true;
  }

  bool RemoveOutliers(const cv::cuda::GpuMat &d_status, const std::vector<unsigned int>
    &vecs_to_use, const std::vector<unsigned int> &id_vec_to_use = {}) {
    std::vector<uchar> status;
    d_status.download(status);
    return RemoveOutliers(status, vecs_to_use, id_vec_to_use);
  }

  bool MarkPointsOutOfFrame(const cv::Size &framesize, const unsigned int index,
    std::vector<unsigned char> &status) {
    // We need a CPU vector for this operation
    std::vector<cv::Point2f> points = GetCpu(index);

    // Check to make sure the sizes match
    if (status.size() != points.size()) {
      std::cerr << "Error! status and points are not the same size" << std::endl;
      return false;
    }

    // Mark those tracked points out of the image region as untracked.
    for (int j = 0; j < points.size(); j++) {
      if (status[j] == 0) {
        continue;
      }
      if (points[j].y < 0 || points[j].y > framesize.height - 1 ||
          points[j].x < 0 || points[j].x > framesize.width - 1) {
        status[j] = 0;
      }
    }
    return true;
  }

  bool MarkPointsOutOfFrame(const cv::Size &framesize, const unsigned int index, cv::cuda::GpuMat
    &d_status) {
    std::vector<unsigned char> status;
    if (!d_status.empty()) {
      d_status.download(status);
    } else {
      return false;
    }
    if (!MarkPointsOutOfFrame(framesize, index, status)) {
      return false;
    }
    d_status.upload(status);
    return true;
  }

  bool AppendGpuMatColwise(const unsigned int index_src, const unsigned int index_dst) {
    // Handle edge cases where one of the input mats is empty
    if (d_points[index_src].cols == 0) {
      if (d_points[index_dst].cols == 0) {
        d_points[index_dst] = cv::cuda::GpuMat();
        return true;
      } else {
        d_points[index_dst] = d_points[index_dst].clone();
        return true;
      }
    } else if (d_points[index_dst].cols == 0) {
      d_points[index_dst] = d_points[index_src].clone();
      return true;
    }

    // Verify the two mats are the same data type
    if (d_points[index_src].type() != d_points[index_dst].type()) {
      std::cerr << "Error! [AppendGpuMat] Mats are not the same type" << std::endl;
      d_points[index_dst] = cv::cuda::GpuMat();
      return false;
    } else if (d_points[index_src].rows != d_points[index_dst].rows) {
      std::cerr << "Error! [AppendGpuMat] Mats do not have the same amount of"
        " rows" << std::endl;
      d_points[index_dst] = cv::cuda::GpuMat();
      return false;
    }

    cv::Range range_rows(0, d_points[index_src].rows);
    cv::Range range_cols1(0, d_points[index_src].cols);
    cv::Range range_cols2(d_points[index_src].cols, d_points[index_src].cols +
      d_points[index_dst].cols);

    cv::cuda::GpuMat append_mat(d_points[index_src].rows, d_points[index_src].cols +
      d_points[index_dst].cols, d_points[index_src].type());

    d_points[index_src].copyTo(append_mat(range_rows, range_cols1));
    d_points[index_dst].copyTo(append_mat(range_rows, range_cols2));

    d_points[index_dst] = std::move(append_mat);
    return true;
  }

  bool BinAndMarkPoints(const unsigned int index_vec, const unsigned int index_ids,
    const cv::Size &framesize, const unsigned int bins_width, const unsigned int bins_height,
    const unsigned int max_pts_in_bin, std::vector<unsigned char> &status) {
    // We need a CPU vector for this operation
    std::vector<cv::Point2f> points = GetCpu(index_vec);

    status.resize(points.size());
    // Initially set all the statuses to true
    for (int i = 0; i < status.size(); i++) {
      status[i] = 1;
    }

    // Place all of our image points in our map
    // First, calculate the row-major index of the bin, this will be the map's key
    std::map<int, std::vector<std::tuple<unsigned int, unsigned int, cv::Point2f> > > grid;
    for (unsigned int i = 0; i < points.size(); i++) {
      unsigned int bin_row = points[i].y * bins_height / framesize.height;
      unsigned int bin_col = points[i].x * bins_width / framesize.width;

      unsigned int index = bins_width * bin_row + bin_col;
      grid[index].push_back({i, ids[index_ids][i], points[i]});
    }

    // Now that the grid is full, delete points that exceed the threshold of points per bin
    for (auto it = grid.begin(); it != grid.end(); it++) {
      if (it->second.size() >= max_pts_in_bin) {

        // Sort each vector in the grid by the age of the point (done by ID) lower IDs are older
        // points
        std::sort(it->second.begin(), it->second.end(), [](auto const &a, auto const &b)
          { return std::get<1>(a) > std::get<1>(b); });

        // Mark the points that should be deleted
        for (int i = max_pts_in_bin; i < it->second.size(); i++) {
          status[std::get<0>(it->second[i])] = 0;
        }
      }
    }
    return true;
  }


  // Access
  std::vector<cv::Point2f> GetCpu(const unsigned int index) {
    if (d_points[index].empty()) {
      return {};
    }
    std::vector<cv::Point2f> vec;
    d_points[index].download(vec);
    return vec;
  }

  // Write
  void LoadCpu(const std::vector<cv::Point2f> &vec, const unsigned int index) {
    d_points[index].upload(vec);
  }

  cv::cuda::GpuMat& operator[] (unsigned int index) {
    return d_points[index];
  }


  // Data Members

  // Data structure for tracked points
  // index 0: tracked points, cam0 t0
  // index 1: tracked points, cam0 t1
  // index 2: tracked points, cam1 t0
  // index 3: tracked points, cam1 t1
  // index 4: detected points, cam0 t0
  // index 5: detected points, cam0 t1
  // index 6: detected points, cam1 t0
  // index 7: detected points, cam1 t1
  std::array<cv::cuda::GpuMat, 8> d_points;

  // index 0 ids, tracked pts
  // index 1 ids, detected pts
  std::array<std::vector<unsigned int>, 2> ids;
};

#endif  // LIB_INCLUDE_FLY_STEREO_OPTICAL_FLOW_POINTS_H_
