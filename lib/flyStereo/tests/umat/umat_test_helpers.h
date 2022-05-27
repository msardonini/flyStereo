
#include "flyStereo/types/umat.h"
#include "opencv2/core/core.hpp"

// We need a way to identify unique points, so we use an unordered map. Overload the std::hash functor for cv::Point2i
namespace std {
template <>
struct hash<cv::Point2i> {
  std::size_t operator()(const cv::Point2i &k) const;
};
}  // namespace std

/**
 * @brief Generate an array with num_pts random points and an 'answer key' point_map
 *
 * @tparam T
 * @param num_pts  The number of points to assign in the array
 * @param matsize  The size of the image to generate points in
 * @param mat     The image to generate points in
 * @param point_map The answer key for the points. This may be smaller than num_pts, if collisions randomly occur
 * @tparam T The data type of the image. Can be int, float, double, etc. for 2D and cv::Vec<T> for 3D
 */
template <typename T>
inline void generate_array(int num_pts, cv::Size matsize, cv::Mat_<T> &mat,
                           std::unordered_map<cv::Point2i, T> &point_map) {
  for (int i = 0; i < num_pts; i++) {
    const cv::Point2i tmp_pt(rand() % matsize.width, rand() % matsize.height);

    T tmp_vec;
    if constexpr (std::is_integral<T>::value || std::is_floating_point<T>::value) {
      tmp_vec = static_cast<T>(rand() % std::numeric_limits<uint64_t>::max());
    } else {
      for (auto j = 0; j < tmp_vec.rows; j++) {
        tmp_vec[j] = static_cast<T::value_type>(rand() % std::numeric_limits<int>::max());
      }
    }
    point_map.emplace(tmp_pt, tmp_vec);
  }

  // Create images, set to zero and fill with the random points
  mat = cv::Mat_<T>::zeros(matsize);

  for (const auto &pt : point_map) {
    mat(pt.first) = pt.second;
  }
}

/**
 * @brief Ensures that the given mat has the same data as the original frame.
 *
 * @tparam U Data type of the cv::Mat
 * @param mat Mat to check
 * @return true Mat has the same data as the original frame
 * @return false Mat does not have the same data as the original frame
 */
template <typename U>
bool check_pts(const cv::Mat_<U> &mat, const std::unordered_map<cv::Point2i, U> pts) {
  for (const auto &pt : pts) {
    if (mat(pt.first) != pt.second) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Ensures that the given mat has the same data as the original frame. Checks both underlying cv::Mat and the
 * cv::cuda::GpuMat
 *
 * @tparam U Data type of the UMat
 * @param umat UMat<U> to check
 * @param pts Ground truth points
 * @return true UMat has the same data as the original frame
 * @return false UMat does not have the same data as the original frame
 */
template <typename U>
bool check_pts(const UMat<U> &umat, const std::unordered_map<cv::Point2i, U> pts) {
  bool ret_val = true;
  ret_val &= check_pts(umat.frame(), pts);

  cv::Mat_<U> tmp;
  umat.d_frame().download(tmp);
  ret_val &= check_pts(tmp, pts);
  return ret_val;
}
