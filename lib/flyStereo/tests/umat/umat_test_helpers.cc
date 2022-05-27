
#include "flyStereo/tests/umat/umat_test_helpers.h"

#include "opencv2/core/core.hpp"

namespace std {

std::size_t hash<cv::Point2i>::operator()(const cv::Point2i &k) const {
  return std::hash<int>()(k.y) << 32 | std::hash<int>()(k.x);
}

}  // namespace std
