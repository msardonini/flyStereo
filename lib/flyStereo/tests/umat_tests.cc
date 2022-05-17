#include <utility>
#include <vector>

#include "flyStereo/umat.h"
#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/cuda.hpp"

// We need a way to identify unique points, so we use an unordered map. Overload the std::hash functor for cv::Point2i
namespace std {
template <>
struct hash<cv::Point2i> {
  std::size_t operator()(const cv::Point2i &k) const { return std::hash<int>()(k.y) << 32 | std::hash<int>()(k.x); }
};
}  // namespace std

constexpr int num_pts = 300;
const cv::Size2i imsize(1280, 500);

template <typename T>
class UMatTestFixture : public ::testing::Test {
 public:
  UMatTestFixture() {
    // Generate random points
    for (int i = 0; i < num_pts; i++) {
      const cv::Point2i tmp_pt(rand() % imsize.width, rand() % imsize.height);

      T tmp_vec;
      if constexpr (std::is_integral<T>::value || std::is_floating_point<T>::value) {
        tmp_vec = static_cast<T>(rand() % std::numeric_limits<uint64_t>::max());
      } else {
        for (auto j = 0; j < tmp_vec.rows; j++) {
          tmp_vec[j] = static_cast<T::value_type>(rand() % std::numeric_limits<int>::max());
        }
      }
      pts_.emplace(tmp_pt, tmp_vec);
    }

    // Create images, set to zero and fill with the random points
    frame_ = cv::Mat_<T>::zeros(imsize);

    for (const auto &pt : pts_) {
      frame_(pt.first) = pt.second;
    }
    d_frame_.upload(frame_.clone());
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
  bool check_pts(const cv::Mat_<U> &mat) {
    for (const auto &pt : pts_) {
      if (mat(pt.first) != pt.second) {
        return false;
      }
    }
    return true;
  }

  template <typename U>
  bool check_pts(const UMat<U> &umat) {
    bool ret_val = true;
    ret_val &= check_pts(umat.frame());

    cv::Mat_<U> tmp;
    umat.d_frame().download(tmp);
    ret_val &= check_pts(tmp);
    return ret_val;
  }

 protected:
  std::unordered_map<cv::Point2i, T> pts_;
  cv::Mat_<T> frame_;
  cv::cuda::GpuMat d_frame_;
};

using UMatTypes =
    ::testing::Types<uint8_t, uint16_t, int, float, double, cv::Vec2b, cv::Vec2i, cv::Vec2s, cv::Vec2f, cv::Vec2d>;
TYPED_TEST_SUITE(UMatTestFixture, UMatTypes);

TYPED_TEST(UMatTestFixture, MatConstructor) {
  UMat<TypeParam> umat(this->frame_);

  EXPECT_TRUE(this->check_pts(umat));
}

TYPED_TEST(UMatTestFixture, GpuMatConstructor) {
  UMat<TypeParam> umat(this->d_frame_);

  EXPECT_TRUE(this->check_pts(umat));
}

TYPED_TEST(UMatTestFixture, SizeConstructor) {
  UMat<TypeParam> umat(imsize);

  EXPECT_EQ(umat.frame().size(), imsize);
}

TYPED_TEST(UMatTestFixture, CopyConstructor) {
  UMat<TypeParam> umat(this->frame_);
  UMat<TypeParam> umat_copy(umat);

  EXPECT_TRUE(this->check_pts(umat));
}

TYPED_TEST(UMatTestFixture, MoveConstructor) {
  UMat<TypeParam> umat(this->frame_);
  UMat<TypeParam> umat_copy(std::move(umat));

  EXPECT_EQ(umat.frame().size(), cv::Size2i(0, 0));
  EXPECT_TRUE(this->check_pts(umat_copy));
}

TYPED_TEST(UMatTestFixture, CopyAssignmentOperator) {
  UMat<TypeParam> umat(this->frame_);
  UMat<TypeParam> umat_copy(imsize);

  umat_copy = umat;

  EXPECT_TRUE(this->check_pts(umat));
}

TYPED_TEST(UMatTestFixture, MoveAssignmentOperator) {
  UMat<TypeParam> umat(this->frame_);
  UMat<TypeParam> umat_copy(imsize);

  umat_copy = std::move(umat);

  EXPECT_EQ(umat.frame().size(), cv::Size2i(0, 0));
  EXPECT_TRUE(this->check_pts(umat_copy));
}

TYPED_TEST(UMatTestFixture, MatAssignmentOperator) {
  UMat<TypeParam> umat;
  umat = this->frame_;

  EXPECT_TRUE(this->check_pts(umat));
}

TYPED_TEST(UMatTestFixture, GpuMatAssignmentOperator) {
  UMat<TypeParam> umat;
  umat = this->d_frame_;

  EXPECT_TRUE(this->check_pts(umat));
}

TYPED_TEST(UMatTestFixture, Clone) {
  UMat<TypeParam> umat(this->frame_);
  auto umat_clone = umat.clone();

  EXPECT_TRUE(this->check_pts(umat_clone));

  // Check the mats are not sharing data
  if constexpr (std::is_integral<TypeParam>::value || std::is_floating_point<TypeParam>::value) {
    umat.frame()(0, 0) = !umat.frame()(0, 0);
  } else {
    umat.frame()(0, 0)[0] = !umat.frame()(0, 0)[0];
  }
  EXPECT_NE(umat_clone.frame()(0, 0), umat.frame()(0, 0));
}

TYPED_TEST(UMatTestFixture, DecreaseNumRows) {
  UMat<TypeParam> umat(this->frame_);
  umat.decrease_num_rows(100);

  EXPECT_EQ(umat.frame().size(), cv::Size2i(imsize.width, 100));
}
