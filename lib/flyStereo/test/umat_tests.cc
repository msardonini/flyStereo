#include <utility>
#include <vector>

#include "flyStereo/umat.h"
#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/cuda.hpp"

constexpr int num_pts = 100;
const cv::Size2i imsize(640, 480);

template <typename T>
class UMatTestFixture : public ::testing::Test {
 public:
  UMatTestFixture() {
    // Generate random points
    pts_.resize(num_pts);
    for (int i = 0; i < num_pts; i++) {
      const cv::Point2i tmp_pt(rand() % imsize.width, rand() % imsize.height);

      T tmp_vec;
      if constexpr (std::is_integral<T>::value || std::is_floating_point<T>::value) {
        tmp_vec = static_cast<T>(rand() % std::numeric_limits<uint64_t>::max());
      } else {
        for (auto j = 0; j < tmp_vec.rows; j++) {
          tmp_vec[j] = static_cast<T::value_type>(rand() % std::numeric_limits<uint64_t>::max());
        }
      }

      // std::for_each(tmp_vec.begin(), tmp_vec.end(),
      //               [&](T &val) { val = });
      pts_[i] = std::make_pair(tmp_pt, tmp_vec);
    }

    // Create images, set to zero and fill with the random points
    frame_ = cv::Mat_<T>::zeros(imsize);

    for (const auto &pt : pts_) {
      frame_(pt.first) = pt.second;
    }
    d_frame_.upload(frame_);
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
  std::vector<std::pair<cv::Point2i, T>> pts_;
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

TYPED_TEST(UMatTestFixture, DecreaseNumRows) {
  UMat<TypeParam> umat(this->frame_);
  umat.decrease_num_rows(100);

  EXPECT_EQ(umat.frame().size(), cv::Size2i(imsize.width, 100));
}
