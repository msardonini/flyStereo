#include <utility>
#include <vector>

#include "flyStereo/tests/umat/umat_test_helpers.h"
#include "flyStereo/types/umat.h"
#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/cuda.hpp"

constexpr int num_pts = 300;
const cv::Size2i imsize(1280, 500);

template <typename T>
class UMatTestFixture : public ::testing::Test {
 public:
  UMatTestFixture() {
    // Generate random points
    generate_array(num_pts, imsize, frame_, pts_);
    d_frame_.upload(frame_.clone());
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

  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TYPED_TEST(UMatTestFixture, GpuMatConstructor) {
  UMat<TypeParam> umat(this->d_frame_);

  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TYPED_TEST(UMatTestFixture, SizeConstructor) {
  UMat<TypeParam> umat(imsize);

  EXPECT_EQ(umat.frame().size(), imsize);
}

TYPED_TEST(UMatTestFixture, CopyConstructor) {
  UMat<TypeParam> umat(this->frame_);
  UMat<TypeParam> umat_copy(umat);

  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TYPED_TEST(UMatTestFixture, MoveConstructor) {
  UMat<TypeParam> umat(this->frame_);
  UMat<TypeParam> umat_copy(std::move(umat));

  EXPECT_EQ(umat.frame().size(), cv::Size2i(0, 0));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));
}

TYPED_TEST(UMatTestFixture, CopyAssignmentOperator) {
  UMat<TypeParam> umat(this->frame_);
  UMat<TypeParam> umat_copy(imsize);

  umat_copy = umat;

  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TYPED_TEST(UMatTestFixture, MoveAssignmentOperator) {
  UMat<TypeParam> umat(this->frame_);
  UMat<TypeParam> umat_copy(imsize);

  umat_copy = std::move(umat);

  EXPECT_EQ(umat.frame().size(), cv::Size2i(0, 0));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));
}

TYPED_TEST(UMatTestFixture, MatAssignmentOperator) {
  UMat<TypeParam> umat;
  umat = this->frame_;

  EXPECT_TRUE(check_pts(this->frame_, this->pts_));
  EXPECT_TRUE(check_pts(umat, this->pts_));

  // Rerun test when umat is already populated
  umat = UMat<TypeParam>(cv::Size(10, 10));
  umat = this->frame_;

  EXPECT_TRUE(check_pts(this->frame_, this->pts_));
  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TYPED_TEST(UMatTestFixture, GpuMatAssignmentOperator) {
  UMat<TypeParam> umat;
  umat = this->d_frame_;

  EXPECT_TRUE(check_pts(umat, this->pts_));

  // Rerun test when umat is already populated
  umat = UMat<TypeParam>(cv::Size(10, 10));
  umat = this->d_frame_;

  EXPECT_TRUE(check_pts(this->frame_, this->pts_));
  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TYPED_TEST(UMatTestFixture, Clone) {
  UMat<TypeParam> umat(this->frame_);
  auto umat_clone = umat.clone();

  EXPECT_TRUE(check_pts(umat_clone, this->pts_));

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

  umat.decrease_num_rows(umat.size().height);
  EXPECT_EQ(umat.frame().size(), umat.size());

  umat.decrease_num_rows(100);
  EXPECT_EQ(umat.frame().size(), cv::Size2i(imsize.width, 100));

  // Give a bad value and check that it throws
  EXPECT_THROW(umat.decrease_num_rows(101), std::runtime_error);
}
