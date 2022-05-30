#include <utility>
#include <vector>

#include "flyStereo/tests/umat/umat_test_helpers.h"
#include "flyStereo/types/umat_vpiarray.h"
#include "flyStereo/types/vpi_check.h"
#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"
#include "vpi/Array.h"
#include "vpi/Stream.h"

constexpr int num_pts = 600;
const cv::Size2i imsize(1280, 1);

namespace {

template <typename T>
std::array<uint32_t, 10> vpi_test_array(const UMatVpiArray<T>& image) {}
}  // namespace

template <typename T>
class UMatVpiArrayTestFixture : public ::testing::Test {
 public:
  UMatVpiArrayTestFixture() {
    // Generate random points
    cv::Mat_<T> image;
    generate_array(num_pts, imsize, image, pts_);
    frame_ = image;
  }

 protected:
  std::unordered_map<cv::Point2i, T> pts_;
  UMat<T> frame_;
  std::array<uint32_t, 10> vpi_hist_;
};

using UMatVpiArrayTypes = ::testing::Types<uint8_t, cv::Vec2f>;
TYPED_TEST_SUITE(UMatVpiArrayTestFixture, UMatVpiArrayTypes);

TYPED_TEST(UMatVpiArrayTestFixture, DefaultConstructor) {
  UMatVpiArray<TypeParam> umat;
  EXPECT_EQ(umat.frame().size(), cv::Size2i(0, 0));
}

TYPED_TEST(UMatVpiArrayTestFixture, SizeConstructor) {
  UMatVpiArray<TypeParam> umat(imsize);

  EXPECT_EQ(umat.frame().size(), imsize);
}

TYPED_TEST(UMatVpiArrayTestFixture, CopyConstructor) {
  UMatVpiArray<TypeParam> umat(this->frame_);
  UMatVpiArray<TypeParam> umat_copy(umat);

  EXPECT_TRUE(check_pts(umat, this->pts_));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));
}

TYPED_TEST(UMatVpiArrayTestFixture, MoveConstructor) {
  UMatVpiArray<TypeParam> umat(this->frame_);
  UMatVpiArray<TypeParam> umat_copy(std::move(umat));

  EXPECT_EQ(umat.frame().size(), cv::Size2i(0, 0));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));
}

TYPED_TEST(UMatVpiArrayTestFixture, CopyAssignmentOperator) {
  UMatVpiArray<TypeParam> umat(this->frame_);
  UMatVpiArray<TypeParam> umat_copy(imsize);

  umat_copy = umat;

  EXPECT_TRUE(check_pts(umat, this->pts_));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));
}

TYPED_TEST(UMatVpiArrayTestFixture, MoveAssignmentOperator) {
  UMatVpiArray<TypeParam> umat(this->frame_);
  UMatVpiArray<TypeParam> umat_copy(imsize);

  umat_copy = std::move(umat);

  EXPECT_EQ(umat.frame().size(), cv::Size2i(0, 0));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));
}

TYPED_TEST(UMatVpiArrayTestFixture, UMatCopyConstructor) {
  UMatVpiArray<TypeParam> umat(this->frame_);

  EXPECT_TRUE(check_pts(this->frame_, this->pts_));
  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TYPED_TEST(UMatVpiArrayTestFixture, UMatMoveConstructor) {
  UMatVpiArray<TypeParam> umat(std::move(this->frame_));

  EXPECT_EQ(this->frame_.size(), cv::Size2i(0, 0));
  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TYPED_TEST(UMatVpiArrayTestFixture, MatAssignmentOperator) {
  UMatVpiArray<TypeParam> umat;
  umat = this->frame_.frame();

  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TYPED_TEST(UMatVpiArrayTestFixture, GpuMatAssignmentOperator) {
  UMatVpiArray<TypeParam> umat;
  umat = this->frame_.d_frame();

  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TYPED_TEST(UMatVpiArrayTestFixture, ReAllocationTest) {
  UMatVpiArray<TypeParam> umat(this->frame_);
  EXPECT_TRUE(check_pts(umat, this->pts_));

  auto new_imsize = imsize + cv::Size2i(10, 0);
  auto new_num_pts = num_pts + 10;
  std::unordered_map<cv::Point2i, TypeParam> new_pts;
  cv::Mat_<TypeParam> new_image;
  generate_array(new_num_pts, new_imsize, new_image, new_pts);

  umat = new_image;
  EXPECT_TRUE(check_pts(umat, new_pts));
}
