#include <utility>
#include <vector>

#include "flyStereo/tests/umat/umat_test_helpers.h"
#include "flyStereo/types/umat_vpiimage.h"
#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"

constexpr int num_pts = 600;
const cv::Size2i imsize(1280, 500);

class UMatVpiImageTestFixture : public ::testing::Test {
 public:
  UMatVpiImageTestFixture() {
    // Generate random points
    cv::Mat_<uint8_t> image;
    generate_array(num_pts, imsize, image, pts_);
    frame_ = image;
  }

 protected:
  std::unordered_map<cv::Point2i, uint8_t> pts_;
  UMat<uint8_t> frame_;
};

TEST_F(UMatVpiImageTestFixture, UMatCopyConstructor) {
  UMatVpiImage umat(this->frame_);

  EXPECT_TRUE(check_pts(this->frame_, this->pts_));
  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TEST_F(UMatVpiImageTestFixture, UMatMoveConstructor) {
  UMatVpiImage umat(std::move(this->frame_));

  EXPECT_EQ(this->frame_.size(), cv::Size2i(0, 0));
  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TEST_F(UMatVpiImageTestFixture, SizeConstructor) {
  UMatVpiImage umat(imsize);

  EXPECT_EQ(umat.frame().size(), imsize);
}

TEST_F(UMatVpiImageTestFixture, CopyConstructor) {
  UMatVpiImage umat(this->frame_);
  UMatVpiImage umat_copy(umat);

  EXPECT_TRUE(check_pts(umat, this->pts_));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));
}

TEST_F(UMatVpiImageTestFixture, MoveConstructor) {
  UMatVpiImage umat(this->frame_);
  UMatVpiImage umat_copy(std::move(umat));

  EXPECT_EQ(umat.frame().size(), cv::Size2i(0, 0));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));
}

TEST_F(UMatVpiImageTestFixture, CopyAssignmentOperator) {
  UMatVpiImage umat(this->frame_);
  UMatVpiImage umat_copy(imsize);

  umat_copy = umat;

  EXPECT_TRUE(check_pts(umat, this->pts_));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));
}

TEST_F(UMatVpiImageTestFixture, MoveAssignmentOperator) {
  UMatVpiImage umat(this->frame_);
  UMatVpiImage umat_copy(imsize);

  umat_copy = std::move(umat);

  EXPECT_EQ(umat.frame().size(), cv::Size2i(0, 0));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));
}

TEST_F(UMatVpiImageTestFixture, MatAssignmentOperator) {
  UMatVpiImage umat;
  umat = this->frame_.frame();

  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TEST_F(UMatVpiImageTestFixture, GpuMatAssignmentOperator) {
  UMatVpiImage umat;
  umat = this->frame_.d_frame();

  EXPECT_TRUE(check_pts(umat, this->pts_));
}

TEST_F(UMatVpiImageTestFixture, ReAllocationTest) {
  UMatVpiImage umat(this->frame_);
  EXPECT_TRUE(check_pts(umat, this->pts_));

  auto new_imsize = imsize + cv::Size2i(10, 10);
  auto new_num_pts = num_pts + 10;
  std::unordered_map<cv::Point2i, uint8_t> new_pts;
  cv::Mat_<uint8_t> new_image;
  generate_array(new_num_pts, new_imsize, new_image, new_pts);

  umat = new_image;
  EXPECT_TRUE(check_pts(umat, new_pts));
}
