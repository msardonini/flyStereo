
#include "flyStereo/image_processing/opt_flow_cv_gpu.h"
#include "flyStereo/image_processing/opt_flow_vpi_gpu.h"
#include "flyStereo/test/generators.h"
#include "flyStereo/umat.h"
#include "gtest/gtest.h"

constexpr int num_pts = 500;
const cv::Size imsize(1280, 720);
constexpr int window_size = 21;
constexpr int max_pyramid_level = 3;
constexpr int max_itres = 30;
constexpr bool use_initial_flow = true;

template <typename T>
class OptFlowTestFixture : public ::testing::Test {
 public:
  OptFlowTestFixture() {
    std::tie(prev_frame, curr_frame, prev_pts, curr_pts_gt) = generate_image_points(imsize, num_pts);

    // Set the current points to the previous points, so the optical flow alg can update them
    curr_pts = prev_pts;
  }

  UMat<uint8_t> prev_frame;
  UMat<uint8_t> curr_frame;
  UMat<cv::Vec2f> prev_pts;
  UMat<cv::Vec2f> curr_pts;
  UMat<cv::Vec2f> curr_pts_gt;
};

using OptFlowTypes = ::testing::Types<OptFlowVpiGpu, OptFlowCvGpu>;
TYPED_TEST_SUITE(OptFlowTestFixture, OptFlowTypes);

#include "opencv2/highgui.hpp"

TYPED_TEST(OptFlowTestFixture, TestOptFlow) {
  // Initialize the optical flow object
  TypeParam optflow(window_size, max_pyramid_level, max_itres, use_initial_flow);

  UMat<uint8_t> status(cv::Size(num_pts, 1));

  optflow.calc(this->prev_frame, this->curr_frame, this->prev_pts, this->curr_pts, status);

  auto fail_count = 0;
  for (int i = 0; i < num_pts; i++) {
    if (status.frame()(i) == TypeParam::success_value) {
      EXPECT_NEAR(this->curr_pts.frame()(i)[0], this->curr_pts_gt.frame()(i)[0], 1e-1);
      EXPECT_NEAR(this->curr_pts.frame()(i)[1], this->curr_pts_gt.frame()(i)[1], 1e-1);
    } else {
      fail_count++;
    }
  }
  EXPECT_LT(static_cast<double>(fail_count) / num_pts, 0.1);
}
