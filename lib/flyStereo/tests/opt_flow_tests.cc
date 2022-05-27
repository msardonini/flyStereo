
#include "flyStereo/image_processing/opt_flow_cv_gpu.h"
#include "flyStereo/image_processing/opt_flow_vpi_gpu.h"
#include "flyStereo/tests/generators.h"
#include "flyStereo/types/umat.h"
#include "gtest/gtest.h"

constexpr int num_pts = 500;
const cv::Size imsize(1280, 720);
constexpr int window_size = 21;
constexpr int max_pyramid_level = 3;
constexpr int max_iters = 30;
constexpr bool use_initial_flow = false;

template <typename T>
class OptFlowTestFixture : public ::testing::Test {
 public:
  OptFlowTestFixture() {
    std::tie(prev_frame, curr_frame, prev_pts, curr_pts_gt) = generate_image_points(imsize, num_pts);

    // Set the current points to the previous points, so the optical flow alg can update them
    curr_pts = UMat<cv::Vec2f>(prev_pts);
  }

  UMat<uint8_t> prev_frame;
  UMat<uint8_t> curr_frame;
  UMat<cv::Vec2f> prev_pts;
  UMat<cv::Vec2f> curr_pts;
  UMat<cv::Vec2f> curr_pts_gt;
};

#ifdef WITH_VPI
using OptFlowTypes = ::testing::Types<OptFlowVpiGpu, OptFlowCvGpu>;
#else
using OptFlowTypes = ::testing::Types<OptFlowCvGpu>;
#endif
TYPED_TEST_SUITE(OptFlowTestFixture, OptFlowTypes);

#include "opencv2/highgui.hpp"

TYPED_TEST(OptFlowTestFixture, TestOptFlow) {
  std::cout << "adress_prev: " << (void*)this->prev_frame.d_frame().data << std::endl;
  std::cout << "address_curr " << (void*)this->curr_frame.d_frame().data << std::endl;
  // Initialize the optical flow object
  TypeParam optflow(window_size, max_pyramid_level, max_iters, use_initial_flow);

  UMat<uint8_t> status(cv::Size(num_pts, 1));

  optflow.calc(this->prev_frame, this->curr_frame, this->prev_pts, this->curr_pts, status);

  cv::imshow("prev", this->prev_frame.frame());
  cv::waitKey(0);
  cv::imshow("prev", this->curr_frame.frame());
  cv::waitKey(0);

  auto fail_count = 0;
  for (int i = 0; i < num_pts; i++) {
    if (status.frame()(i) == TypeParam::success_value) {
      EXPECT_NEAR(this->curr_pts.frame()(i)[0], this->curr_pts_gt.frame()(i)[0], 1e-1);
      EXPECT_NEAR(this->curr_pts.frame()(i)[1], this->curr_pts_gt.frame()(i)[1], 1e-1);
    } else {
      fail_count++;
    }
  }

  std::cout << "fail count" << fail_count << std::endl;

  EXPECT_LT(static_cast<double>(fail_count) / num_pts, 0.1);
}
