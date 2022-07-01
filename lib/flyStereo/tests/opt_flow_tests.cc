
#ifdef WITH_VPI
#include "flyStereo/image_processing/vpi_backend.h"
#endif

#include "flyStereo/image_processing/cv_backend.h"
#include "flyStereo/image_processing/cv_cpu_backend.h"
#include "flyStereo/tests/generators.h"
#include "flyStereo/types/umat.h"
#include "gtest/gtest.h"
#include "opencv2/highgui.hpp"

constexpr int num_pts = 500;
const cv::Size imsize(1280, 720);
constexpr int window_size = 21;
constexpr int max_pyramid_level = 3;
constexpr int max_iters = 30;
constexpr bool use_initial_flow = false;

template <typename IpBackend>
class OptFlowTestFixture : public ::testing::Test {
 public:
  OptFlowTestFixture() {
    std::tie(prev_frame, curr_frame, prev_pts, curr_pts_gt) = generate_image_points(imsize, num_pts);
  }

  IpBackend::image_type prev_frame;
  IpBackend::image_type curr_frame;
  IpBackend::array_type prev_pts;
  IpBackend::array_type curr_pts;
  IpBackend::array_type curr_pts_gt;
};

using OptFlowTypes = ::testing::Types<CvBackend,
#ifdef WITH_VPI
                                      VpiBackend
#endif
                                      >;

TYPED_TEST_SUITE(OptFlowTestFixture, OptFlowTypes);

TYPED_TEST(OptFlowTestFixture, TestOptFlow) {
  // Initialize the optical flow object
  typename TypeParam::flow_type optflow(window_size, max_pyramid_level, max_iters, use_initial_flow);
  typename TypeParam::status_type status(cv::Size(num_pts, 1));

  optflow.calc(this->prev_frame, this->curr_frame, this->prev_pts, this->curr_pts, status);

  cv::imshow("prev", this->prev_frame.frame());
  cv::waitKey(0);
  cv::imshow("prev", this->curr_frame.frame());
  cv::waitKey(0);

  auto fail_count = 0;
  for (int i = 0; i < num_pts; i++) {
    if (status.frame()(i) == TypeParam::flow_type::success_value) {
      EXPECT_NEAR(this->curr_pts.frame()(i)[0], this->curr_pts_gt.frame()(i)[0], 1e-1);
      EXPECT_NEAR(this->curr_pts.frame()(i)[1], this->curr_pts_gt.frame()(i)[1], 1e-1);
    } else {
      fail_count++;
    }
  }

  std::cout << "fail count" << fail_count << std::endl;

  EXPECT_LT(static_cast<double>(fail_count) / num_pts, 0.1);
}
