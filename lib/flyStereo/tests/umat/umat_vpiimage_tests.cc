#include <utility>
#include <vector>

#include "flyStereo/tests/umat/umat_test_helpers.h"
#include "flyStereo/types/umat_vpiimage.h"
#include "flyStereo/types/vpi_check.h"
#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"
#include "vpi/Array.h"
#include "vpi/Stream.h"
#include "vpi/algo/Histogram.h"

constexpr int num_pts = 600;
const cv::Size2i imsize(1280, 500);

namespace {
/**
 * @brief Uses the VPI backend to calculate a 10-bin histogram of the given image. This validates the VPIImage's use in
 * vpi API functions
 *
 * @param image The image to calculate the histogram of
 * @return std::array<uint32_t, 10> The 10-bin histogram of the image
 */
std::array<uint32_t, 10> generate_vpi_histogram(const UMatVpiImage& image) {
  const int min_val = 0;
  const int max_val = 255;
  const int num_bins = 10;

  image.unlock();

  VPIStream stream;
  vpiStreamCreate(0, &stream);

  VPIPayload payload;
  check_status(vpiCreateHistogramEven(VPI_BACKEND_CUDA, VPI_IMAGE_FORMAT_U8, min_val, max_val, num_bins, &payload));

  VPIArray output_array;
  check_status(vpiArrayCreate(num_bins, VPI_ARRAY_TYPE_U32, 0, &output_array));

  check_status(vpiSubmitHistogram(stream, 0, payload, image.vpi_frame(), output_array, 0));
  check_status(vpiStreamSync(stream));

  VPIArrayData array_data;
  check_status(vpiArrayLockData(output_array, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &array_data));

  std::array<uint32_t, 10> hist;
  auto data_ptr = reinterpret_cast<int*>(array_data.buffer.aos.data);
  std::for_each(hist.begin(), hist.end(), [&data_ptr](uint32_t& val) { val = *data_ptr++; });

  check_status(vpiArrayUnlock(output_array));

  vpiArrayDestroy(output_array);
  vpiPayloadDestroy(payload);
  vpiStreamDestroy(stream);

  image.lock();

  return hist;
}
}  // namespace

class UMatVpiImageTestFixture : public ::testing::Test {
 public:
  UMatVpiImageTestFixture() {
    // Generate random points
    cv::Mat_<uint8_t> image;
    generate_array(num_pts, imsize, image, pts_);
    frame_ = image;

    UMatVpiImage umat_vpi(frame_);
    vpi_hist_ = generate_vpi_histogram(umat_vpi);
  }

  /**
   * @brief Calculate the histogram of the given image using the VPI backend, and compare it to the expected histogram
   * values
   *
   * @param umat_vpi Image to calculate and check
   */
  void calc_and_check_histogram(const UMatVpiImage& umat_vpi) {
    const auto vpi_hist = generate_vpi_histogram(umat_vpi);

    for (int i = 0; i < vpi_hist_.size(); ++i) {
      EXPECT_EQ(vpi_hist[i], vpi_hist_[i]);
    }
  }

 protected:
  std::unordered_map<cv::Point2i, uint8_t> pts_;
  UMat<uint8_t> frame_;
  std::array<uint32_t, 10> vpi_hist_;
};

TEST_F(UMatVpiImageTestFixture, UMatCopyConstructor) {
  UMatVpiImage umat(this->frame_);

  EXPECT_TRUE(check_pts(this->frame_, this->pts_));
  EXPECT_TRUE(check_pts(umat, this->pts_));

  this->calc_and_check_histogram(umat);
}

TEST_F(UMatVpiImageTestFixture, UMatMoveConstructor) {
  UMatVpiImage umat(std::move(this->frame_));

  EXPECT_EQ(this->frame_.size(), cv::Size2i(0, 0));
  EXPECT_TRUE(check_pts(umat, this->pts_));

  this->calc_and_check_histogram(umat);
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

  this->calc_and_check_histogram(umat_copy);
}

TEST_F(UMatVpiImageTestFixture, MoveConstructor) {
  UMatVpiImage umat(this->frame_);
  UMatVpiImage umat_copy(std::move(umat));

  EXPECT_EQ(umat.frame().size(), cv::Size2i(0, 0));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));

  this->calc_and_check_histogram(umat_copy);
}

TEST_F(UMatVpiImageTestFixture, CopyAssignmentOperator) {
  UMatVpiImage umat(this->frame_);
  UMatVpiImage umat_copy(imsize);

  umat_copy = umat;

  EXPECT_TRUE(check_pts(umat, this->pts_));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));

  this->calc_and_check_histogram(umat_copy);
}

TEST_F(UMatVpiImageTestFixture, MoveAssignmentOperator) {
  UMatVpiImage umat(this->frame_);
  UMatVpiImage umat_copy(imsize);

  umat_copy = std::move(umat);

  EXPECT_EQ(umat.frame().size(), cv::Size2i(0, 0));
  EXPECT_TRUE(check_pts(umat_copy, this->pts_));

  this->calc_and_check_histogram(umat_copy);
}

TEST_F(UMatVpiImageTestFixture, MatAssignmentOperator) {
  UMatVpiImage umat;
  umat = this->frame_.frame();

  EXPECT_TRUE(check_pts(umat, this->pts_));

  this->calc_and_check_histogram(umat);
}

TEST_F(UMatVpiImageTestFixture, GpuMatAssignmentOperator) {
  UMatVpiImage umat;
  umat = this->frame_.d_frame();

  EXPECT_TRUE(check_pts(umat, this->pts_));

  this->calc_and_check_histogram(umat);
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
