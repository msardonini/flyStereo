#pragma once

#include "flyStereo/image_processing/optical_flow/optical_flow_base.h"
#include "flyStereo/image_processing/streams/cv_cpu_stream.h"
#include "opencv2/cudaoptflow.hpp"
#include "opencv2/video/tracking.hpp"

class PyrLkCvCpu
    : public OpticalFlowBase<PyrLkCvCpu, CvStreamCpu, UMatCpu<uint8_t>, UMatCpu<cv::Vec2f>, UMatCpu<uint8_t>> {
 public:
  PyrLkCvCpu(int window_size = 21, int max_pyramid_level = 3, int max_iters = 30, bool use_initial_flow = false)
      : window_size_(window_size),
        max_pyramid_level_(max_pyramid_level),
        max_iters_(max_iters),
        use_initial_flow_(use_initial_flow) {}

  void init(cv::Size size) {}

  /**
   * @brief Convenience function to copy a cv::Mat into a UMatCpu, while checking for size. Our convention is to hold
   * points along the x axis (cols), but openCV is not very consistent about how they output points. We sometimes need
   * to make adjustments to the points to match our convention.
   *
   * @tparam U
   * @param src
   * @param dst
   * @param ref_size
   */
  template <UMatDerivative U>
  void copy_output(const cv::Mat& src, U& dst) {
    if (src.rows == 1 && src.cols >= 1) {
      dst = src;
    } else if (src.cols == 1 && src.rows > 1) {
      dst = src.t();
    } else {
      throw std::runtime_error("PyrLkCvCpu::calc: status_tmp.size() != ref_size");
    }
  }

  void calc(const UMatCpu<uint8_t>& prev_image, const UMatCpu<uint8_t>& curr_image, const UMatCpu<cv::Vec2f>& prev_pts,
            UMatCpu<cv::Vec2f>& curr_pts, UMatCpu<uint8_t>& status, CvStreamCpu* stream = nullptr) {
    cv::Mat pts_tmp;
    if (use_initial_flow_) {
      pts_tmp = prev_pts.frame();
    }
    cv::Mat status_tmp;
    cv::calcOpticalFlowPyrLK(prev_image.frame(), curr_image.frame(), prev_pts.frame(), pts_tmp, status_tmp,
                             cv::noArray(), cv::Size(window_size_, window_size_), max_pyramid_level_,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, max_iters_, 0.01),
                             (use_initial_flow_ ? cv::OPTFLOW_USE_INITIAL_FLOW : 0));
    copy_output(status_tmp, status);
    copy_output(pts_tmp, curr_pts);
  }

  const static uint8_t success_value = 1;

 private:
  int window_size_;
  int max_pyramid_level_;
  int max_iters_;
  bool use_initial_flow_;
};
