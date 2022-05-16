#pragma once

#include <opencv2/cudawarping.hpp>

#include "flyStereo/image_processing/optical_flow_base.h"
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaoptflow.hpp"

class OptFlowCvGpu : public OptFlowBase<OptFlowCvGpu> {
 public:
  OptFlowCvGpu(int window_size = 21, int max_pyramid_level = 3, int max_iters = 30, bool use_initial_flow = false) {
    opt_flow_ptr_ = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(window_size, window_size), max_pyramid_level,
                                                             max_iters, use_initial_flow);
  }

  void init(cv::Size size) {}

  void calc(const UMat<uint8_t>& prev_image, const UMat<uint8_t>& curr_image, const UMat<cv::Vec2f>& prev_pts,
            UMat<cv::Vec2f>& curr_pts, UMat<uint8_t>& status) {
    if (opt_flow_ptr_) {
      cv::cuda::GpuMat status_tmp;
      opt_flow_ptr_->calc(prev_image.d_frame().clone(), curr_image.d_frame().clone(), prev_pts.d_frame(),
                          curr_pts.d_frame(), status_tmp);
      status = status_tmp;
    } else {
      throw std::runtime_error("OptFlowCvGpu::calc: opt_flow_ptr_ is null");
    }
  }

  const static uint8_t success_value = 1;

 private:
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> opt_flow_ptr_;
};