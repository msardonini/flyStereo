#pragma once

#include <opencv2/cudawarping.hpp>

#include "flyStereo/image_processing/optical_flow/optical_flow_base.h"
#include "flyStereo/image_processing/streams/cv_stream.h"
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaoptflow.hpp"

class PyrLkCvGpu : public OpticalFlowBase<PyrLkCvGpu, CvStream> {
 public:
  using stream_type = CvStream;

  PyrLkCvGpu(int window_size = 21, int max_pyramid_level = 3, int max_iters = 30, bool use_initial_flow = false) {
    opt_flow_ptr_ = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(window_size, window_size), max_pyramid_level,
                                                             max_iters, use_initial_flow);
  }

  void init(cv::Size size) {}

  void calc(const cv::cuda::GpuMat& prev_image, const cv::cuda::GpuMat& curr_image, const cv::cuda::GpuMat& prev_pts,
            cv::cuda::GpuMat& curr_pts, cv::cuda::GpuMat& status, CvStream* stream = nullptr) {
    if (opt_flow_ptr_) {
      if (stream) {
        opt_flow_ptr_->calc(prev_image, curr_image, prev_pts, curr_pts, status, cv::noArray(), stream->stream);
      } else {
        opt_flow_ptr_->calc(prev_image, curr_image, prev_pts, curr_pts, status, cv::noArray());
      }

    } else {
      throw std::runtime_error("PyrLkCvGpu::calc: opt_flow_ptr_ is null");
    }
  }

  void calc(const UMat<uint8_t>& prev_image, const UMat<uint8_t>& curr_image, const UMat<cv::Vec2f>& prev_pts,
            UMat<cv::Vec2f>& curr_pts, UMat<uint8_t>& status, CvStream* stream = nullptr) {
    cv::cuda::GpuMat status_tmp;
    cv::cuda::GpuMat pts_tmp;
    // auto tmp = curr_pts.d_frame();
    calc(prev_image.d_frame().clone(), curr_image.d_frame().clone(), prev_pts.d_frame(), curr_pts.d_frame(), status_tmp,
         stream);
    status = status_tmp;
  }

  const static uint8_t success_value = 1;

 private:
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> opt_flow_ptr_;
};
