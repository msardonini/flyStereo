#pragma once
#include <chrono>

#include "cuda_runtime.h"
#include "flyStereo/image_processing/optical_flow/optical_flow_base.h"
#include "flyStereo/image_processing/streams/vpi_stream.h"
#include "flyStereo/types/umat.h"
#include "flyStereo/types/umat_vpiarray.h"
#include "flyStereo/types/umat_vpiimage.h"
#include "flyStereo/types/vpi_check.h"
#include "vpi/Array.h"
#include "vpi/CUDAInterop.h"
#include "vpi/Image.h"
#include "vpi/Pyramid.h"
#include "vpi/algo/GaussianPyramid.h"
#include "vpi/algo/OpticalFlowPyrLK.h"

constexpr auto max_num_keypoints = 10000;

class PyrLkVpiGpu
    : public OpticalFlowBase<PyrLkVpiGpu, VpiStream, UMatVpiImage, UMatVpiArray<cv::Vec2f>, UMatVpiArray<uint8_t>> {
 public:
  PyrLkVpiGpu() = default;

  PyrLkVpiGpu(int window_size, int max_pyramid_level, int max_iters, bool use_initial_flow)
      : initialized_(false),
        window_size_(window_size),
        max_pyramid_level_(max_pyramid_level),
        max_iters_(max_iters),
        use_initial_flow_(use_initial_flow) {
    check_status(vpiInitOpticalFlowPyrLKParams(&lk_params_));
    lk_params_.useInitialFlow = use_initial_flow_;
    lk_params_.windowDimension = window_size_;
    lk_params_.numIterations = max_iters_;
    lk_params_.epsilon = 0.01f;
  }

  ~PyrLkVpiGpu() {
    if (initialized_) {
      vpiPayloadDestroy(optflow_);
      vpiPyramidDestroy(curr_pyramid_);
      vpiPyramidDestroy(prev_pyramid_);
      vpiStreamDestroy(stream_);
    }
  }

  void init(cv::Size frame_size) {
    // Create VPI stream
    check_status(vpiStreamCreate(VPI_BACKEND_CUDA, &stream_));

    vpiCreateOpticalFlowPyrLK(VPI_BACKEND_CUDA, frame_size.width, frame_size.height, VPI_IMAGE_FORMAT_U8,
                              max_pyramid_level_, 0.5, &optflow_);

    check_status(vpiPyramidCreate(frame_size.width, frame_size.height, VPI_IMAGE_FORMAT_U8, max_pyramid_level_, 0.5, 0,
                                  &prev_pyramid_));
    check_status(vpiPyramidCreate(frame_size.width, frame_size.height, VPI_IMAGE_FORMAT_U8, max_pyramid_level_, 0.5, 0,
                                  &curr_pyramid_));
    initialized_ = true;
  }

  void calc(UMatVpiImage& prev_image, UMatVpiImage& curr_image, UMatVpiArray<cv::Vec2f>& prev_pts,
            UMatVpiArray<cv::Vec2f>& curr_pts, UMatVpiArray<uint8_t>& status, VpiStream* stream = nullptr) {
    if (!initialized_) {
      init(prev_image.frame().size());
    }

    // If the user has not initialized status to the right size, do it now
    if (status.frame().size() != curr_pts.frame().size()) {
      status = UMatVpiArray<uint8_t>(curr_pts.frame().size());
    }

    prev_image.unlock();
    curr_image.unlock();
    prev_pts.unlock();
    curr_pts.unlock();
    status.unlock();

    // If the user has provided a stream, run asynchronously
    if (stream != nullptr && 0) {
      calc_opt_flow(stream->stream, prev_image, curr_image, prev_pts, curr_pts, status);
    } else {
      calc_opt_flow(stream_, prev_image, curr_image, prev_pts, curr_pts, status);
      check_status(vpiStreamSync(stream_));
    }
    prev_image.lock();
    curr_image.lock();
    prev_pts.lock();
    curr_pts.lock();
    status.lock();

    // // DEBUG
    // UMatVpiImage curr_image_tmp(curr_image);
    // curr_image = curr_image_tmp;
    // UMatVpiImage prev_image_tmp(prev_image);
    // prev_image = prev_image_tmp;
    // UMatVpiArray<cv::Vec2f> curr_pts_tmp(curr_pts);
    // curr_pts = curr_pts_tmp;
    // UMatVpiArray<cv::Vec2f> prev_pts_tmp(prev_pts);
    // prev_pts = prev_pts_tmp;
    // UMatVpiArray<uint8_t> status_tmp(status);
    // status = status_tmp;
  }

  void calc(const UMat<uint8_t>& prev_image, const UMat<uint8_t>& curr_image, const UMat<cv::Vec2f>& prev_pts,
            UMat<cv::Vec2f>& curr_pts, UMat<uint8_t>& status, VpiStream* stream = nullptr) {
    return calc(prev_image.d_frame(), curr_image.d_frame(), prev_pts, curr_pts, status, stream);
  }

  void calc(const cv::cuda::GpuMat& prev_image, const cv::cuda::GpuMat& curr_image, const UMat<cv::Vec2f>& prev_pts,
            UMat<cv::Vec2f>& curr_pts, UMat<uint8_t>& status, VpiStream* stream = nullptr) {
    // Sanity Checks
    if (prev_image.size() != curr_image.size()) {
      throw std::runtime_error("prev_image and curr_image must have the same size");
    } else if (use_initial_flow_ && prev_pts.frame().size() != curr_pts.frame().size()) {
      throw std::runtime_error("prev_pts and curr_pts must have the same size");
    }

    // auto prev_image_vpi = prev_image;
    // auto curr_image_vpi = curr_image;

    // Print first 10 points
    // std::cout << "Before PyrLK: " << std::endl;
    // for (int i = 0; i < 10; i++) {
    //   std::cout << "(" << prev_pts.frame()(i)[0] << ", " << prev_pts.frame()(i)[1] << ") -> (" <<
    //   curr_pts.frame()(i)[0]
    //             << ", " << curr_pts.frame()(i)[1] << ")" << std::endl;
    // }

    // Convert to VPI format
    // const UMatVpiImage prev_image_vpi(prev_image);
    // const UMatVpiImage curr_image_vpi(curr_image);
    // const UMatVpiArray<cv::Vec2f> prev_pts_vpi(prev_pts);
    // UMatVpiArray<cv::Vec2f> curr_pts_vpi(curr_pts);
    // UMatVpiArray<uint8_t> status_vpi(status);

    // calc(prev_image_vpi, curr_image_vpi, prev_pts_vpi, curr_pts_vpi, status_vpi, stream);

    // // Print first 10 points
    // std::cout << "After PyrLK: " << std::endl;
    // for (int i = 0; i < 10; i++) {
    //   std::cout << "(" << prev_pts.frame()(i)[0] << ", " << prev_pts.frame()(i)[1] << ") -> (" <<
    //   curr_pts.frame()(i)[0]
    //             << ", " << curr_pts.frame()(i)[1] << ")" << std::endl;
    // }

    // DEBUG destroy all vpi wrappers so we no longer require the inputs to be alive
    //   umat_to_vpi_image(prev_image, prev_image_);
    // umat_to_vpi_image(curr_image, curr_image_);
    // umat_to_vpi_array(prev_pts, prev_pts_);
    // umat_to_vpi_array(curr_pts, curr_pts_);
    // umat_to_vpi_array(status, status_);
  }

  const static uint8_t success_value = 0;

 private:
  void calc_opt_flow(VPIStream& stream, const UMatVpiImage& prev_image, const UMatVpiImage& curr_image,
                     const UMatVpiArray<cv::Vec2f>& prev_pts, UMatVpiArray<cv::Vec2f>& curr_pts,
                     UMatVpiArray<uint8_t>& status) {
    check_status(vpiSubmitGaussianPyramidGenerator(stream, VPI_BACKEND_CUDA, prev_image.vpi_frame(), prev_pyramid_,
                                                   VPI_BORDER_CLAMP));
    check_status(vpiSubmitGaussianPyramidGenerator(stream, VPI_BACKEND_CUDA, curr_image.vpi_frame(), curr_pyramid_,
                                                   VPI_BORDER_CLAMP));

    check_status(vpiSubmitOpticalFlowPyrLK(stream, VPI_BACKEND_CUDA, optflow_, prev_pyramid_, curr_pyramid_,
                                           prev_pts.vpi_array(), curr_pts.vpi_array(), status.vpi_array(),
                                           &lk_params_));
  }
  bool initialized_;
  int window_size_;
  int max_pyramid_level_;
  int max_iters_;
  bool use_initial_flow_;
  VPIStream stream_ = nullptr;

  // VPIPyramid pyr_prev_frame_;

  // VPIArray prev_pts_ = nullptr;
  // VPIArray curr_pts_ = nullptr;
  // VPIArray status_ = nullptr;

  VPIPyramid prev_pyramid_;
  VPIPyramid curr_pyramid_;

  VPIOpticalFlowPyrLKParams lk_params_;
  VPIPayload optflow_;
};
