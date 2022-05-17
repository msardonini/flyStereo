#pragma once
#include <chrono>

#include "cuda_runtime.h"
#include "flyStereo/image_processing/optical_flow_base.h"
#include "vpi/Array.h"
#include "vpi/CUDAInterop.h"
#include "vpi/Image.h"
#include "vpi/Pyramid.h"
#include "vpi/algo/GaussianPyramid.h"
#include "vpi/algo/OpticalFlowPyrLK.h"

constexpr auto max_num_keypoints = 10000;

static inline void check_status(VPIStatus status) {
  if (status != VPI_SUCCESS) {
    throw std::runtime_error(vpiStatusGetName(status));
  }
}
static inline void check_status(cudaError status) {
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
}

inline void umat_to_vpi_image(const UMat<uint8_t>& umat, VPIImage& vpi_image) {
  const cv::cuda::GpuMat& gpuMat = umat.d_frame();
  VPIImageData img_data;
  memset(&img_data, 0, sizeof(img_data));

  img_data.bufferType = VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR;
  img_data.buffer.pitch.numPlanes = 1;
  img_data.buffer.pitch.format = VPI_IMAGE_FORMAT_U8;
  // img_data.buffer.pitch.format = vpiImageFormatSetDataType(img_data.buffer.pitch.format, VPI_DATA_TYPE_UNSIGNED);
  // img_data.buffer.pitch.format = vpiImageFormatSetMemLayout(img_data.buffer.pitch.format,
  // VPI_MEM_LAYOUT_PITCH_LINEAR);
  img_data.buffer.pitch.planes[0].width = gpuMat.cols;
  img_data.buffer.pitch.planes[0].height = gpuMat.rows;
  img_data.buffer.pitch.planes[0].pitchBytes = gpuMat.step;
  img_data.buffer.pitch.planes[0].data = gpuMat.data;
  img_data.buffer.pitch.planes[0].pixelType = VPI_PIXEL_TYPE_U8;

  if (vpi_image == nullptr) {
    check_status(vpiImageCreateWrapper(&img_data, nullptr, VPI_BACKEND_CUDA, &vpi_image));
  } else {
    check_status(vpiImageSetWrapper(vpi_image, &img_data));
  }
}

template <typename T>
inline void umat_to_vpi_array(const UMat<T>& umat, VPIArray& vpi_array) {
  const cv::cuda::GpuMat& gpuMat = umat.d_frame();

  VPIArrayData array_data = {};
  memset(&array_data, 0, sizeof(array_data));

  array_data.bufferType = VPI_ARRAY_BUFFER_CUDA_AOS;
  array_data.buffer.aos.capacity = max_num_keypoints;
  array_data.buffer.aos.data = gpuMat.data;
  array_data.buffer.aos.sizePointer = const_cast<int*>(umat.get_cols());
  array_data.buffer.aos.strideBytes = sizeof(T);
  if constexpr (std::is_same_v<T, uint8_t>) {
    array_data.buffer.aos.type = VPI_ARRAY_TYPE_U8;
  } else if (std::is_same_v<T, cv::Vec2f>) {
    array_data.buffer.aos.type = VPI_ARRAY_TYPE_KEYPOINT;
  }

  if (vpi_array == nullptr) {
    check_status(vpiArrayCreateWrapper(&array_data, VPI_BACKEND_CUDA, &vpi_array));
  } else {
    check_status(vpiArraySetWrapper(vpi_array, &array_data));
  }
}

class OptFlowVpiGpu : public OptFlowBase<OptFlowVpiGpu> {
 public:
  OptFlowVpiGpu() = default;

  OptFlowVpiGpu(int window_size, int max_pyramid_level, int max_iters, bool use_initial_flow)
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

  ~OptFlowVpiGpu() {
    if (initialized_) {
      vpiPayloadDestroy(optflow_);
      vpiPyramidDestroy(curr_pyramid_);
      vpiPyramidDestroy(prev_pyramid_);
      vpiArrayDestroy(status_);
      vpiArrayDestroy(curr_pts_);
      vpiArrayDestroy(prev_pts_);
      vpiImageDestroy(curr_image_);
      vpiImageDestroy(prev_image_);
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

  // void calc(const UMat<uint8_t>& prev_image, const UMat<uint8_t>& curr_image, const UMat<cv::Vec2f>& prev_pts,
  //           UMat<cv::Vec2f>& curr_pts, UMat<uint8_t>& status) {
  void calc(const UMat<uint8_t>& prev_image, const UMat<uint8_t>& curr_image, const UMat<cv::Vec2f>& prev_pts,
            UMat<cv::Vec2f>& curr_pts, UMat<uint8_t>& status) {
    // Sanity Checks
    if (prev_image.frame().size() != curr_image.frame().size()) {
      throw std::runtime_error("prev_image and curr_image must have the same size");
    } else if (use_initial_flow_ && prev_pts.frame().size() != curr_pts.frame().size()) {
      throw std::runtime_error("prev_pts and curr_pts must have the same size");
    }

    if (!initialized_) {
      init(prev_image.frame().size());
    }

    // If the user has not initialized status to the right size, do it now
    if (status.frame().size() != prev_pts.frame().size()) {
      status = UMat<uint8_t>(prev_pts.frame().size());
    }

    umat_to_vpi_image(prev_image, prev_image_);
    umat_to_vpi_image(curr_image, curr_image_);
    umat_to_vpi_array(prev_pts, prev_pts_);
    umat_to_vpi_array(curr_pts, curr_pts_);
    umat_to_vpi_array(status, status_);

    // Print first 10 points
    // std::cout << "Before PyrLK: " << std::endl;
    // for (int i = 0; i < 10; i++) {
    //   std::cout << "(" << prev_pts.frame()(i)[0] << ", " << prev_pts.frame()(i)[1] << ") -> (" <<
    //   curr_pts.frame()(i)[0]
    //             << ", " << curr_pts.frame()(i)[1] << ")" << std::endl;
    // }

    check_status(
        vpiSubmitGaussianPyramidGenerator(stream_, VPI_BACKEND_CUDA, prev_image_, prev_pyramid_, VPI_BORDER_ZERO));
    check_status(
        vpiSubmitGaussianPyramidGenerator(stream_, VPI_BACKEND_CUDA, curr_image_, curr_pyramid_, VPI_BORDER_ZERO));

    check_status(vpiSubmitOpticalFlowPyrLK(stream_, VPI_BACKEND_CUDA, optflow_, prev_pyramid_, curr_pyramid_, prev_pts_,
                                           curr_pts_, status_, &lk_params_));
    check_status(vpiStreamSync(stream_));

    // // Print first 10 points
    // std::cout << "After PyrLK: " << std::endl;
    // for (int i = 0; i < 10; i++) {
    //   std::cout << "(" << prev_pts.frame()(i)[0] << ", " << prev_pts.frame()(i)[1] << ") -> (" <<
    //   curr_pts.frame()(i)[0]
    //             << ", " << curr_pts.frame()(i)[1] << ")" << std::endl;
    // }
  }

  const static uint8_t success_value = 0;

 private:
  bool initialized_;
  int window_size_;
  int max_pyramid_level_;
  int max_iters_;
  bool use_initial_flow_;
  VPIStream stream_ = NULL;

  // VPIPyramid pyr_prev_frame_;

  VPIImage prev_image_ = nullptr;
  VPIImage curr_image_ = nullptr;
  VPIArray prev_pts_ = nullptr;
  VPIArray curr_pts_ = nullptr;
  VPIArray status_ = nullptr;

  VPIPyramid prev_pyramid_;
  VPIPyramid curr_pyramid_;

  VPIOpticalFlowPyrLKParams lk_params_;
  VPIPayload optflow_;
};
