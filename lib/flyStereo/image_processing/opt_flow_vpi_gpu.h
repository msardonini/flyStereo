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

constexpr auto max_num_keypoints = 2000;

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

inline VPIImage umat_to_vpi_image(const UMat<uint8_t>& umat) {
  const cv::cuda::GpuMat& gpuMat = umat.d_frame();
  VPIImage image = NULL;
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
  check_status(vpiImageCreateWrapper(&img_data, nullptr, VPI_BACKEND_CUDA, &image));
  return image;
}

template <typename T>
inline VPIArray umat_to_vpi_array(const UMat<T>& umat) {
  const cv::cuda::GpuMat& gpuMat = umat.d_frame();
  VPIArray array = NULL;
  VPIArrayData array_data = {};
  memset(&array_data, 0, sizeof(array_data));

  int num_elements = gpuMat.cols;
  // int num_elements = 0;
  array_data.bufferType = VPI_ARRAY_BUFFER_CUDA_AOS;
  array_data.buffer.aos.capacity = num_elements;
  array_data.buffer.aos.data = gpuMat.data;
  array_data.buffer.aos.sizePointer = const_cast<int*>(umat.get_cols());
  array_data.buffer.aos.strideBytes = sizeof(T);
  if constexpr (std::is_same_v<T, uint8_t>) {
    array_data.buffer.aos.type = VPI_ARRAY_TYPE_U8;
  } else if (std::is_same_v<T, cv::Vec2f>) {
    array_data.buffer.aos.type = VPI_ARRAY_TYPE_KEYPOINT;
  }

  check_status(vpiArrayCreateWrapper(&array_data, VPI_BACKEND_CUDA, &array));
  return array;
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
    // lk_params_.epsilon = 0.01f;
  }

  ~OptFlowVpiGpu() {
    if (initialized_) {
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

    // Allocate memory for UMats
    prev_image_umat_ = UMat<uint8_t>(frame_size);
    prev_image_ = umat_to_vpi_image(prev_image_umat_);

    curr_image_umat_ = UMat<uint8_t>(frame_size);
    curr_image_ = umat_to_vpi_image(curr_image_umat_);

    prev_pts_umat_ = UMat<cv::Vec2f>(cv::Size(max_num_keypoints, 1));
    prev_pts_ = umat_to_vpi_array(prev_pts_umat_);

    curr_pts_umat_ = UMat<cv::Vec2f>(cv::Size(max_num_keypoints, 1));
    curr_pts_ = umat_to_vpi_array(curr_pts_umat_);

    status_umat_ = UMat<uint8_t>(cv::Size(max_num_keypoints, 1));
    status_ = umat_to_vpi_array(status_umat_);

    vpiCreateOpticalFlowPyrLK(VPI_BACKEND_CUDA, frame_size.width, frame_size.height, VPI_IMAGE_FORMAT_U8,
                              max_pyramid_level_, 0.5, &optflow_);

    check_status(vpiPyramidCreate(frame_size.width, frame_size.height, VPI_IMAGE_FORMAT_U8, max_pyramid_level_, 0.5, 0,
                                  &prev_pyramid_));
    check_status(vpiPyramidCreate(frame_size.width, frame_size.height, VPI_IMAGE_FORMAT_U8, max_pyramid_level_, 0.5, 0,
                                  &curr_pyramid_));
    initialized_ = true;
  }

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
    check_status(vpiArraySetSize(prev_pts_, prev_pts.d_frame().cols));
    check_status(vpiArraySetSize(curr_pts_, curr_pts.d_frame().cols));

    // Copy the input data to the UMats
    check_status(cudaMemcpy(prev_image_umat_.d_frame().data, prev_image.d_frame().data,
                            prev_image.d_frame().size().area() * sizeof(uint8_t), cudaMemcpyDeviceToDevice));
    check_status(cudaMemcpy(curr_image_umat_.d_frame().data, curr_image.d_frame().data,
                            curr_image.d_frame().size().area() * sizeof(uint8_t), cudaMemcpyDeviceToDevice));

    check_status(cudaMemcpy(prev_pts_umat_.d_frame().data, prev_pts.d_frame().data,
                            prev_pts.d_frame().size().area() * sizeof(cv::Vec2f), cudaMemcpyDeviceToDevice));
    check_status(cudaMemcpy(curr_pts_umat_.d_frame().data, curr_pts.d_frame().data,
                            curr_pts.d_frame().size().area() * sizeof(cv::Vec2f), cudaMemcpyDeviceToDevice));

    check_status(
        vpiSubmitGaussianPyramidGenerator(stream_, VPI_BACKEND_CUDA, prev_image_, prev_pyramid_, VPI_BORDER_CLAMP));
    check_status(
        vpiSubmitGaussianPyramidGenerator(stream_, VPI_BACKEND_CUDA, curr_image_, curr_pyramid_, VPI_BORDER_CLAMP));

    // auto destroy = std::chrono::high_resolution_clock::now();
    check_status(vpiSubmitOpticalFlowPyrLK(stream_, VPI_BACKEND_CUDA, optflow_, prev_pyramid_, curr_pyramid_, prev_pts_,
                                           curr_pts_, status_, &lk_params_));

    check_status(vpiStreamSync(stream_));

    check_status(cudaMemcpy(curr_pts.d_frame().data, curr_pts_umat_.d_frame().data,
                            curr_pts.d_frame().size().area() * sizeof(cv::Vec2f), cudaMemcpyDeviceToDevice));

    // Allocate new memory for status if it's not already sized properly
    if (status.d_frame().size() != prev_pts.d_frame().size()) {
      status = UMat<uint8_t>(cv::Size(prev_pts.d_frame().cols, 1));
    }
    check_status(cudaMemcpy(status.d_frame().data, status_umat_.d_frame().data,
                            status.d_frame().size().area() * sizeof(uint8_t), cudaMemcpyDeviceToDevice));

    // vpiArrayDestroy(prev_pts_array);
    // vpiArrayDestroy(curr_pts_array);
    // vpiArrayDestroy(status_vpi);

    // auto init_t = std::chrono::duration_cast<std::chrono::microseconds>(init - start).count();
    // auto calc_t = std::chrono::duration_cast<std::chrono::microseconds>(calc - init).count();
    // auto destroy_t = std::chrono::duration_cast<std::chrono::microseconds>(destroy - calc).count();

    // std::cout << "init: " << init_t << " calc: " << calc_t << " destroy: " << destroy_t << std::endl;
  }

  const static uint8_t success_value = 0;

 private:
  inline void gen_vpi_pyramid(const UMat<uint8_t>& image, VPIPyramid& pyr) {
    VPIImage curr_image_vpi = umat_to_vpi_image(image);
    check_status(vpiSubmitGaussianPyramidGenerator(stream_, VPI_BACKEND_CUDA, curr_image_vpi, pyr, VPI_BORDER_CLAMP));
  }

  bool initialized_;
  int window_size_;
  int max_pyramid_level_;
  int max_iters_;
  bool use_initial_flow_;
  VPIStream stream_ = NULL;

  // VPIPyramid pyr_prev_frame_;

  UMat<uint8_t> prev_image_umat_;
  VPIImage prev_image_;

  UMat<uint8_t> curr_image_umat_;
  VPIImage curr_image_;
  UMat<cv::Vec2f> prev_pts_umat_;
  VPIArray prev_pts_;
  UMat<cv::Vec2f> curr_pts_umat_;
  VPIArray curr_pts_;
  UMat<uint8_t> status_umat_;
  VPIArray status_;

  VPIPyramid prev_pyramid_;
  VPIPyramid curr_pyramid_;

  VPIOpticalFlowPyrLKParams lk_params_;
  VPIPayload optflow_;
};
