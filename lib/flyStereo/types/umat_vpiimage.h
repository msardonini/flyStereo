#pragma once
#include "flyStereo/types/umat.h"
#include "flyStereo/types/vpi_check.h"
#include "opencv2/core/core.hpp"
#include "vpi/Image.h"

/**
 * @brief UMat with extended functionality to support the VPIImage data type
 *
 */
class UMatVpiImage : public UMat<uint8_t> {
 public:
  /**
   * @brief Construct a new UMatVpiImage object
   *
   */
  UMatVpiImage() = default;

  /**
   * @brief Construct a new UMatVpiImage object
   *
   * @param size The size of the array. Values are left uninitialized
   */
  UMatVpiImage(cv::Size size) : UMat<uint8_t>(size) { init(size); }

  UMatVpiImage(const UMatVpiImage& umat) : UMat<uint8_t>(umat) {
    init(umat.frame().size());
    if (!umat.frame().empty()) {
      umat.frame().copyTo(frame_);
    }
  }

  /**
   * @brief Construct a new UMatVpiImage object with the copy assignment operator
   *
   * @param umat
   * @return UMatVpiImage&
   */
  UMatVpiImage& operator=(const UMatVpiImage& umat) {
    UMat<uint8_t>::operator=(umat);
    init(umat.frame().size());
    if (!umat.frame().empty()) {
      umat.frame().copyTo(frame_);
    }
    return *this;
  }

  /**
   * @brief Construct a new UMatVpiImage object with the move constructor
   *
   * @param umat
   */
  UMatVpiImage(UMatVpiImage&& umat) : UMat<uint8_t>(std::move(umat)) {
    vpi_frame_ = umat.vpi_frame_;
    umat.vpi_frame_ = nullptr;
  }

  /**
   * @brief Construct a new UMatVpiImage with the move assignment operator
   *
   * @param umat The UMat to move
   * @return UMatVpiImage&
   */
  UMatVpiImage& operator=(UMatVpiImage&& umat) {
    UMat<uint8_t>::operator=(std::move(umat));
    vpi_frame_ = umat.vpi_frame_;
    umat.vpi_frame_ = nullptr;
    return *this;
  }

  /**
   * @brief Construct a new UMatVpuImage with a cv::cuda::GpuMat
   *
   * @param gpu_mat The input cv::cuda::GpuMat
   * @return UMatVpiImage&
   */
  UMatVpiImage& operator=(const cv::cuda::GpuMat& gpu_mat) {
    UMat<uint8_t>::operator=(gpu_mat);
    init(gpu_mat.size());
    return *this;
  }

  /**
   * @brief Construct a new UMatVpuImage with a cv::Mat
   *
   * @param mat The input cv::Mat
   * @return UMatVpiImage
   */
  UMatVpiImage operator=(const cv::Mat& mat) {
    UMat<uint8_t>::operator=(mat);
    init(mat.size());
    return *this;
  }

  /**
   * @brief Construct a new UMatVpuImage with a UMat
   *
   * @param image The input UMat<uint8_t>
   * @return UMatVpiImage
   */
  UMatVpiImage(const UMat<uint8_t>& umat) : UMat<uint8_t>(umat) { init(umat.size()); }

  /**
   * @brief Construct a new UMatVpuImage with a UMat with the move constructor
   *
   * @param image The input UMat<uint8_t>
   * @return UMatVpiImage
   */
  UMatVpiImage(UMat<uint8_t>&& umat) : UMat<uint8_t>(std::move(umat)) { init(this->size()); }

  /**
   * @brief Construct a new UMatVpuImage with a UMat
   *
   * @param image The input vpi::Image
   * @return UMatVpiImage
   */
  UMatVpiImage& operator=(const UMat<uint8_t>& umat) {
    UMat<uint8_t>::operator=(umat);
    init(umat.size());
    return *this;
  }

  /**
   * @brief Construct a new UMatVpuImage with a UMat using move semantics
   *
   * @param image The input vpi::Image
   * @return UMatVpiImage
   */
  UMatVpiImage& operator=(UMat<uint8_t>&& umat) {
    UMat<uint8_t>::operator=(std::move(umat));
    init(this->size());
    return *this;
  }
  /**
   * @brief Destroy the UMatVpiImag1e object
   *
   */
  ~UMatVpiImage() {
    if (vpi_frame_) {
      vpiImageDestroy(vpi_frame_);
    }
  }

  /**
   * @brief Get the VPIImage object
   *
   * @return vpiImage& The VPIImage object
   */
  VPIImage& vpi_frame() { return vpi_frame_; }

  /**
   * @brief Get the VPIImage object
   *
   * @return const VPIImage&
   */
  const VPIImage& vpi_frame() const { return vpi_frame_; }

  void lock() const {
    if (vpi_frame_) {
      vpiImageLock(vpi_frame_, VPI_LOCK_READ_WRITE);
    }
  }

  void unlock() const {
    if (vpi_frame_) {
      vpiImageUnlock(vpi_frame_);
    }
  }

 protected:
  /**
   * @brief Initialize the UMatVpiImage object with the given size.
   *
   * @param size The size of the UMatVpiImage
   */
  void init(cv::Size size) {
    VPIImageData img_data;
    memset(&img_data, 0, sizeof(img_data));

    img_data.bufferType = VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR;
    img_data.buffer.pitch.numPlanes = d_frame().channels();
    img_data.buffer.pitch.format = VPI_IMAGE_FORMAT_U8;
    img_data.buffer.pitch.planes[0].width = size.width;
    img_data.buffer.pitch.planes[0].height = size.height;
    img_data.buffer.pitch.planes[0].pitchBytes = d_frame().step;
    img_data.buffer.pitch.planes[0].data = d_frame().data;
    img_data.buffer.pitch.planes[0].pixelType = VPI_PIXEL_TYPE_U8;

    if (vpi_frame_) {
      // If a vpiImage already exists, check if it's the same size for reuse. Otherwise, destroy it.
      int width, height;
      vpiImageGetSize(vpi_frame_, &width, &height);
      if (width != size.width || height != size.height) {
        vpiImageDestroy(vpi_frame_);
        vpi_frame_ = nullptr;
      } else {
        // Note: this function is much faster than vpiImageCreateWrapper
        check_status(vpiImageSetWrapper(vpi_frame_, &img_data));
        return;
      }
    }

    check_status(
        vpiImageCreateWrapper(&img_data, nullptr, VPI_BACKEND_CUDA | VPI_EXCLUSIVE_STREAM_ACCESS, &vpi_frame_));
    lock();
  }

  VPIImage vpi_frame_ = nullptr;
};
