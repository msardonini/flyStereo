#pragma once

#include "flyStereo/types/umat.h"
#include "flyStereo/types/vpi_check.h"
#include "opencv2/core/core.hpp"
#include "vpi/Array.h"

constexpr auto umat_vpiarray_capacity = 5000;

/**
 * @brief UMat with extended functionality to support the VPIArray data type
 *
 */
template <typename T>
class UMatVpiArray : public UMat<T> {
 public:
  /**
   * @brief Construct a new UMatVpiArray object
   *
   */
  UMatVpiArray() = default;

  /**
   * @brief Construct a new UMatVpiArray object
   *
   * @param size The size of the array. Values are left uninitialized
   */
  UMatVpiArray(cv::Size size) : UMat<T>(size) { init(size); }

  UMatVpiArray(const UMatVpiArray& umat) : UMat<T>(umat) {
    init(umat.frame().size());
    if (!umat.frame().empty()) {
      umat.frame().copyTo(this->frame_);
    }
  }

  /**
   * @brief Construct a new UMatVpiArray object with the copy assignment operator
   *
   * @param umat
   * @return UMatVpiArray&
   */
  UMatVpiArray& operator=(const UMatVpiArray& umat) {
    UMat<T>::operator=(umat);
    init(umat.frame().size());
    if (!umat.frame().empty()) {
      umat.frame().copyTo(this->frame_);
    }
    return *this;
  }

  /**
   * @brief Construct a new UMatVpiArray object with the move constructor
   *
   * @param umat
   */
  UMatVpiArray(UMatVpiArray&& umat) : UMat<T>(std::move(umat)) {
    vpi_array_ = umat.vpi_array_;
    umat.vpi_array_ = nullptr;
    cols_ = umat.cols_;
    umat.cols_ = nullptr;
    set_capacity_ = umat.set_capacity_;
    umat.set_capacity_ = 0;
  }

  /**
   * @brief Construct a new UMatVpiArray with the move assignment operator
   *
   * @param umat The UMat to move
   * @return UMatVpiArray&
   */
  UMatVpiArray& operator=(UMatVpiArray&& umat) {
    UMat<T>::operator=(std::move(umat));
    vpi_array_ = umat.vpi_array_;
    umat.vpi_array_ = nullptr;
    if (cols_) {
      cudaFree(cols_);
    }
    cols_ = umat.cols_;
    umat.cols_ = nullptr;
    set_capacity_ = umat.set_capacity_;
    umat.set_capacity_ = 0;

    return *this;
  }

  /**
   * @brief Construct a new UMatVpiArray with a cv::cuda::GpuMat
   *
   * @param gpu_mat The input cv::cuda::GpuMat
   * @return UMatVpiArray&
   */
  UMatVpiArray& operator=(const cv::cuda::GpuMat& gpu_mat) {
    UMat<T>::operator=(gpu_mat);
    init(gpu_mat.size());
    return *this;
  }

  /**
   * @brief Construct a new UMatVpiArray with a cv::Mat
   *
   * @param mat The input cv::Mat
   * @return UMatVpiArray
   */
  UMatVpiArray& operator=(const cv::Mat& mat) {
    UMat<T>::operator=(mat);
    init(mat.size());
    return *this;
  }

  /**
   * @brief Construct a new UMatVpiArray with a UMat
   *
   * @param umat The input UMat
   * @return UMatVpiArray
   */
  UMatVpiArray(const UMat<T>& umat) : UMat<T>(umat) { init(umat.frame().size()); }

  /**
   * @brief Construct a new UMatVpiArray with a UMat using the move constructor
   *
   * @param umat The input UMat
   * @return UMatVpiArray
   */
  UMatVpiArray(UMat<T>&& umat) : UMat<T>(std::move(umat)) { init(this->size()); }

  /**
   * @brief Construct a new UMatVpiArray with a UMat
   *
   * @param umat The input UMat
   * @return UMatVpiArray
   */
  UMatVpiArray& operator=(const UMat<T>& umat) {
    UMat<T>::operator=(umat);
    init(umat.size());
    return *this;
  }

  /**
   * @brief Construct a new UMatVpiArray with a UMat using move semantics
   *
   * @param umat The input UMat
   * @return UMatVpiArray
   */
  UMatVpiArray& operator=(UMat<T>&& umat) {
    UMat<T>::operator=(std::move(umat));
    init(this->size());
    return *this;
  }
  /**
   * @brief Destroy the UMatVpiArray object
   *
   */
  ~UMatVpiArray() {
    if (vpi_array_) {
      vpiArrayDestroy(vpi_array_);
    }
    if (cols_ != nullptr) {
      cudaFree(cols_);
    }
  }

  /**
   * @brief Get the VPIArray object
   *
   * @return VPIArray& The VPIArray object
   */
  VPIArray& vpi_array() { return vpi_array_; }

  /**
   * @brief Get the VPIArray object
   *
   * @return const VPIArray&
   */
  const VPIArray& vpi_array() const { return vpi_array_; }

  void lock() const {
    if (vpi_array_) {
      check_status(vpiArrayLock(vpi_array_, VPI_LOCK_READ_WRITE));
    }
  }

  void unlock() const {
    if (vpi_array_) {
      check_status(vpiArrayUnlock(vpi_array_));
    }
  }

 protected:
  /**
   * @brief Initialize the UMatVpiArray object with the given size.
   *
   * @param size The size of the UMatVpiArray
   */
  void init(cv::Size size) {
    if (size.height != 1) {
      throw std::runtime_error("Only Arrays with rows == 1 are supported for UMatVpiArray");
    }

    if (size.width > umat_vpiarray_capacity) {
      throw std::runtime_error("Requested UMatVpiArray size is too large!");
    }
    // All arrays must have the same capacity. Reallocate the underlying data if needed
    if (!this->unified_ptr_ || this->set_capacity_ != umat_vpiarray_capacity) {
      void* old_ptr = this->unified_ptr_;
      cudaMallocManaged(&this->unified_ptr_, umat_vpiarray_capacity * sizeof(T));
      this->set_capacity_ = umat_vpiarray_capacity;

      // If there was data in our old buffer, copy it over and free it
      if (old_ptr) {
        std::copy(static_cast<T*>(old_ptr), static_cast<T*>(old_ptr) + this->size().width,
                  static_cast<T*>(this->unified_ptr_));
        cudaFree(old_ptr);
      }

      // Redefine the opencv frames to use the new pointer
      this->frame_ = cv::Mat_<T>(size.height, size.width, static_cast<T*>(this->unified_ptr_));
      this->d_frame_ = cv::cuda::GpuMat(size.height, size.width, this->get_type(), static_cast<T*>(this->unified_ptr_));
    }

    // Set cols_, the pointer to the size (number of utilized elements) of the array
    if (!this->cols_) {
      cudaMallocManaged(&cols_, sizeof(int));
    }
    *cols_ = size.width;

    VPIArrayData array_data = {};
    memset(&array_data, 0, sizeof(array_data));

    array_data.bufferType = VPI_ARRAY_BUFFER_CUDA_AOS;
    array_data.buffer.aos.data = this->unified_ptr_;
    array_data.buffer.aos.capacity = set_capacity_;
    array_data.buffer.aos.sizePointer = cols_;
    array_data.buffer.aos.strideBytes = sizeof(T);
    if constexpr (std::is_same_v<T, uint8_t>) {
      array_data.buffer.aos.type = VPI_ARRAY_TYPE_U8;
    } else if (std::is_same_v<T, cv::Vec2f>) {
      array_data.buffer.aos.type = VPI_ARRAY_TYPE_KEYPOINT;
    }

    if (vpi_array_) {
      check_status(vpiArraySetWrapper(vpi_array_, &array_data));
    } else {
      check_status(vpiArrayCreateWrapper(&array_data, VPI_BACKEND_CUDA | VPI_EXCLUSIVE_STREAM_ACCESS, &vpi_array_));
      lock();
    }
  }

  int* cols_ = nullptr;
  std::size_t set_capacity_ = 0;
  VPIArray vpi_array_ = nullptr;
};
