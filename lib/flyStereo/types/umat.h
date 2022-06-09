#pragma once

#include <concepts>
#include <cstdlib>
#include <iostream>

#include "cuda_runtime.h"
#include "flyStereo/types/vpi_check.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/cuda.hpp"

template <typename T>
concept MemoryHandler = requires(T t) {
  { t.malloc(std::size_t{}) }
  ->std::same_as<void *>;
  { t.free(std::declval<void *>()) }
  ->std::same_as<void>;
};

struct CudaMemAllocator {
  static void *malloc(std::size_t size) {
    void *ptr;
    check_status(cudaMallocManaged(&ptr, size));
    return ptr;
  }

  static void free(void *ptr) { check_status(cudaFree(ptr)); }
};

struct MemAllocator {
  static void *malloc(std::size_t size) { return std::malloc(size); }
  static void free(void *ptr) { std::free(ptr); }
};

/**
 * @brief Unified memory matrix class.
 *
 */
template <typename T, MemoryHandler MemoryHandlerT = CudaMemAllocator>
class UMat {
 public:
  using value_type = T;
  using memory_handler_type = MemoryHandlerT;

  /**
   * @brief Allow default constructor.
   *
   */
  UMat() = default;

  /**
   * @brief Construct a new UMat object using the copy constructor.
   *
   * @param umat
   */
  UMat(const UMat &umat) {
    init(umat.frame().size());
    if (!umat.frame().empty()) {
      umat.frame().copyTo(frame_);
    }
  }

  /**
   * @brief Construct a new UMat object using the move constructor.
   *
   * @param umat The UMat to move. UMat's cannot share data, this will no longer be valid after the move.
   */
  UMat(UMat &&umat) {
    frame_ = umat.frame_;
    d_frame_ = umat.d_frame_;
    unified_ptr_ = umat.unified_ptr_;
    umat.unified_ptr_ = nullptr;
    umat.frame_ = cv::Mat_<T>();
    umat.d_frame_ = cv::cuda::GpuMat();
  }

  /**
   * @brief Copy assignment operator
   *
   * @param umat UMat to copy from
   * @return UMat New UMat with the same data as the old one
   */
  UMat operator=(const UMat &umat) {
    if (unified_ptr_) {
      MemoryHandlerT::free(unified_ptr_);
    }
    init(umat.frame().size());
    if (!umat.frame().empty()) {
      umat.frame().copyTo(frame_);
    }
    return *this;
  }

  /**
   * @brief Move assignment operator
   *
   * @param umat The UMat to move. UMat's cannot share data, this will no longer be valid after the move.
   * @return UMat New UMat
   */
  UMat operator=(UMat &&umat) {
    if (unified_ptr_) {
      MemoryHandlerT::free(unified_ptr_);
    }
    unified_ptr_ = umat.unified_ptr_;
    frame_ = umat.frame_;
    d_frame_ = umat.d_frame_;
    umat.unified_ptr_ = nullptr;
    umat.frame_ = cv::Mat_<T>();
    umat.d_frame_ = cv::cuda::GpuMat();
    return *this;
  }

  /**
   * @brief Construct a new UMat object with unallocated memory
   *
   * @param frame_size The size of the frame
   */
  UMat(const cv::Size2i frame_size) { init(frame_size); }

  /**
   * @brief Construct a new UMat object with data from a cv::Mat
   *
   * @param frame UMat is consatructed with data from frame
   */
  UMat(const cv::Mat &frame) {
    init(frame.size());
    if (!frame.empty()) {
      frame.copyTo(frame_);
    }
  }

  /**
   * @brief Construct a new UMat object with data from a cv::cuda::GpuMat
   *
   * @param frame UMat is consatructed with data from frame
   */
  UMat(const cv::cuda::GpuMat &frame) {
    init(frame.size());
    if (!frame.empty()) {
      frame.copyTo(d_frame_);
    }
  }

  /**
   * @brief operator= for UMat objects from a cv::Mat
   *
   * @param frame The frame to copy data from
   * @return UMat The UMat object
   */
  UMat operator=(const cv::Mat &frame) {
    // Resize the frame if it is not the same size, reallocate memory if needed
    if (frame.size() != frame_.size()) {
      if (unified_ptr_ != nullptr) {
        MemoryHandlerT::free(unified_ptr_);
      }
      init(frame.size());
    }
    if (!frame.empty()) {
      frame.copyTo(frame_);
    }
    return *this;
  }

  /**
   * @brief operator= for UMat objects from a cv::cuda::GpuMat
   *
   * @param frame The frame to copy data from
   * @return UMat The UMat object
   */
  UMat operator=(const cv::cuda::GpuMat &frame) {
    // Resize the frame if it is not the same size, reallocate memory if needed
    if (frame.size() != frame_.size()) {
      if (unified_ptr_ != nullptr) {
        MemoryHandlerT::free(unified_ptr_);
      }
      init(frame.size());
    }
    if (!frame.empty()) {
      frame.copyTo(d_frame_);
    }
    return *this;
  }

  /**
   * @brief Destroy the UMat object
   *
   */
  ~UMat() {
    if (unified_ptr_ != nullptr) {
      MemoryHandlerT::free(unified_ptr_);
    }
  }

  /**
   * @brief Gets the frame for CPU based operations
   *
   * @return cv::Mat&
   */
  cv::Mat_<T> &frame() { return frame_; }

  /**
   * @brief Gets the frame for GPU based operations, const version
   *
   * @return const cv::Mat_<T>&
   */
  const cv::Mat_<T> &frame() const { return frame_; }

  /**
   * @brief Gets the frame for GPU based operations
   *
   * @return cv::cuda::GpuMat&
   */
  cv::cuda::GpuMat &d_frame() { return d_frame_; }

  /**
   * @brief Gets the frame for GPU based operations, const version
   *
   * @return const cv::cuda::GpuMat&
   */
  const cv::cuda::GpuMat &d_frame() const { return d_frame_; }

  /**
   * @brief Creates a copy of the UMat
   *
   * @return UMat<T>
   */
  UMat<T> clone() const { return UMat<T>(*this); }

  /**
   * @brief Decreases the number of rows in the UMat. The requsted number must be less than the current number of
   * rows, if not an exception is thrown. The memory is not reallocated, this is a fast operation.
   *
   * @param num_rows The number of rows to decrease the UMat to
   */
  void decrease_num_rows(int num_rows) {
    // Check to make sure the requested num rows is less than the current
    if (num_rows > frame_.rows) {
      throw std::runtime_error("Requested num rows is greater than the current num rows");
    } else if (num_rows == frame_.rows) {
      return;
    } else {
      frame_ = cv::Mat_<T>(num_rows, frame_.size().width, reinterpret_cast<T *>(unified_ptr_));
      d_frame_ = cv::cuda::GpuMat(num_rows, frame_.size().width, get_type(), reinterpret_cast<T *>(unified_ptr_));
    }
  }

  /**
   * @brief Decreases the number of cols in the UMat. The number of rows MUST be 1 for this operation to work. The
   * memory is not reallocated, this is a fast operation.
   *
   * @param num_cols The number of rows to decrease the UMat to
   */
  void decrease_num_cols(int num_cols) {
    // Check to make sure the requested num rows is less than the current
    if (frame_.rows != 1) {
      throw std::runtime_error("Requested num rows is not 1");
    } else if (num_cols > frame_.cols) {
      throw std::runtime_error("Requested num cols is greater than the current num cols");
    } else if (num_cols == frame_.cols) {
      return;
    } else {
      frame_ = cv::Mat_<T>(frame_.size().height, num_cols, reinterpret_cast<T *>(unified_ptr_));
      d_frame_ = cv::cuda::GpuMat(frame_.size().height, num_cols, get_type(), reinterpret_cast<T *>(unified_ptr_));
    }
  }
  void *data() const { return unified_ptr_; }

  /**
   * @brief Convienence function for getting the size of the UMat
   *
   * @return cv::Size
   */
  cv::Size size() const { return frame_.size(); }

 protected:
  /**
   * @brief Initializes the memory for the UMat object
   *
   * @param frame_size
   */
  void init(const cv::Size2i frame_size) {
    // Unified memory
    unified_ptr_ = MemoryHandlerT::malloc(frame_size.area() * sizeof(T));
    frame_ = cv::Mat_<T>(frame_size.height, frame_size.width, static_cast<T *>(unified_ptr_));
    d_frame_ = cv::cuda::GpuMat(frame_size.height, frame_size.width, get_type(), static_cast<T *>(unified_ptr_));
  }

  /**
   * @brief Get the openCV custom runtime type
   *
   * @return constexpr int The type corresponding to T
   */
  static constexpr int get_type() {
    if constexpr (std::is_same<T, uint8_t>::value) {
      return CV_8UC1;
    } else if constexpr (std::is_same<T, uint16_t>::value) {
      return CV_16UC1;
    } else if constexpr (std::is_same<T, int8_t>::value) {
      return CV_8SC1;
    } else if constexpr (std::is_same<T, int16_t>::value) {
      return CV_16SC1;
    } else if constexpr (std::is_same<T, int32_t>::value) {
      return CV_32SC1;
    } else if constexpr (std::is_same<T, float>::value) {
      return CV_32FC1;
    } else if constexpr (std::is_same<T, double>::value) {
      return CV_64FC1;
    } else if (std::is_same<T, cv::Vec<uint8_t, 2>>::value) {
      return CV_8UC2;
    } else if (std::is_same<T, cv::Vec<uint16_t, 2>>::value) {
      return CV_16UC2;
    } else if (std::is_same<T, cv::Vec<int8_t, 2>>::value) {
      return CV_8SC2;
    } else if (std::is_same<T, cv::Vec<int16_t, 2>>::value) {
      return CV_16SC2;
    } else if (std::is_same<T, cv::Vec<int32_t, 2>>::value) {
      return CV_32SC2;
    } else if (std::is_same<T, cv::Vec<float, 2>>::value) {
      return CV_32FC2;
    } else if (std::is_same<T, cv::Vec<double, 2>>::value) {
      return CV_64FC2;
    } else if (std::is_same<T, cv::Vec<uint8_t, 3>>::value) {
      return CV_8UC3;
    } else if (std::is_same<T, cv::Vec<uint16_t, 3>>::value) {
      return CV_16UC3;
    } else if (std::is_same<T, cv::Vec<int8_t, 3>>::value) {
      return CV_8SC3;
    } else if (std::is_same<T, cv::Vec<int16_t, 3>>::value) {
      return CV_16SC3;
    } else if (std::is_same<T, cv::Vec<int32_t, 3>>::value) {
      return CV_32SC3;
    } else if (std::is_same<T, cv::Vec<float, 3>>::value) {
      return CV_32FC3;
    } else if (std::is_same<T, cv::Vec<double, 3>>::value) {
      return CV_64FC3;
    } else {
      throw std::runtime_error("Unsupported type");
    }
  }

  cv::cuda::GpuMat d_frame_;     //< d_frame_, non-owning matrix
  cv::Mat_<T> frame_;            //< The frame_, non-ownding matrix
  void *unified_ptr_ = nullptr;  //< Pointer to the unified memory
};

/**
 * @brief Concept that defines the base class for all points classes supported by this library
 *
 * @tparam T The datatype that derives from UMat<cv::Vec2f>
 */
template <typename T>
concept UMatDerivative = std::derived_from<T, UMat<typename T::value_type, typename T::memory_handler_type>>;

/**
 * @brief Alias for CPU only umat. This type is useful for hardware without an nvidia gpu
 *
 * @tparam T
 */
template <typename T>
using UMatCpu = UMat<T, MemAllocator>;
