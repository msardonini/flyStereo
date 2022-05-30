#pragma once
#include "flyStereo/types/umat.h"

template <typename E, typename StreamType>
class OpticalFlowBase {
 public:
  using derived_type = E;

  derived_type& derived_cast() & noexcept;
  const derived_type& derived_cast() const& noexcept;
  derived_type derived_cast() && noexcept;

  void calc(const UMat<uint8_t>& prev_image, const UMat<uint8_t>& curr_image, const UMat<cv::Vec2f>& prev_pts,
            UMat<cv::Vec2f>& curr_pts, StreamType* stream) {
    return derived_cast().calc(prev_image, curr_image, prev_pts, curr_pts);
  }

 protected:
  OpticalFlowBase() = default;
  ~OpticalFlowBase() = default;

  OpticalFlowBase(const OpticalFlowBase&) = default;
  OpticalFlowBase& operator=(const OpticalFlowBase&) = default;

  OpticalFlowBase(OpticalFlowBase&&) = default;
  OpticalFlowBase& operator=(OpticalFlowBase&&) = default;
};

#include "flyStereo/image_processing/optical_flow/optical_flow_base.h"
