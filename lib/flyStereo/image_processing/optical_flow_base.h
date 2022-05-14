#pragma once
#include "flyStereo/umat.h"

template <typename E>
class OptFlowBase {
 public:
  using derived_type = E;

  derived_type& derived_cast() & noexcept;
  const derived_type& derived_cast() const& noexcept;
  derived_type derived_cast() && noexcept;

  void calc(const UMat<uint8_t>& prev_image, const UMat<uint8_t>& curr_image, const UMat<cv::Vec2f>& prev_pts,
            UMat<cv::Vec2f>& curr_pts) {
    return derived_cast().calc(prev_image, curr_image, prev_pts, curr_pts);
  }

 protected:
  OptFlowBase() = default;
  ~OptFlowBase() = default;

  OptFlowBase(const OptFlowBase&) = default;
  OptFlowBase& operator=(const OptFlowBase&) = default;

  OptFlowBase(OptFlowBase&&) = default;
  OptFlowBase& operator=(OptFlowBase&&) = default;
};

#include "flyStereo/image_processing/optical_flow_base.tpp"
