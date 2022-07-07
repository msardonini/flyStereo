#pragma once
#include "flyStereo/types/umat.h"

template <typename E, typename StreamType, UMatDerivative image_type, UMatDerivative array_type,
          UMatDerivative status_type>
class OpticalFlowBase {
 public:
  using derived_type = E;

  derived_type& derived_cast() & noexcept;
  const derived_type& derived_cast() const& noexcept;
  derived_type derived_cast() && noexcept;

  void calc(const image_type& prev_image, const image_type& curr_image, const array_type& prev_pts,
            array_type& curr_pts, status_type& status, StreamType* stream) {
    return derived_cast().calc(prev_image, curr_image, prev_pts, curr_pts, status, stream);
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
