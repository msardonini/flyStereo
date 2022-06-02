#pragma once
#include "flyStereo/types/umat.h"
#include "flyStereo/types/umat_vpiarray.h"
#include "flyStereo/types/umat_vpiimage.h"

template <typename E, typename StreamType, UMatDerivative ImageType, UMatDerivative ArrayType>
class DetectionsBase {
 public:
  using derived_type = E;

  derived_type& derived_cast() & noexcept;
  const derived_type& derived_cast() const& noexcept;
  derived_type derived_cast() && noexcept;

  void detect(const ImageType& image, const ImageType& mask, ArrayType& detections, StreamType* stream = nullptr) {
    return derived_cast().impl(image, mask, detections, stream);
  }

 protected:
  DetectionsBase() = default;
  ~DetectionsBase() = default;

  DetectionsBase(const DetectionsBase&) = default;
  DetectionsBase& operator=(const DetectionsBase&) = default;

  DetectionsBase(DetectionsBase&&) = default;
  DetectionsBase& operator=(DetectionsBase&&) = default;
};

#include "flyStereo/image_processing/detections/detections_base.tpp"
