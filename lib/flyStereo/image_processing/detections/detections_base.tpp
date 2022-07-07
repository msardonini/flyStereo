#include "flyStereo/image_processing/detections/detections_base.h"

/**
 * Returns a reference to the actual derived type of the DetectionsBase
 */
template <typename E, typename StreamType, UMatDerivative ImageType, UMatDerivative ArrayType>
inline auto DetectionsBase<E, StreamType, ImageType, ArrayType>::derived_cast() & noexcept -> derived_type& {
  return *static_cast<derived_type*>(this);
}

/**
 * Returns a constant reference to the actual derived type of the DetectionsBase
 */
template <typename E, typename StreamType, UMatDerivative ImageType, UMatDerivative ArrayType>
inline auto DetectionsBase<E, StreamType, ImageType, ArrayType>::derived_cast() const& noexcept -> const derived_type& {
  return *static_cast<const derived_type*>(this);
}

/**
 * Returns a constant reference to the actual derived type of the DetectionsBase
 */
template <typename E, typename StreamType, UMatDerivative ImageType, UMatDerivative ArrayType>
inline auto DetectionsBase<E, StreamType, ImageType, ArrayType>::derived_cast() && noexcept -> derived_type {
  return *static_cast<derived_type*>(this);
}
