#include "flyStereo/image_processing/optical_flow_base.h"

/**
 * Returns a reference to the actual derived type of the OptFlowBase
 */
template <class D>
inline auto OptFlowBase<D>::derived_cast() & noexcept -> derived_type& {
  return *static_cast<derived_type*>(this);
}

/**
 * Returns a constant reference to the actual derived type of the OptFlowBase
 */
template <class D>
inline auto OptFlowBase<D>::derived_cast() const& noexcept -> const derived_type& {
  return *static_cast<const derived_type*>(this);
}

/**
 * Returns a constant reference to the actual derived type of the OptFlowBase
 */
template <class D>
inline auto OptFlowBase<D>::derived_cast() && noexcept -> derived_type {
  return *static_cast<derived_type*>(this);
}
