#include "flyStereo/image_processing/opt_flow_stream_base.h"

/**
 * Returns a reference to the actual derived type of the OptFlowStreamBase
 */
template <class D>
inline auto OptFlowStreamBase<D>::derived_cast() & noexcept -> derived_type& {
  return *static_cast<derived_type*>(this);
}

/**
 * Returns a constant reference to the actual derived type of the OptFlowStreamBase
 */
template <class D>
inline auto OptFlowStreamBase<D>::derived_cast() const& noexcept -> const derived_type& {
  return *static_cast<const derived_type*>(this);
}

/**
 * Returns a constant reference to the actual derived type of the OptFlowStreamBase
 */
template <class D>
inline auto OptFlowStreamBase<D>::derived_cast() && noexcept -> derived_type {
  return *static_cast<derived_type*>(this);
}
