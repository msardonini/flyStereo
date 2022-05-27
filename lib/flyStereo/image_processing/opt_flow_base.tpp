#include "flyStereo/image_processing/opt_flow_base.h"

/**
 * Returns a reference to the actual derived type of the OptFlowBase
 */
template <typename E, typename StreamType>
inline auto OptFlowBase<E, StreamType>::derived_cast() & noexcept -> derived_type& {
  return *static_cast<derived_type*>(this);
}

/**
 * Returns a constant reference to the actual derived type of the OptFlowBase
 */
template <typename E, typename StreamType>
inline auto OptFlowBase<E, StreamType>::derived_cast() const& noexcept -> const derived_type& {
  return *static_cast<const derived_type*>(this);
}

/**
 * Returns a constant reference to the actual derived type of the OptFlowBase
 */
template <typename E, typename StreamType>
inline auto OptFlowBase<E, StreamType>::derived_cast() && noexcept -> derived_type {
  return *static_cast<derived_type*>(this);
}
