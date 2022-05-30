#pragma once

template <typename E>
class OptFlowStreamBase {
 public:
  using derived_type = E;

  derived_type& derived_cast() & noexcept;
  const derived_type& derived_cast() const& noexcept;
  derived_type derived_cast() && noexcept;

  void sync() { return derived_cast().sync(); }

 protected:
  OptFlowStreamBase() = default;
  ~OptFlowStreamBase() = default;

  OptFlowStreamBase(const OptFlowStreamBase&) = default;
  OptFlowStreamBase& operator=(const OptFlowStreamBase&) = default;

  OptFlowStreamBase(OptFlowStreamBase&&) = default;
  OptFlowStreamBase& operator=(OptFlowStreamBase&&) = default;
};

#include "flyStereo/image_processing/streams/stream_base.tpp"
