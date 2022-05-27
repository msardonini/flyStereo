#pragma once
#include "vpi/Types.h"

static inline void check_status(VPIStatus status) {
  if (status != VPI_SUCCESS) {
    throw std::runtime_error(vpiStatusGetName(status));
  }
}
static inline void check_status(cudaError status) {
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
}
