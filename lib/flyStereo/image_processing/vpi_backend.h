#pragma once

#include "flyStereo/image_processing/optical_flow/pyr_lk_vpi_gpu.h"
#include "flyStereo/image_processing/streams/vpi_stream.h"

struct VpiBackend {
  using stream_type = VpiStream;
  using flow_type = PyrLkVpiGpu;
};
