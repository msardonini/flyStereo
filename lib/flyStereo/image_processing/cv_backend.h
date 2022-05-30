#pragma once

#include "flyStereo/image_processing/optical_flow/pyr_lk_cv_gpu.h"
#include "flyStereo/image_processing/streams/cv_stream.h"

struct CvBackend {
  using stream_type = CvStream;
  using flow_type = PyrLkCvGpu;
};
