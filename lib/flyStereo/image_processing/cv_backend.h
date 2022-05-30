#pragma once

#include "flyStereo/image_processing/optical_flow/pyr_lk_cv_gpu.h"
#include "flyStereo/image_processing/streams/cv_stream.h"

struct CvBackend {
  using flow_type = PyrLkCvGpu;
  using stream_type = CvStream;
};
