#pragma once

#include "flyStereo/image_processing/detections/vpi_harris.h"
#include "flyStereo/image_processing/optical_flow/pyr_lk_vpi_gpu.h"
#include "flyStereo/image_processing/streams/vpi_stream.h"

struct VpiBackend {
  using flow_type = PyrLkVpiGpu;
  using detector_type = VpiHarris;
  using stream_type = VpiStream;
  using image_type = UMatVpiImage;
  using array_type = UMatVpiArray<cv::Vec2f>;
  using status_type = UMatVpiArray<uint8_t>;
};
