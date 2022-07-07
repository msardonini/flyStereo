#pragma once

#include "flyStereo/image_processing/detections/cv_goodfeaturestotrack.h"
#include "flyStereo/image_processing/optical_flow/pyr_lk_cv_gpu.h"
#include "flyStereo/image_processing/streams/cv_stream.h"

struct CvBackend {
  using flow_type = PyrLkCvGpu;
  using stream_type = CvStream;
  using detector_type = CvGoodFeaturesToTrack;
  using image_type = UMat<uint8_t>;
  using array_type = UMat<cv::Vec2f>;
  using status_type = UMat<uint8_t>;
};
