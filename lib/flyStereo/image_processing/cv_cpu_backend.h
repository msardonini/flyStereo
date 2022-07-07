#pragma once

#include "flyStereo/image_processing/detections/cv_goodfeaturestotrack_cpu.h"
#include "flyStereo/image_processing/optical_flow/pyr_lk_cv_cpu.h"
#include "flyStereo/image_processing/streams/cv_cpu_stream.h"

struct CvCpuBackend {
  using flow_type = PyrLkCvCpu;
  using stream_type = CvStreamCpu;
  using detector_type = CvGoodFeaturesToTrackCpu;
  using image_type = UMatCpu<uint8_t>;
  using array_type = UMatCpu<cv::Vec2f>;
  using status_type = UMatCpu<uint8_t>;
};
