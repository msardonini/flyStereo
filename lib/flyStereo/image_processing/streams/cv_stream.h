#pragma once
#include "flyStereo/image_processing/streams/stream_base.h"
#include "opencv2/core/cuda.hpp"

class CvStream : public OptFlowStreamBase<CvStream> {
 public:
  CvStream(bool use_default_stream = false) {
    if (use_default_stream) {
      stream = cv::cuda::Stream::Null();
    }
  }

  void sync() { stream.waitForCompletion(); }

  cv::cuda::Stream stream = cv::cuda::Stream();
};
