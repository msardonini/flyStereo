#include "flyStereo/image_processing/opt_flow_stream_base.h"
#include "opencv2/core/cuda.hpp"

class OptFlowCvStream : public OptFlowStreamBase<OptFlowCvStream> {
 public:
  OptFlowCvStream(bool use_default_stream = false) {
    if (use_default_stream) {
      stream = cv::cuda::Stream::Null();
    }
  }

  void sync() { stream.waitForCompletion(); }

  cv::cuda::Stream stream = cv::cuda::Stream();
};
