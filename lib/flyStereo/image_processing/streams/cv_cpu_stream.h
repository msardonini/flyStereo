#pragma once
#include "flyStereo/image_processing/streams/stream_base.h"

/**
 * @brief The CPU based classes don't have a stream, but it's required by the interface. So include an empty class here
 *
 */
class CvStreamCpu : public OptFlowStreamBase<CvStreamCpu> {
 public:
  CvStreamCpu(bool use_default_stream = false) {}

  void sync() {}
};
