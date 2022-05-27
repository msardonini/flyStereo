#include "flyStereo/image_processing/opt_flow_stream_base.h"
#include "flyStereo/image_processing/vpi_check.h"
#include "vpi/Stream.h"

class OptFlowVpiStream : public OptFlowStreamBase<OptFlowVpiStream> {
 public:
  OptFlowVpiStream(bool use_default_stream = false) {
    if (use_default_stream) {
      stream = nullptr;
    } else {
      check_status(vpiStreamCreate(VPI_BACKEND_CUDA, &stream));
    }
  }

  ~OptFlowVpiStream() {
    if (stream) {
      vpiStreamDestroy(stream);
    }
  }

  void sync() {
    if (stream) {
      check_status(vpiStreamSync(stream));
    }
  }

  VPIStream stream = nullptr;
};
