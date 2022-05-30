#include "flyStereo/image_processing/streams/stream_base.h"
#include "flyStereo/image_processing/vpi_check.h"
#include "vpi/Stream.h"

class VpiStream : public OptFlowStreamBase<VpiStream> {
 public:
  VpiStream(bool use_default_stream = false) {
    if (use_default_stream) {
      stream = nullptr;
    } else {
      check_status(vpiStreamCreate(VPI_BACKEND_CUDA, &stream));
    }
  }

  ~VpiStream() {
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
