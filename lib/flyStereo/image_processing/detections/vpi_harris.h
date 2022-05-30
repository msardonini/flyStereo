
#include "flyStereo/image_processing/detections/detections_base.h"
#include "flyStereo/image_processing/streams/vpi_stream.h"
#include "flyStereo/types/umat_vpiarray.h"
#include "flyStereo/types/umat_vpiimage.h"
#include "flyStereo/types/vpi_check.h"
#include "opencv2/core/core.hpp"
#include "vpi/Stream.h"
#include "vpi/algo/HarrisCorners.h"

constexpr int array_capacity = 5000;

class VpiHarris : public DetectionsBase<VpiHarris, VpiStream> {
 public:
  VpiHarris() = default;

  void impl(const UMatVpiImage& image, const UMat<uint8_t>& mask, UMatVpiArray<cv::Vec2f>& detections,
            VpiStream* stream) {
    if (!initialized_) {
      init(image.size());
    }

    image.unlock();
    detections.unlock();

    check_status(vpiSubmitHarrisCornerDetector(stream_, VPI_BACKEND_CUDA, payload_, image.vpi_frame(),
                                               detections.vpi_array(), out_scores_, &params_));
    check_status(vpiStreamSync(stream_));

    image.lock();
    detections.lock();

    detections.sync_cv_wrapper();
  }

 private:
  void init(cv::Size frame_size) {
    check_status(vpiStreamCreate(VPI_BACKEND_CUDA, &stream_));
    check_status(vpiCreateHarrisCornerDetector(VPI_BACKEND_CUDA, frame_size.width, frame_size.height, &payload_));
    check_status(vpiInitHarrisCornerDetectorParams(&params_));
    check_status(vpiArrayCreate(array_capacity, VPI_ARRAY_TYPE_U32, VPI_BACKEND_CUDA, &out_scores_));
    initialized_ = true;
  }
  bool initialized_;
  VPIPayload payload_;
  VPIStream stream_;
  VPIHarrisCornerDetectorParams params_;
  VPIArray out_scores_;
};
