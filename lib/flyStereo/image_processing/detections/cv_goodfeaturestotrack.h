
#include "flyStereo/image_processing/detections/detections_base.h"
#include "flyStereo/image_processing/streams/cv_stream.h"
#include "flyStereo/types/umat.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaimgproc.hpp"

// Algorithm Constants
static constexpr int max_corners = 300;
static constexpr float quality_level = 0.15f;
static constexpr float min_dist = 15.f;

class CvGoodFeaturesToTrack : public DetectionsBase<CvGoodFeaturesToTrack, CvStream, UMat<uint8_t>, UMat<cv::Vec2f>> {
 public:
  CvGoodFeaturesToTrack() = default;

  void impl(const UMat<uint8_t>& image, const UMat<uint8_t>& mask, UMat<cv::Vec2f>& detections, CvStream* stream) {
    if (!initialized_) {
      init(image.size());
    }

    cv::cuda::GpuMat output;
    if (stream) {
      detector_ptr_->detect(image.d_frame(), output, mask.d_frame(), stream->stream);
    } else {
      detector_ptr_->detect(image.d_frame(), output, mask.d_frame());
    }
    detections = output;
  }

 private:
  void init(cv::Size frame_size) {
    detector_ptr_ = cv::cuda::createGoodFeaturesToTrackDetector(CV_8U, max_corners, quality_level, min_dist);
    initialized_ = true;
  }

  bool initialized_ = false;
  cv::Ptr<cv::cuda::CornersDetector> detector_ptr_;
};
