
#include "flyStereo/image_processing/detections/detections_base.h"
#include "flyStereo/image_processing/streams/cv_cpu_stream.h"
#include "flyStereo/types/umat.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class CvGoodFeaturesToTrackCpu
    : public DetectionsBase<CvGoodFeaturesToTrackCpu, CvStreamCpu, UMatCpu<uint8_t>, UMatCpu<cv::Vec2f>> {
 public:
  CvGoodFeaturesToTrackCpu() = default;

  void impl(const UMatCpu<uint8_t>& image, const UMatCpu<uint8_t>& mask, UMatCpu<cv::Vec2f>& detections,
            CvStreamCpu* stream) {
    cv::Mat output;
    cv::goodFeaturesToTrack(image.frame(), output, max_corners_, quality_level_, min_dist_, mask.frame());
    if (output.rows > 1 && output.cols == 1) {
      detections = output.t();
    } else {
      detections = output;
    }
  }

 private:
  // Algorithm Constants
  static constexpr int max_corners_ = 300;
  static constexpr float quality_level_ = 0.15f;
  static constexpr float min_dist_ = 15.f;
};
