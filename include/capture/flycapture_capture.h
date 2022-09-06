#ifndef SRC_FLYCAPTURE_CAPTURE_
#define SRC_FLYCAPTURE_CAPTURE_

#ifdef USE_FLYCAPTURE
#include "flycapture/FlyCapture2.h"
#include "capture.h"


class FlyCaptureCapture : public Capture {
 public:
  FlyCaptureCapture(unsigned int camera_id);
  bool close() override;
  bool read(cv::Mat &frame) override;

 private:
  std::unique_ptr<FlyCapture2::Camera> capture;
};

#endif // USE_FLYCAPTURE

#endif  // SRC_FLYCAPTURE_CAPTURE_