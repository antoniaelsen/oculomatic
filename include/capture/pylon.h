#ifndef SRC_PYLON_CAPTURE_
#define SRC_PYLON_CAPTURE_

#ifdef USE_PYLON
#include <opencv2/core.hpp>
#include "pylon/PylonIncludes.h"
#include "capture.h"


class PylonCapture : public Capture {
 public:
  PylonCapture(unsigned int camera_id);
  ~PylonCapture();
  bool close() override;
  bool ready() override;
  bool read(cv::Mat &frame) override;

 private:
  std::unique_ptr<Pylon::CInstantCamera> capture;
  Pylon::CImageFormatConverter converter;
  Pylon::CGrabResultPtr grab;
};

#endif // USE_PYLON

#endif  // SRC_PYLON_CAPTURE