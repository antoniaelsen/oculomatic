#ifndef SRC_OPENCV_CAPTURE_
#define SRC_OPENCV_CAPTURE_

#include <memory>
#include <opencv2/videoio.hpp>
#include "capture.h"


class OpenCvCapture : public Capture {
 public:
  OpenCvCapture(unsigned int camera_id);
  bool close() override;
  bool ready() override;
  bool read(cv::Mat &frame) override;

 private:
  std::unique_ptr<cv::VideoCapture> capture;
};

#endif  // SRC_OPENCV_CAPTURE_