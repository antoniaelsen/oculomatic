#ifndef SRC_CAPTURE_
#define SRC_CAPTURE_

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>


class Capture {
 public:
  virtual bool close() { return true; };
  virtual bool ready() { return true; };
  virtual bool read(cv::Mat &frame) = 0;
  cv::Size get_frame_size() { return capture_size; };

 protected:
  int frame_i = -1;
  cv::Size capture_size;
};

#endif  // SRC_CAPTURE_