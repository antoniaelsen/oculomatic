#include "capture/opencv.h"
#include <iostream>

OpenCvCapture::OpenCvCapture(unsigned int camera_id) {
  capture = std::make_unique<cv::VideoCapture>(camera_id);
  capture_size = cv::Size((int) capture->get(cv::CAP_PROP_FRAME_WIDTH),
                          (int) capture->get(cv::CAP_PROP_FRAME_HEIGHT));
}

bool OpenCvCapture::close() {
  capture->release();
  return true;
}

bool OpenCvCapture::ready() {
  return capture->isOpened();
}

bool OpenCvCapture::read(cv::Mat& frame) {
  return capture->read(frame);
}