#include "capture/flycapture_capture.h"

#ifdef USE_FLYCAPTURE

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


FlyCaptureCapture::FlyCaptureCapture(unsigned int camera_id) {
  FlyCapture2::Error error;
  FlyCapture2::CameraInfo camera_info;
  capture = std::make_unique<FlyCapture2::Camera>(camera_id);
  error = camera->Connect(camera_id);
  if (error != FlyCapture2::PGRERROR_OK) {
    std::cout << "Failed to connect to camera" << std::endl;
  }

  error = camera->GetCameraInfo(&camera_info);
  if (error != FlyCapture2::PGRERROR_OK) {
    std::cout << "Failed to get camera info from camera" << std::endl;
  }
  std::cout << camera_info.vendorName << " " << camera_info.modelName << " " << camera_info.serialNumber << std::endl;

  error = camera->StartCapture();
  if (error != FlyCapture2::PGRERROR_OK) {
    std::cout << "Failed to start camera" << std::endl;

      if (error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
      std::cout << "Camera bandwidth exceeded" << std::endl;
      return false;
    }
  }
}

bool FlyCaptureCapture::close() {
  capture->release();
  return true;
}

bool FlyCaptureCapture::read(cv::Mat& frame) {
  FlyCapture2::Image image;
  FlyCapture2::Error error;
  error = capture->RetrieveBuffer(&image);
  if (error != FlyCapture2::PGRERROR_OK) {
    std::cout << "Failed to capture frame" << std::endl;
    return false;
  }

  image.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &image_rgb);
  unsigned int bytes = (float)image_rgb.GetReceivedDataSize() / (float)image_rgb.GetRows();
  frame = cv::Mat(image_rgb.GetRows(), image_rgb.GetCols(), CV_8UC3, image_rgb.GetData(), bytes);
  return true;
}

#endif