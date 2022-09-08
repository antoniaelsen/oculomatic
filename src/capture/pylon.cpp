#include "capture/pylon.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#ifdef USE_PYLON

const bool MONO = false;
const unsigned int TIMEOUT = 5000;
const float SCALE = 0.5;

PylonCapture::PylonCapture(unsigned int camera_id) {
  Pylon::PylonInitialize();

  if (MONO) {
    converter.OutputPixelFormat = Pylon::PixelType_Mono8;
  } else {
    converter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
  }

  try {
    capture = std::make_unique<Pylon::CInstantCamera>(
      Pylon::CTlFactory::GetInstance().CreateFirstDevice()
    );
    std::cout << "Created Pylon camera for device " << capture->GetDeviceInfo().GetModelName() << std::endl;

    capture->Open();

    GenApi::INodeMap& info = capture->GetNodeMap();
    GenApi::CIntegerPtr w_ptr = info.GetNode("Width");
    GenApi::CIntegerPtr h_ptr = info.GetNode("Height");
    unsigned int width = (int)w_ptr->GetValue();
    unsigned int height = (int)h_ptr->GetValue();
    capture_size = cv::Size(int(float(width) * SCALE), int(float(height) * SCALE));

    std::cout << "Frame dimentions: " << width << " x " << height << std::endl;
    std::cout << "Rescaled dimentions: " << capture_size.width << " x " << capture_size.height << std::endl;

  } catch (const Pylon::GenericException& e) {
    std::cout << "Failed to create Pylon camera - " << e.what() << std::endl;
  }
}

PylonCapture::~PylonCapture() {
  if (capture) {
    capture->Close();
  }
  close();
}

bool PylonCapture::close() {
  Pylon::PylonTerminate();
  return true;
}

bool PylonCapture::ready() {
  if (!capture) return false;
  return capture->IsOpen();
}

bool PylonCapture::read(cv::Mat& frame) {
  cv::Mat frame_cv;
  Pylon::CPylonImage frame_pylon;

  bool ok = capture->GrabOne(TIMEOUT, grab, Pylon::TimeoutHandling_ThrowException);
  if (!ok) {
    std::cout
      << "Failed to grab image from pylon camera: "
      << grab->GetErrorCode() << " "
      << grab->GetErrorDescription()
      << std::endl;
    return false;
  }

  converter.Convert(frame_pylon, grab);
  size_t cols = frame_pylon.GetWidth();
  size_t rows = frame_pylon.GetHeight();

  std::cout
    << "frame - "
    << ", width: " << cols
    << ", height: " << rows
    << ", # pixels: " << rows * cols
    << ", # bytes (expected): " << rows * cols * (MONO ? 1 : 3)
    << ", # bytes (reported): " << frame_pylon.GetImageSize()
    << ", pix type: " << std::hex << frame_pylon.GetPixelType() << std::dec
    << ", pix type (mono8): " << std::hex << Pylon::PixelType_Mono8 << std::dec
    << ", pix type (bgr8p): " << std::hex << Pylon::PixelType_BGR8packed << std::dec
    << std::endl;

  if (MONO) {
    frame_cv = cv::Mat(rows, cols, CV_8UC1, (uint8_t*)frame_pylon.GetBuffer());

    std::cout << "Pixels Pylon:" << std::endl;
    for (int i = 0; i < 3; i ++) {
      Pylon::SPixelData pix_py = frame_pylon.GetPixelData(i, 0);
        std::cout << "- mono: " << pix_py.Data.Mono << std::endl;
    }

    std::cout << "Pixels OpenCv:" << std::endl;
    for (int i = 0; i < 3; i ++) {
      unsigned int pix_cv = frame_cv.at<uint8_t>(0, i);
      std::cout << "- mono: " << pix_cv << std::endl;
    }

  } else {
    frame_cv = cv::Mat(rows, cols, CV_8UC3, (uint8_t*)frame_pylon.GetBuffer());

    std::cout << "Pixels Pylon:" << std::endl;
    for (int i = 0; i < 3; i ++) {
      Pylon::SPixelData pix_py = frame_pylon.GetPixelData(i, 0);
      std::cout
        << "- b: " << pix_py.Data.RGB.B << std::endl
        << "- g: " << pix_py.Data.RGB.G << std::endl
        << "- r: " << pix_py.Data.RGB.R << std::endl;
    }

    std::cout << "Pixels OpenCv:" << std::endl;
    for (int i = 0; i < 3; i ++) {
      cv::Vec3b pix_cv = frame_cv.at<cv::Vec3b>(0, i);
      std::cout
        << "- b: " << (unsigned int)pix_cv[0] << std::endl
        << "- g: " << (unsigned int)pix_cv[0] << std::endl
        << "- r: " << (unsigned int)pix_cv[0] << std::endl;
    }
  }

  cv::resize(frame_cv, frame, capture_size, 0, 0, cv::INTER_AREA);
  // frame_cv.copyTo(frame);
  return true;
}

#endif