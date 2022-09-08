#include "capture/pylon.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#ifdef USE_PYLON

const bool MONO = true;
const unsigned int TIMEOUT = 10;
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
    capture->RegisterConfiguration(new Pylon::CSoftwareTriggerConfiguration, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete);
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
  return capture->IsOpen() && capture->CanWaitForFrameTriggerReady();
}

bool PylonCapture::read(cv::Mat& frame) {
  cv::Mat frame_cv;
  Pylon::CPylonImage frame_pylon;

  capture->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
  bool ready = capture->WaitForFrameTriggerReady(1, Pylon::TimeoutHandling_ThrowException);
  capture->ExecuteSoftwareTrigger();
  bool ok = capture->RetrieveResult(TIMEOUT, grab, Pylon::TimeoutHandling_Return);
  capture->StopGrabbing();
  if (!ok) {
    std::cout
      << "Failed to grab image from pylon camera: "
      << grab->GetErrorCode() << " "
      << grab->GetErrorDescription()
      << std::endl;
    return ok;
  }

  converter.Convert(frame_pylon, grab);
  size_t cols = frame_pylon.GetWidth();
  size_t rows = frame_pylon.GetHeight();

  if (MONO) {
    frame_cv = cv::Mat(rows, cols, CV_8UC1, (uint8_t*)frame_pylon.GetBuffer());
  } else {
    frame_cv = cv::Mat(rows, cols, CV_8UC3, (uint8_t*)frame_pylon.GetBuffer());
  }

  cv::resize(frame_cv, frame, capture_size, 0, 0, cv::INTER_AREA);
  return true;
}

#endif