#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "gui.h"
#include "utils.h"

#if defined(USE_PYLON)
  #include "capture/pylon.h"
#elif defined(USE_FLYCAPTURE)
  #include "capture/flycapture.h"
#else
  #include "capture/opencv.h"
#endif


#define SAMPLE_CT 5
unsigned long int dataxl[SAMPLE_CT];
unsigned long int datayl[SAMPLE_CT];

size_t BUFFER_LENGTH = 4;
size_t MAX_RANGE_X;
size_t MAX_RANGE_Y;
float VIDEO_FPS = 30;
bool DETECT = true;
bool DETECT_BLOB = true;

std::string WINDOW_MAIN = "window";
std::string WINDOW_BINARY = "binary";
std::string WINDOW_CONTROL = "control";
std::string WINDOW_SET = "set";

cv::Scalar RED = cv::Scalar(0, 0, 255);
cv::Scalar GREEN = cv::Scalar(0, 255, 0);

int main() {
  char key = 0;
  CStopWatch timer;

  std::string filename = "output/" + get_file_name();
  std::ofstream save_file((filename + ".csv").c_str());
  cv::Mat frame;
  cv::Mat frame_mono;
  cv::Mat frame_annotated;

  // buffers for heuristic filtering
  boost::circular_buffer<float> buffer_x(BUFFER_LENGTH);
  boost::circular_buffer<float> buffer_y(BUFFER_LENGTH);
  float tmp1;
  float tmp2;

  // Camera Setup
  int frame_i = -1;
  unsigned int camera_id = 0;

  #ifdef USE_PYLON
    std::cout << "Using Pylon capture" << std::endl;
    PylonCapture capture(camera_id);
  #elif defined(USE_FLYCAPTURE)
    std::cout << "Using FlyCapture capture" << std::endl;
    PylonCapture capture(camera_id);
  #else
    std::cout << "Using OpenCV capture" << std::endl;
    OpenCvCapture capture(camera_id);
  #endif

  cv::Size capture_size = capture.get_frame_size();
  float max_x = capture_size.width;
  float max_y = capture_size.height;


  // Set up window with ROI and offset
  // cv::namedWindow(WINDOW_SET, cv::WINDOW_NORMAL);
  cv::namedWindow(WINDOW_MAIN, cv::WINDOW_NORMAL);
  cv::namedWindow(WINDOW_CONTROL, cv::WINDOW_NORMAL);
  cv::resizeWindow(WINDOW_MAIN, 600, 500);
  cv::resizeWindow(WINDOW_CONTROL, 250, 80);
  cv::moveWindow(WINDOW_MAIN, 0, 0);
  cv::moveWindow(WINDOW_CONTROL, 650, 0);

  // Trackbar stuff
  Params params = {
    0,        // pupil_threshold_min
    100,      // pupil_threshold_max
    5,        // pupil_min_area
    1000,     // pupil_max_area
    200,      // cr_threshold_min
    255,      // cr_threshold_max
    14,       // cr_min_area
    500,      // cr_max_area
    // 0.1,      // circularity
    100,      // circularity
    // 0.87,     // convexity
    870,      // convexity
    // 0.01,     // inertia
    10,       // inertia
    100,      // gain_x
    100,      // gain_y
    0,        // heuristic
    0         // record
  };
  Gui gui(&params, WINDOW_CONTROL);

  // Initialize video recorder
  cv::VideoWriter vid;
  vid.open(filename + "-video.avi", 1196444237, VIDEO_FPS, capture_size, true);

  key = 0;
  while(key != 'q') {
    timer.start();

    bool ok = capture.read(frame);
    if (!ok) {
      continue;
    }

    // frame_mono = frame;
    cv::cvtColor(frame, frame_mono, cv::COLOR_BGR2GRAY);
    // cv::imshow(WINDOW_MAIN, frame_mono);

    if (DETECT) {
      // TODO(aelsen) should these be kept, not reinitialized?
      float pos_x = 0;
      float pos_y = 0;


      // // Blob detection
      // std::vector<cv::KeyPoint> keypoints;


      cv::Mat binary_pupil = cv::Mat(frame_mono.clone());
      cv::Mat binary_cr = cv::Mat(frame_mono.clone());


      if (DETECT_BLOB) {
        cv::SimpleBlobDetector::Params p_params;
        p_params.blobColor = 0;
        p_params.filterByArea = true;
        p_params.filterByCircularity = true;
        p_params.filterByConvexity = true;
        p_params.filterByInertia = true;
        p_params.minThreshold = params.pupil_threshold_min;
        p_params.maxThreshold = params.pupil_threshold_max;
        p_params.minArea = (float) params.pupil_min_area;
        p_params.maxArea = (float) params.pupil_max_area;
        p_params.minCircularity = (float) params.min_circularity / 10.0;
        p_params.minConvexity = (float) params.min_convexity / 1000.0;
        p_params.minInertiaRatio = (float) params.min_inertia / 10000.0;

        cv::SimpleBlobDetector::Params cr_params;
        cr_params.blobColor = 0;
        cr_params.filterByArea = true;
        cr_params.filterByCircularity = true;
        cr_params.filterByConvexity = true;
        cr_params.filterByInertia = true;
        cr_params.minThreshold = params.cr_threshold_min;
        cr_params.maxThreshold = params.cr_threshold_max;
        cr_params.minArea = (int) params.cr_min_area * 100.0;
        cr_params.maxArea = (int) params.cr_max_area * 100.0;
        cr_params.minCircularity = (int) params.min_circularity / 10.0;
        cr_params.minConvexity = (int) params.min_convexity / 1000.0;
        cr_params.minInertiaRatio = (int) params.min_inertia / 10000.0;

        cv::Ptr<cv::SimpleBlobDetector> detector_p = cv::SimpleBlobDetector::create(p_params);
        cv::Ptr<cv::SimpleBlobDetector> detector_cr = cv::SimpleBlobDetector::create(cr_params);


        // Find pupil
        std::vector<cv::KeyPoint> keypoints_p;
        detector_p->detect(frame, keypoints_p);

        for (cv::KeyPoint kp : keypoints_p) {
          cv::circle(frame, kp.pt, kp.size / 2, RED);
          cv::circle(binary_pupil, kp.pt, kp.size / 2, RED);
          std::cout << "Pupil: ["
            << kp.pt.x << ", " << kp.pt.y
            << "], diameter: " << kp.size
            << std::endl;
        }


        // Find cr
        std::vector<cv::KeyPoint> keypoints_cr;
        detector_p->detect(frame, keypoints_cr);

        for (cv::KeyPoint kp : keypoints_cr) {
          cv::circle(frame, kp.pt, kp.size / 2, GREEN);
          cv::circle(binary_pupil, kp.pt, kp.size / 2, GREEN);
        }

      } else {
        // Contour Algo
        // Find pupil
        cv::threshold(binary_pupil, binary_pupil, params.pupil_threshold_min, params.pupil_threshold_max, cv::THRESH_BINARY_INV);
        std::vector<Blob> blobs_p = find_contours(binary_pupil, params.pupil_min_area, params.pupil_max_area, 1);
        std::vector<Contour> contours_p;

        cv::cvtColor(binary_pupil, binary_pupil, cv::COLOR_GRAY2BGR);
        for (Blob blob : blobs_p) {
          cv::circle(frame, blob.centroid, blob.bounding_box.width / 2, RED);
          cv::circle(binary_pupil, blob.centroid, blob.bounding_box.width / 2, RED);
          contours_p.push_back(blob.points);
          std::cout << "Pupil: ["
            << blob.centroid.x << ", " << blob.centroid.y
            << "], area: " << blob.area
            << std::endl;
        }

        cv::drawContours(binary_pupil, contours_p, -1, RED, 2);

        // Find cr
        cv::threshold(binary_cr, binary_cr, params.cr_threshold_min, params.cr_threshold_max, cv::THRESH_BINARY);
        std::vector<Blob> blobs_cr = find_contours(binary_cr, params.cr_min_area, params.cr_max_area, 3);
        std::vector<Contour> contours_cr;
        cv::cvtColor(binary_cr, binary_cr, cv::COLOR_GRAY2BGR);
        for (Blob blob : blobs_cr) {
          cv::circle(frame, blob.centroid, blob.bounding_box.width / 2, GREEN);
          cv::circle(binary_pupil, blob.centroid, blob.bounding_box.width / 2, GREEN);
          contours_cr.push_back(blob.points);
        }
        cv::drawContours(binary_cr, contours_cr, -1, GREEN, 2);
        std::cout << "Blobs: pupil " << contours_p.size() << ", cr " << contours_cr.size() << std::endl;

        cv::imshow(WINDOW_BINARY, binary_pupil);
        cv::imshow("BINARY CR", binary_cr);
        
      }



      cv::imshow(WINDOW_MAIN, frame);


      // // Downsampling
      // int n = SAMPLE_CT * sizeof(unsigned int);
      // for (int i = 0; i < SAMPLE_CT; i++) {
      //   dataxl[i] = buffer_x[2];
      //   datayl[i] = buffer_y[2];
      // }
    }

    // Record the video - this is slow!
    // if (record_video == 1){
    //   vid.write(frame);
    // }

    timer.stop();
    unsigned int delay_ms = timer.get_duration();
    float delay_secs = float(delay_ms) / 1000;
    // std::cout << "FPS: " << 1 / delay_secs << ", delay: " << delay_secs << " secs" << std::endl;
    key = cv::waitKey(1);
  }
}

