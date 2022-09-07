
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>
#include <string>


#include <boost/circular_buffer.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "capture/opencv_capture.h"
#include "gui.h"
#include "utils.h"


#define SAMPLE_CT 5
unsigned long int dataxl[SAMPLE_CT];
unsigned long int datayl[SAMPLE_CT];

int MAX_RANGE_X;
int MAX_RANGE_Y;
float VIDEO_FPS = 30;

std::string WINDOW_MAIN = "window";
std::string WINDOW_CONTROL = "control";
std::string WINDOW_SET = "set";

int main() {
  char key = 0;
  float delay;
  CStopWatch timer;

  std::string filename = "output/" + get_file_name();
  std::ofstream save_file((filename + ".csv").c_str());
  cv::Mat frame;
  
  // buffers for heuristic filtering
  boost::circular_buffer<float> buffer_x(4);
  boost::circular_buffer<float> buffer_y(4);
  float tmp1;
  float tmp2;

  // Camera Setup
  int frame_i = -1;
  unsigned int camera_id = 0;
  OpenCvCapture capture(camera_id);
  cv::Size capture_size = capture.get_frame_size();
  float max_x = capture_size.width;
  float max_y = capture_size.height;

  // Configure Camera -----------------------------------
  // // Setup: User positions eye in FOV
  // // Wait for 'c' to be pushed to move on
  // std::cout << "Position eye inside field of view" << std::endl;
  // std::cout << "ROI selection is now done automagically" << std::endl;
  // std::cout << "press 1 or 2 to mirror image" << std::endl;
  // std::cout << "press c to continue" << std::endl;
  // while(key != 'c'){
  //   capture.read(frame);

  //   max_x = frame.cols;
  //   max_y = frame.rows;

  //   cv::imshow(WINDOW_SET, frame);
  //   key = cv::waitKey(30);
  // }
  // cv::destroyWindow(WINDOW_SET);
  // std::cout << "Configuration completed" << std::endl;

  // Set up window with ROI and offset
  cv::namedWindow(WINDOW_MAIN, cv::WINDOW_NORMAL);
  cv::namedWindow(WINDOW_CONTROL, cv::WINDOW_NORMAL);
  cv::namedWindow(WINDOW_SET, cv::WINDOW_NORMAL);
  cv::resizeWindow(WINDOW_MAIN, 600, 500);
  cv::resizeWindow(WINDOW_CONTROL, 250, 80);
  cv::moveWindow(WINDOW_MAIN, 0, 0);
  cv::moveWindow(WINDOW_CONTROL, 650, 0);

  // Trackbar stuff
  Params params = {
    15,       // min_threshold
    45,       // max_threshold
    1400,     // min_area
    29000,    // max_area
    0.1,      // min_circularity
    0.87,     // min_convexity
    0.01,     // min_inertia
    160,      // gain_x
    160,      // gain_y
    0,
    0
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
      std::cout << "Failed to capture frame" << std::endl;
    }

    cv::Mat image_mono;
    cv::Mat image = frame;
    cv::cvtColor(image, image_mono, cv::COLOR_BGR2GRAY);
    // cv::Mat(capture_size, CV_8UC1)
    // cv::flip(image, image, 0);
    // cv::flip(image_mono, image_mono, 0);

    std::vector<cv::KeyPoint> keypoints;
    cv::SimpleBlobDetector::Params blobparams_;

    blobparams_.blobColor = 0;
    blobparams_.filterByArea = true;
    blobparams_.filterByCircularity = true;
    blobparams_.filterByConvexity = true;
    blobparams_.filterByInertia = true;
    blobparams_.minThreshold = params.min_threshold;
    blobparams_.maxThreshold = params.max_threshold;
    blobparams_.minArea = params.min_area;
    blobparams_.maxArea = params.max_area;
    blobparams_.minCircularity = params.min_circularity;
    blobparams_.minConvexity = params.min_convexity;
    blobparams_.minInertiaRatio = params.min_inertia;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(blobparams_);

    // Detect blobs
    detector->detect(image_mono, keypoints);
    std::cout << "Keypoints: " << keypoints.size() << std::endl;

    cv::Mat image_annotated;
    cv::drawKeypoints(image_mono, keypoints, image_annotated, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );


    float pos_x = 0;
    float pos_y = 0;
    if (keypoints.size() > 0){
      cv::circle(image_annotated, keypoints[0].pt, 3, cv::Scalar(255, 0, 0), -1, 8, 0);
      pos_x = ((keypoints[0].pt.x - (max_x / 2)) / (max_x - params.gain_x)) * (float) MAX_RANGE_X;
      pos_y = ((keypoints[0].pt.y - (max_y / 2)) / (max_y - params.gain_y)) * (float) MAX_RANGE_Y;
      pos_x = pos_x - params.center_offset_x;
      pos_y = pos_y - params.center_offset_y;
    }

    if (buffer_x.size() < 3){
      buffer_x.push_front(pos_x);
      buffer_y.push_front(pos_y);
  
    } else {
      buffer_x.push_front(pos_x);
      buffer_y.push_front(pos_y);
  
      // filter level 1
      if (buffer_x[2] > buffer_x[1] && buffer_x[1] < buffer_x[0]){
        tmp1 = std::abs(buffer_x[1] - buffer_x[0]);
        tmp2 = std::abs(buffer_x[1] - buffer_x[2]);

        if(tmp2 > tmp1){
          buffer_x[1] = buffer_x[0];
        }
        else{
          buffer_x[1] = buffer_x[2] ;
        }
        buffer_x[3] = buffer_x[2];
        buffer_x[2] = buffer_x[1];
      } else if (buffer_x[2] < buffer_x[1] && buffer_x[1] > buffer_x[0]){
        tmp1 = std::abs(buffer_x[1] - buffer_x[0]);
        tmp2 = std::abs(buffer_x[1] - buffer_x[2]);

        if(tmp2 > tmp1){
          buffer_x[1] = buffer_x[0];
        }
        else{
          buffer_x[1] = buffer_x[2] ;
        }

        buffer_x[3] = buffer_x[2];
        buffer_x[2] = buffer_x[1];
      } else {
        buffer_x[3] = buffer_x[2];
        buffer_x[2] = buffer_x[1];
      }

      if (buffer_y[2] > buffer_y[1] && buffer_y[1] < buffer_y[0]) {
        tmp1 = std::abs(buffer_y[1] - buffer_y[0]);
        tmp2 = std::abs(buffer_y[1] - buffer_y[2]);

        if(tmp2 > tmp1){
          buffer_y[1] = buffer_y[0];
        }
        else{
          buffer_y[1] = buffer_y[2] ;
        }
        buffer_y[3] = buffer_y[2];
        buffer_y[2] = buffer_y[1];

      } else if (buffer_y[2] < buffer_y[1] && buffer_y[1] > buffer_y[0]) {
        tmp1 = std::abs(buffer_y[1] - buffer_y[0]);
        tmp2 = std::abs(buffer_y[1] - buffer_y[2]);

        if(tmp2 > tmp1){
          buffer_y[1] = buffer_y[0];
        }
        else{
          buffer_y[1] = buffer_y[2] ;
        }

        buffer_y[3] = buffer_y[2];
        buffer_y[2] = buffer_y[1];
      } else {
        buffer_y[3] = buffer_y[2];
        buffer_y[2] = buffer_y[1];
      }

      // Downsampling
      int n = SAMPLE_CT * sizeof(unsigned int);
      for (int i = 0; i < SAMPLE_CT; i++) {
        dataxl[i] = buffer_x[2];
        datayl[i] = buffer_y[2];
      }

      // Output to console
      // std::cout << "- " << dataxl[0] << "," << datayl[0] << std::endl;
  
      // TODO - output to analog voltage

      // Draw image
      cv::imshow(WINDOW_MAIN, image_annotated);
    }

    // Record the video - this is slow!
    // if (record_video == 1){
    //   vid.write(image);   
    // }
  
    timer.stop();
    delay = timer.get_duration() * 1000;
    key = cv::waitKey(10);
  }
}
