#include "utils.h"
#include <iostream>
#include <string>


using Clock = std::chrono::steady_clock;

using Circular = boost::circular_buffer<float>;

using Contour = std::vector<cv::Point>;


unsigned int CStopWatch::get_duration() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
}

void CStopWatch::start() {
  start_t = Clock::now();
}

void CStopWatch::stop() {
  end_t = Clock::now();
}

// Returns the current date and time
const std::string current_date_time() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d-%H%M%S", &tstruct);

  return buf;
}

const std::string get_file_name() {
  std::string input = "";
  std::string filename;
  // std::cout << "\nChoose a file name (default)" << std::endl;
  // getline(std::cin, input);
  if (input == "") {
    return current_date_time();
  }
  return input;
}

bool compare_contour_area(Contour a, Contour b) {
	float area_a = cv::contourArea(a);
	float area_b = cv::contourArea(b);
	
	return (area_a > area_b);
}

std::vector<Blob> find_contours(cv::Mat& input, int min_area, int max_area, int n_considered = -1) {
  std::vector<Blob> blobs;
  cv::Mat frame = cv::Mat(input.clone());

  std::vector<Contour> contours_all;
  std::vector<Contour> contours_filtered;
  std::vector<cv::Vec4i> hierarchy;
  int mode = cv::RETR_EXTERNAL;
  // int mode = cv::RETR_LIST;

  cv::findContours(frame, contours_all, hierarchy, mode, cv::CHAIN_APPROX_NONE);

  // Filter contours by area
  for (Contour contour : contours_all) {
    float area = cv::contourArea(contour);
    if (area > min_area && area < max_area) {
      contours_filtered.push_back(contour);
    }
  }

  sort(contours_filtered.begin(), contours_filtered.end(), compare_contour_area);

  if (n_considered == -1 || n_considered > contours_filtered.size()) n_considered = contours_filtered.size();
  for (int i = 0; i < n_considered; i++) {
    Blob blob;
    Contour contour = contours_filtered[i];
    float area = cv::contourArea(contour);
    cv::Moments moments = cv::moments(contour, false);
    cv::Rect rect = cv::boundingRect(contour);
    cv::Point2f centroid = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);

    blob.area = area;
    blob.length = cv::arcLength(contour, false);
    blob.bounding_box = rect;
    blob.centroid = centroid;
    blob.points = contour;

    blobs.push_back(blob);
  }

  return blobs;
}


void heuristic_filter(Circular& buffer_x, Circular& buffer_y, float pos_x, float pos_y) {
  // if (buffer_x.size() < 3) {
  //   buffer_x.push_front(pos_x);
  //   buffer_y.push_front(pos_y);

  // } else {
  //   buffer_x.push_front(pos_x);
  //   buffer_y.push_front(pos_y);

  //   // filter level 1
  //   if (buffer_x[2] > buffer_x[1] && buffer_x[1] < buffer_x[0]){
  //     tmp1 = std::abs(buffer_x[1] - buffer_x[0]);
  //     tmp2 = std::abs(buffer_x[1] - buffer_x[2]);

  //     if(tmp2 > tmp1){
  //       buffer_x[1] = buffer_x[0];
  //     }
  //     else{
  //       buffer_x[1] = buffer_x[2] ;
  //     }
  //     buffer_x[3] = buffer_x[2];
  //     buffer_x[2] = buffer_x[1];
  //   } else if (buffer_x[2] < buffer_x[1] && buffer_x[1] > buffer_x[0]) {
  //     tmp1 = std::abs(buffer_x[1] - buffer_x[0]);
  //     tmp2 = std::abs(buffer_x[1] - buffer_x[2]);

  //     if(tmp2 > tmp1){
  //       buffer_x[1] = buffer_x[0];
  //     }
  //     else{
  //       buffer_x[1] = buffer_x[2] ;
  //     }

  //     buffer_x[3] = buffer_x[2];
  //     buffer_x[2] = buffer_x[1];
  //   } else {
  //     buffer_x[3] = buffer_x[2];
  //     buffer_x[2] = buffer_x[1];
  //   }

  //   if (buffer_y[2] > buffer_y[1] && buffer_y[1] < buffer_y[0]) {
  //     tmp1 = std::abs(buffer_y[1] - buffer_y[0]);
  //     tmp2 = std::abs(buffer_y[1] - buffer_y[2]);

  //     if(tmp2 > tmp1){
  //       buffer_y[1] = buffer_y[0];
  //     }
  //     else{
  //       buffer_y[1] = buffer_y[2] ;
  //     }
  //     buffer_y[3] = buffer_y[2];
  //     buffer_y[2] = buffer_y[1];

  //   } else if (buffer_y[2] < buffer_y[1] && buffer_y[1] > buffer_y[0]) {
  //     tmp1 = std::abs(buffer_y[1] - buffer_y[0]);
  //     tmp2 = std::abs(buffer_y[1] - buffer_y[2]);

  //     if(tmp2 > tmp1){
  //       buffer_y[1] = buffer_y[0];
  //     }
  //     else{
  //       buffer_y[1] = buffer_y[2] ;
  //     }

  //     buffer_y[3] = buffer_y[2];
  //     buffer_y[2] = buffer_y[1];
  //   } else {
  //     buffer_y[3] = buffer_y[2];
  //     buffer_y[2] = buffer_y[1];
  //   }
  // }
}
