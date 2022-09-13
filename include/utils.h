#ifndef SRC_UTILS_
#define SRC_UTILS_

#include <chrono>
#include <string>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using Clock = std::chrono::steady_clock;
using std::chrono::time_point;

using Circular = boost::circular_buffer<float>;

using Contour = std::vector<cv::Point>;

// CStopWatch:
// A simple timer class with start, stop, and get_duration function calls
class CStopWatch {
 private:
  time_point<Clock> start_t;
  time_point<Clock> end_t;

 public:
  unsigned int get_duration();
  void start();
  void stop();
};

struct Blob {
  float area = 0;
  float length = 0;

  cv::Rect2f bounding_box;
  cv::Point2f centroid;
  Contour points;
};

struct Params {
  int pupil_threshold_min;
  int pupil_threshold_max;
  int pupil_min_area;
  int pupil_max_area;
  int cr_threshold_min;
  int cr_threshold_max;
  int cr_min_area;
  int cr_max_area;
  int min_circularity;
  int min_convexity;
  int min_inertia;
  int gain_x;
  int gain_y;
  int heuristic = 0;
  int record = 0;
};

const std::string current_date_time();

const std::string get_file_name();

bool compare_contour_area(Contour a, Contour b);

std::vector<Blob> find_contours(cv::Mat& input, int min_area, int max_area, int n_considered);

void heuristic_filter(Circular& buffer_x, Circular& buffer_y, float pos_x, float pos_y);

#endif  // SRC_UTILS_