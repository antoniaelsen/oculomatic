#ifndef SRC_UTILS_
#define SRC_UTILS_

#include <string>
#include <time.h>

// CStopWatch:
// A simple timer class with start, stop, and get_duration function calls
class CStopWatch {
private:
  clock_t start_t;
  clock_t finish_t;

public:
  float get_duration();
  void start();
  void stop();
};

struct Params {
  int min_threshold;
  int max_threshold;
  int min_area;
  int max_area;
  float min_circularity;
  float min_convexity;
  float min_inertia;
  int gain_x;
  int gain_y;
  int center_offset_x = 0;
  int center_offset_y = 0;
};

const std::string current_date_time();

const std::string get_file_name();

#endif  // SRC_UTILS_