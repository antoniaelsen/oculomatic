#ifndef SRC_GUI_
#define SRC_GUI_

#include "utils.h"

class Gui {
 public:
  Gui(Params* params, std::string window);

 private:
  Params* params_;
  int min_threshold_slider_max = 50;
  int min_threshold_slider = 15;

  int max_threshold_slider_max = 100;
  int max_threshold_slider = 45;

  int min_area_slider_max = 100;
  int min_area_slider = 14;

  int max_area_slider_max = 500;
  int max_area_slider = 290;

  int min_circularity_slider_max = 10;
  int min_circularity_slider = 1;

  int min_convexity_slider_max = 1000;
  int min_convexity_slider = 870;

  int min_inertia_slider_max = 1000;
  int min_inertia_slider = 100;

  int record_video = 0;
  int rec_slider = 0;
  int rec_slider_max = 1;

  int gain_x_slider_max = 300;
  int gain_x_slider = 160;

  int gain_y_slider_max = 300;
  int gain_y_slider = 160;

  int centerxy_slider = 0;

  static void min_threshold_trackbar(int, void* object);
  static void max_threshold_trackbar(int, void* object);

  static void min_area_trackbar(int, void* object);
  static void max_area_trackbar(int, void* object);

  static void min_circularity_trackbar(int, void* object);
  static void min_convexity_trackbar(int, void* object);
  static void min_inertia_trackbar(int, void* object);

  static void rec_trackbar(int, void* object);

  static void gain_x_trackbar(int, void* object);
  static void gain_y_trackbar(int, void* object);

  static void centerxy_trackbar(int, void* object);
};

#endif  // SRC_GUI_