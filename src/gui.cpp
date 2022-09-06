#include "gui.h"
#include <opencv2/highgui.hpp>

// Initialize Trackbars

Gui::Gui(Params* params, std::string window) : params_(params) {
  // make sliders
  cv::createTrackbar("Min Threshold", window, &min_threshold_slider, min_threshold_slider_max, &Gui::min_threshold_trackbar, this);
  cv::createTrackbar("Max Threshold", window, &max_threshold_slider, max_threshold_slider_max, &Gui::max_threshold_trackbar, this);
  cv::createTrackbar("Min Area", window, &min_area_slider, min_area_slider_max, &Gui::min_area_trackbar, this);
  cv::createTrackbar("Max Area", window, &max_area_slider, max_area_slider_max, &Gui::max_area_trackbar, this);
  cv::createTrackbar("Min Circularity", window, &min_circularity_slider, min_circularity_slider_max, &Gui::min_circularity_trackbar, this);
  cv::createTrackbar("Min Convexity", window, &min_convexity_slider, min_convexity_slider_max, &Gui::min_convexity_trackbar, this);
  cv::createTrackbar("Min Inertia", window, &min_inertia_slider, min_inertia_slider_max, &Gui::min_inertia_trackbar, this);
  cv::createTrackbar("Record", window, &rec_slider, rec_slider_max, &Gui::rec_trackbar, this);
  cv::createTrackbar("Gain X" ,window, &gain_x_slider, gain_x_slider_max, &Gui::gain_x_trackbar, this);
  cv::createTrackbar("Gain Y", window, &gain_y_slider, gain_y_slider_max, &Gui::gain_y_trackbar, this);
  cv::createTrackbar("center-XY", window, &centerxy_slider, 1, &Gui::centerxy_trackbar, this);
};


void Gui::min_threshold_trackbar(int, void* object) {
  Gui* gui = (Gui*) object;
  gui->params_->min_threshold = (int) gui->min_threshold_slider;
}

void Gui::max_threshold_trackbar(int, void* object) {
  Gui* gui = (Gui*) object;
  gui->params_->max_threshold = (int) gui->max_threshold_slider;
}

void Gui::min_area_trackbar(int, void* object) {
  Gui* gui = (Gui*) object;
  gui->params_->min_area = (int) gui->min_area_slider * 100;
}

void Gui::max_area_trackbar(int, void* object) {
  Gui* gui = (Gui*) object;
  gui->params_->max_area = (int) gui->max_area_slider * 100;
}

void Gui::min_circularity_trackbar(int, void* object) {
  Gui* gui = (Gui*) object;
  gui->params_->min_circularity = (int) gui->min_circularity_slider / 10;
}

void Gui::min_convexity_trackbar(int, void* object) {
  Gui* gui = (Gui*) object;
  gui->params_->min_convexity = (int) gui->min_convexity_slider / 1000;
}

void Gui::min_inertia_trackbar(int, void* object) {
  Gui* gui = (Gui*) object;
  gui->params_->min_inertia = (int) gui->min_inertia_slider / 10000;
}

void Gui::rec_trackbar(int, void* object) {
  Gui* gui = (Gui*) object;
  gui->record_video = (int) gui->rec_slider;
}

void Gui::gain_x_trackbar(int, void* object) {
  Gui* gui = (Gui*) object;
  gui->params_->gain_x = (int) gui->gain_x_slider;
}

void Gui::gain_y_trackbar(int, void* object) {
  Gui* gui = (Gui*) object;
  gui->params_->gain_y = (int) gui->gain_y_slider;
}

void Gui::centerxy_trackbar(int, void* object) {
  Gui* gui = (Gui*) object;
  // if (centerxy_slider == 0) {
  //   gui->params_->center_offset_x = 0;
  //   gui->params_->center_offset_y = 0;
  // }
  // else if(centerxy_slider == 1) {
  //   gui->params_->center_offset_x = xpos - cvFloor(0.5*max_rngx);
  //   gui->params_->center_offset_y = ypos - cvFloor(0.5*max_rngy);
  // }
}
