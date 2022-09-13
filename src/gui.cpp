#include "gui.h"
#include <opencv2/highgui.hpp>

// Initialize Trackbars

Gui::Gui(Params* params, std::string window) : params_(params) {
  // make sliders
  cv::createTrackbar("Pupil Min Brightness Threshold", window, &params_->pupil_threshold_min, GUI_LUMINOSITY_MAX);
  cv::createTrackbar("Pupil Max Brightness Threshold", window, &params_->pupil_threshold_max, GUI_LUMINOSITY_MAX);
  cv::createTrackbar("Pupil Min Area", window, &params_->pupil_min_area, PUPIL_MIN_AREA_MAX);
  cv::createTrackbar("Pupil Max Area", window, &params_->pupil_max_area, PUPIL_MAX_AREA_MAX);

  cv::createTrackbar("CR Min Brightness Threshold", window, &params_->cr_threshold_min, GUI_LUMINOSITY_MAX);
  cv::createTrackbar("CR Max Brightness Threshold", window, &params_->cr_threshold_max, GUI_LUMINOSITY_MAX);
  cv::createTrackbar("CR Min Area", window, &params_->cr_min_area, CR_MIN_AREA_MAX);
  cv::createTrackbar("CR Max Area", window, &params_->cr_max_area, CR_MAX_AREA_MAX);

  cv::createTrackbar("Min Circularity", window, &params_->min_circularity, MIN_CIRCULARITY_MAX);
  cv::createTrackbar("Min Convexity", window, &params_->min_convexity, MIN_CONVEXITY_MAX);
  cv::createTrackbar("Min Inertia", window, &params_->min_inertia, MIN_INERTIA_MAX);

  cv::createTrackbar("Gain X" ,window, &params_->gain_x, GAIN_X_MAX);
  cv::createTrackbar("Gain Y", window, &params_->gain_y, GAIN_Y_MAX);

  cv::createTrackbar("Heuristic", window, &params_->heuristic, GUI_BOOL_MAX);
  cv::createTrackbar("Record", window, &params_->record, GUI_BOOL_MAX);
};
