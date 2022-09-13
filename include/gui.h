#ifndef SRC_GUI_
#define SRC_GUI_

#include "utils.h"

class Gui {
 public:
  Gui(Params* params, std::string window);

 private:
  Params* params_;
  int PUPIL_MAX_AREA_MAX = 1000;
  int PUPIL_MIN_AREA_MAX = 500;

  int CR_MAX_AREA_MAX = 500;
  int CR_MIN_AREA_MAX = 250;

  float MIN_CIRCULARITY_MAX = 1000;
  float MIN_CONVEXITY_MAX = 1000;
  float MIN_INERTIA_MAX = 100;

  int GAIN_X_MAX = 500;
  int GAIN_Y_MAX = 500;
  int GUI_BOOL_MAX = 1;
  int GUI_LUMINOSITY_MAX = 255;
};

#endif  // SRC_GUI_