#pragma once

#include "my_aruco/utils/parameters.h"
#include "my_aruco/types/markers.h"

namespace my_aruco::optim
{

class ArucoCalibrator {
public:
  ArucoCalibrator();
  ArucoCalibrator(const Parameters&);

  bool Valid();

  void Calib();

private:

  Parameters p_;

};
  
} // namespace my_aruco:optim
