#pragma once

#include "my_aruco/utils/parameters.h"
#include "my_aruco/types/markers.h"

namespace my_aruco::optim
{

class ArucoOptimizer {
public:
  ArucoOptimizer() = delete;
  ArucoOptimizer(const Parameters& p);

  virtual void Update(Markers& markers) = 0;

private:
  Parameters p_;
};
  
} // namespace my_aruco
