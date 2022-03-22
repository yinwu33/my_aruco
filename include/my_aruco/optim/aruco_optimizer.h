#pragma once

#include "my_aruco/utils/parameters.h"
#include "my_aruco/types/markers.h"

namespace my_aruco::optim
{

class ArucoOptimizer {
public:

  using Ptr = std::unique_ptr<ArucoOptimizer>;

  ArucoOptimizer() = delete;
  ArucoOptimizer(const Parameters& p);

  virtual void Update(my_aruco::Markers& markers) = 0;

protected:
  Parameters p_;
};
  
} // namespace my_aruco
