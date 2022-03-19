#pragma once

#include "my_aruco/utils/parameters.h"

namespace my_aruco::optim
{

class ArucoOptimizer {
public:
  ArucoOptimizer() = default;
  ArucoOptimizer(const Parameters& p);
};
  
} // namespace my_aruco
