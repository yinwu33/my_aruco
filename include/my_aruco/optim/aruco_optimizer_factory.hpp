#pragma once

#include "my_aruco/utils/parameters.h"
#include "my_aruco/optim/aruco_optimizer.h"
#include "my_aruco/optim/aruco_optimizer_moving_avg.h"

namespace my_aruco::optim
{

std::unique_ptr<ArucoOptimizer> create(const Parameters& p) {
  switch (p.optimizer)
  {
  case my_aruco::Optimizer::MOVING_AVG:
    return std::make_unique<ArucoOptimizerMovingAvg>(p);
  
  default:
    return nullptr;
  }
}
  
} // namespace my_aruco_optim
