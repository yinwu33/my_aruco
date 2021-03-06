#pragma once

#include "my_aruco/utils/parameters.h"
#include "my_aruco/optim/aruco_optimizer.h"
#include "my_aruco/optim/aruco_optimizer_moving_avg.h"
#include "my_aruco/optim/aruco_optimizer_kalman_filter.h"

namespace my_aruco::optim
{

std::unique_ptr<ArucoOptimizer> create(const Parameters& p, const ros::NodeHandle& nh) {
  switch (p.optimizer)
  {
  case Optimizer::NONE:
    return nullptr;

  case Optimizer::MOVING_AVG:
    return std::make_unique<ArucoOptimizerMovingAvg>(p, nh);

  case Optimizer::EKF:
    return std::make_unique<ArucoOptimizerKalmanFilter>(p, nh);
  
  default:
    return nullptr;
  }
}
  
} // namespace my_aruco_optim
