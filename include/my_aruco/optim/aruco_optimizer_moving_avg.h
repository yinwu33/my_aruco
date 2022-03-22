#pragma once

#include "my_aruco/optim/aruco_optimizer.h"


namespace my_aruco::optim
{

class ArucoOptimizerMovingAvg : public ArucoOptimizer {
public:
  ArucoOptimizerMovingAvg() = delete;
  ArucoOptimizerMovingAvg(const Parameters& p);

  void SetWindowSize(int size);
  void Update(my_aruco::Markers& markers);

private:
  int windowSize_ = 5;
  std::vector<double> buffer_;

};

} // namespace my_aruco:optim
