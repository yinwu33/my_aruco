#pragma once

#include "my_aruco/optim/aruco_optimizer.h"


namespace my_aruco::optim
{

class ArucoOptimizerMovingAvg : public ArucoOptimizer {
public:
  ArucoOptimizerMovingAvg() = delete;
  ArucoOptimizerMovingAvg(const Parameters&, const ros::NodeHandle&);

  void Run();

  bool Update() override;

  void SetWindowSize(int size);

private:
  int windowSize_ = 5;
  std::vector<double> buffer_;

  double optimizedAngle_;
};

} // namespace my_aruco:optim
