#include "my_aruco/optim/aruco_optimizer_moving_avg.h"

namespace my_aruco::optim {

ArucoOptimizerMovingAvg::ArucoOptimizerMovingAvg(const Parameters& p, const ros::NodeHandle& nh) : ArucoOptimizer(p, nh) {
  windowSize_ = p_.windowSize;
}

void ArucoOptimizerMovingAvg::Run() {
  Update1D();

  angle_.radian = optimizedAngle_;
  angle_.degree = optimizedAngle_ * 180 / M_PI;

  estimationPub_.publish(angle_);

  newMeasurement_ = false;
}

void ArucoOptimizerMovingAvg::Update1D() {
  if (!newMeasurement_)
    return; 

  if (windowSize_ == 0)
    return;

  // keep the buffer size
  if (buffer_.size() >= windowSize_) {
    std::vector<double> tempBuffer(buffer_.begin() + 1, buffer_.end());
    std::swap(tempBuffer, buffer_);
  }

  buffer_.emplace_back(angle_.radian);

  double sum = 0.0;
  for (size_t i = 0; i < buffer_.size(); ++i) {
    sum += buffer_[i];
  }

  optimizedAngle_ = sum / buffer_.size();
}

void ArucoOptimizerMovingAvg::SetWindowSize(int size) {
  if (size < 0) {
    throw;
  }

  windowSize_ = size;
}

}