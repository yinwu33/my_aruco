#include "my_aruco/optim/aruco_optimizer_moving_avg.h"

namespace my_aruco::optim {

ArucoOptimizerMovingAvg::ArucoOptimizerMovingAvg(const Parameters& p, const ros::NodeHandle& nh) : ArucoOptimizer(p, nh) {
  windowSize_ = p_.windowSize;
}

void ArucoOptimizerMovingAvg::Run() {
  Update();

  estimation_ = measurement_;
  estimation_.radian = optimizedAngle_;
  estimation_.degree = optimizedAngle_ * 180 / M_PI;

  estimationPub_.publish(estimation_);

  isNewMeasurement_ = false;
}

bool ArucoOptimizerMovingAvg::Update() {
  if (!isNewMeasurement_)
    return false; 

  if (windowSize_ == 0)
    return false;

  // keep the buffer size
  if (buffer_.size() >= windowSize_) {
    std::vector<double> tempBuffer(buffer_.begin() + 1, buffer_.end());
    std::swap(tempBuffer, buffer_);
  }

  buffer_.emplace_back(measurement_.radian);

  double sum = 0.0;
  for (size_t i = 0; i < buffer_.size(); ++i) {
    sum += buffer_[i];
  }

  optimizedAngle_ = sum / buffer_.size();

  return true;
}

void ArucoOptimizerMovingAvg::SetWindowSize(int size) {
  if (size < 0) {
    throw;
  }

  windowSize_ = size;
}

}