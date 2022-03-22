#include "my_aruco/optim/aruco_optimizer_moving_avg.h"

namespace my_aruco::optim {

void ArucoOptimzerMovingAvg::Update(Markers& markers) {
  if (buffer_.size() >= windowSize_) {
    std::vector<double> tempBuffer(buffer_.begin() + 1, buffer_.end());
    std::swap(tempBuffer, buffer_);
  }

  buffer_.emplace_back(markers.yaw);

  double sum = 0.0;
  for (size_t i=0; i < buffer_.size(); ++i) {
    sum += buffer_[i];
  }

  markers.yaw = sum / buffer_.size();
}


}