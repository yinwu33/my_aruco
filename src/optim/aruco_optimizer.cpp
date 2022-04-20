#include "my_aruco/optim/aruco_optimizer.h"

namespace my_aruco::optim
{

ArucoOptimizer::ArucoOptimizer(const Parameters& p, const ros::NodeHandle& nh) : p_(p), nh_(nh) {
  measurementSub_ = nh_.subscribe("measurement", 100, &ArucoOptimizer::MsgCallback1D, this);
  estimationPub_ = nh_.advertise<my_aruco_msg::AngleStamped>("estimation", 100);

  isNewMeasurement_ = false;
}

void ArucoOptimizer::MsgCallback1D(const my_aruco_msg::AngleStamped::Ptr& msg) {
  measurement_ = *msg;

  isNewMeasurement_ = true;
}

} // namespace my_aruco::optim
