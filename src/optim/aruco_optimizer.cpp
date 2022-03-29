#include "my_aruco/optim/aruco_optimizer.h"

namespace my_aruco::optim
{

ArucoOptimizer::ArucoOptimizer(const Parameters& p, const ros::NodeHandle& nh) : p_(p), nh_(nh) {
  if (p_.mode == Mode::ARUCO_1D) {
    measurementSub_ = nh_.subscribe("measurement", 100, &ArucoOptimizer::MsgCallback1D, this);
    estimationPub_ = nh_.advertise<my_aruco_msg::AngleStamped>("estimation", 100);
  }
  else {
    throw; // todo
  }

  newMeasurement_ = false;
}

void ArucoOptimizer::MsgCallback1D(const my_aruco_msg::AngleStamped::Ptr& msg) {
  angle_ = *msg;

  newMeasurement_ = true;
}

} // namespace my_aruco::optim
