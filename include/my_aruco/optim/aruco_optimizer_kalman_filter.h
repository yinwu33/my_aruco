#pragma once

#include "my_aruco/optim/aruco_optimizer.h"
#include "my_aruco_msg/AngleStamped.h"
#include "my_aruco_msg/RobotState.h"

#include <geometry_msgs/Twist.h>


namespace my_aruco::optim
{
class ArucoOptimizerKalmanFilter : public ArucoOptimizer {
public:
  using Ptr = std::unique_ptr<ArucoOptimizerKalmanFilter>;

  ArucoOptimizerKalmanFilter() = delete;
  ArucoOptimizerKalmanFilter(const Parameters& p, const ros::NodeHandle&);

  void Run();

  bool Update();

  void Predict();

private:
  void RobotStateMsgCallback(const my_aruco_msg::RobotState::Ptr& msg);

private:
  ros::Publisher predictionPub_;

  ros::Subscriber stateSub_;

  my_aruco_msg::AngleStamped prediction_;

  // robot state
  geometry_msgs::Twist robotState_;
  double lastV_, lastW_, currV_, currW_;
  bool isNewRobotState_ = false;

  // vehicle model parameters
  double l1_, l2_;

  double dt_;

  bool initialized_;

  double F_ = 1.;
  double G_ = 1.;

  // kalmanfilter covariance term
  double P0_, P_, Q_, R_;

  double K_;

  double lastYaw_, currYaw_, predYaw_;

};
} // namespace my_aruco

