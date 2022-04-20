#include "my_aruco/optim/aruco_optimizer_kalman_filter.h"


namespace my_aruco::optim
{

ArucoOptimizerKalmanFilter::ArucoOptimizerKalmanFilter(const Parameters& p, const ros::NodeHandle& nh) : ArucoOptimizer(p, nh) {
  // init publisher
  predictionPub_ = nh_.advertise<my_aruco_msg::AngleStamped>("prediction", 10);

  // init subscriber
  stateSub_ = nh_.subscribe(p_.topicRobotState, 10, &ArucoOptimizerKalmanFilter::RobotStateMsgCallback, this);

  // init params
  dt_ = 1.0 / p_.fps;

  l1_ = p_.l1;
  l2_ = p_.l2;
  P0_ = p_.P0;
  Q_ = p_.Q;
  R_ = p_.R;

  currV_ = lastV_ = 0.0;
  currW_ = lastW_ = 0.0;
}

void ArucoOptimizerKalmanFilter::Run() {
  if (!Update())
    return;

  // publish prediction
  prediction_.header.stamp = ros::Time::now();
  prediction_.radian = predYaw_;
  prediction_.degree = predYaw_ * 180 / M_PI;
  predictionPub_.publish(prediction_);

  // publish estimation
  estimation_.header.stamp = ros::Time::now();
  estimation_.radian = currYaw_;
  estimation_.degree = currYaw_ * 180 / M_PI;

  estimationPub_.publish(estimation_);

  return;

}

bool ArucoOptimizerKalmanFilter::Update() {
  // ekf -  initialization
  if (!initialized_) {
    if (!isNewMeasurement_)
      return false;

    // use the first measurement as initial yaw angle in kalman filter
    currYaw_ = measurement_.radian;
    lastYaw_ = measurement_.radian;
    isNewMeasurement_ = false;

    P_ = P0_;

    initialized_ = true;

    ROS_INFO("EKF initialized");
    return false;
  }

  // efk - prediction
  if (!isNewRobotState_) {
    ROS_WARN("robot state is not valid");
    return false;
  }

  Predict();
  P_ = F_ * P_ * F_ + Q_;

  // ekf - without measurenment
  if (!isNewMeasurement_) {
    lastYaw_ = currYaw_;
    currYaw_ = predYaw_; // use prediction as estimation, when no measurement
    return true;
  }

  // ekf - has measurement
  // ekf - gain
  K_ = P_ * G_ / (G_ * P_ * G_ + R_);
  P_ = (1 - K_ * G_) * P_;

  // ekf - update
  lastYaw_ = currYaw_;
  currYaw_ = predYaw_ + (measurement_.radian - G_ * predYaw_) * K_;

  isNewMeasurement_ = false;
  return true;
}

void ArucoOptimizerKalmanFilter::RobotStateMsgCallback(const my_aruco_msg::RobotState::Ptr& msg) {
  currV_ = msg->linear;
  currW_ = msg->angular;

  isNewRobotState_ = true;
}

void ArucoOptimizerKalmanFilter::Predict() {
  // middle value theorem to get dYaw
  double dLastYaw = -lastV_ / l2_ * sin(lastYaw_) - lastW_ * l1_ / l2_ * cos(lastYaw_) - lastW_;
  double dCurrYaw = -currV_ / l2_ * sin(currYaw_) - currW_ * l1_ / l2_ * cos(currYaw_) - currW_;
  double dYaw = (dLastYaw + dCurrYaw) * 0.5;
  // std::cout << dt_ << std::endl;
  predYaw_ = lastYaw_ + dYaw * dt_;

  F_ = 1 - lastV_ / l1_ * cos(lastYaw_) + lastW_ * l1_  / l2_ * sin(lastYaw_);

  // store states
  lastV_ = currV_;
  lastW_ = currW_;

  isNewRobotState_ = false;
}

} // namespace my_aruco:optim
