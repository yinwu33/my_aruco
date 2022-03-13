#include "my_aruco/optim/kalman_filter_ddrobot.h"

namespace my_aruco
{
static Eigen::Matrix3d Skew(Eigen::Vector3d v) {
  Eigen::Matrix3d m;
  m << 0, -v[2], v[1],
    v[2], 0, -v[0],
    -v[1], v[0], 0;
  return m;
}

KalmanFilterDDRobot::KalmanFilterDDRobot(ros::NodeHandle& nh, cv::FileNode& node)
  : nh_(nh), node_(node) {
  // init subscribers
  std::string leftVelTopic = (std::string)node["leftVelTopic"];
  std::string rightVelTopic = (std::string)node["rightVelTopic"];
  std::string measurementTopic = (std::string)node["measurementTopic"];
  std::string predictionTopic = (std::string)node["predictionTopic"];
  std::string estimationTopic = (std::string)node["estimationTopic"];

  leftVelSub_ = nh_.subscribe(leftVelTopic, 100, &KalmanFilterDDRobot::LeftVelCallback, this);
  rightVelSub_ = nh_.subscribe(rightVelTopic, 100, &KalmanFilterDDRobot::RightVelCallback, this);
  measurementSub_ = nh_.subscribe(measurementTopic, 100, &KalmanFilterDDRobot::MeasurenmentCallback, this);

  predictPub_ = nh_.advertise<std_msgs::Float64>(predictionTopic, 100);
  estimationPub_ = nh_.advertise<std_msgs::Float64>(estimationTopic, 100);

  // init covariance matrix
  P0_ = (double)node["P0"];
  Q_ = (double)node["Q"];
  R_ = (double)node["R"];

  // slamdog modell parameters
  b1_ = (double)node["b1"];
  b2_ = (double)node["b2"];
  l1_ = (double)node["l1"];
  l2_ = (double)node["l2"];
}

void KalmanFilterDDRobot::LeftVelCallback(const std_msgs::Float64::ConstPtr& msg) {
  vl_ = msg->data;
  vlIsNew_ = true;
}

void KalmanFilterDDRobot::RightVelCallback(const std_msgs::Float64::ConstPtr& msg) {
  vr_ = msg->data;
  vrIsNew_ = true;
}

void KalmanFilterDDRobot::MeasurenmentCallback(const std_msgs::Float64::ConstPtr& msg) {
  measurement_ = msg->data;
  measurementIsNew_ = true;
}


bool KalmanFilterDDRobot::Update() {
  if (!initialized_) {
    // cannot initiaiization because init state is unknown
    if (!measurementIsNew_) {
      return false;
    }

    // initialization: use first measurement as init state
    theta_last_ = theta_curr_ = measurement_;
    t_last_ = t_curr_ = ros::Time::now(); // ???

    P_ = P0_;

    measurementIsNew_ = false;

    initialized_ = true;

    return false;
  }

  // if (vlIsNew_ == false or vrIsNew_ == false)
  //   return false;

  // vlIsNew_ = false;
  // vrIsNew_ = false;
  std::cout << "=======" << std::endl;
  std::cout << theta_pred_ << std::endl;
  std::cout << vl_ << " " <<  vr_ << std::endl;
  std::cout << theta_last_ << std::endl;
  std::cout << theta_curr_ << std::endl;
  Predict();

  P_ = A_ * P_ * A_ + Q_;

  if (measurementIsNew_) {
    K_ = P_ * C_ / (C_ * P_ * C_ + R_);
    P_ = (1 - K_ * C_) * P_;

    // storage last states
    theta_last_ = theta_curr_;
    t_last_ = t_curr_;

    // update estimation
    theta_curr_ = theta_pred_ + K_ * (measurement_ - C_ * theta_pred_);
    t_curr_ = ros::Time::now();
  }
  else {
    // storage last states
    theta_last_ = theta_curr_;
    t_last_ = t_curr_;

    // update estimation
    theta_curr_ = theta_pred_;
    t_curr_ = ros::Time::now();
  }
  return true;
}

void KalmanFilterDDRobot::Run() {
  Update();
  Publish();
}

void KalmanFilterDDRobot::Publish() {

  prediction_.data = theta_pred_;
  predictPub_.publish(prediction_);

  estimation_.data = theta_curr_;
  estimationPub_.publish(estimation_);
}

void KalmanFilterDDRobot::Predict() {
  if (vl_ == 0 && vr_ == 0) {
    theta_pred_ = theta_curr_;
    return;
  }
  if (vl_ == vr_ and theta_curr_ == 0) {
    vm_ << vl_, 0, 0;
    vc1_ << vl_, 0, 0;
    vc2_ << vl_, 0, 0;
    theta_pred_ = 0.0;
  }

  // the angular velocity of the slamdog
  Eigen::Vector3d w1(0, 0, (vr_ - vl_) / b1_);

  // the velocity of center of the slamdog
  vc1_ << (vl_ + vr_) / 2, 0, 0;

  // the velocity of joint vm
  // std::cout << "---------------------" << std::endl;
  vm_ = vc1_ + Skew(w1) * Eigen::Vector3d(-l1_, 0, 0);

  // the icp of the trailer
  Eigen::Matrix2d A;
  A.block(0, 0, 1, 2) = vm_.block(0, 0, 2, 1).normalized().transpose();
  A.block(1, 0, 1, 2) = Eigen::Vector2d(-cos(theta_curr_), -sin(theta_curr_)).transpose();

  Eigen::Vector3d icp = Eigen::Vector3d::Zero();
  icp.block(0, 0, 2, 1) = A.inverse() * Eigen::Vector2d(0, l2_);

  // the angular velocity of the trailer
  Eigen::Vector3d w2(0, 0, vm_[0] / icp[1]);

  // the velocity of the center of the trailer
  vc2_ = Skew(w2) * Eigen::Vector3d(-l2_ * cos(theta_curr_) - icp[0], -l2_ * sin(theta_curr_) - icp[1], 0);

  // the predicted position of center 1, center 2, middle joint
  double dt = (t_curr_ - t_last_).toSec();
  Eigen::Vector2d pc1(vc1_[0] * dt, vc1_[1] * dt);
  Eigen::Vector2d pm(-l1_ + vm_[0] * dt, vm_[1] * dt);
  Eigen::Vector2d pc2(-l1_ - l2_ * cos(theta_curr_) + vc2_[0] * dt, -l2_ * sin(theta_curr_) + vc2_[1] * dt);

  Eigen::Vector2d m_c1 = pc1 - pm;
  Eigen::Vector2d c2_m = pm - pc2;

  double radian = acos(m_c1.dot(c2_m) / (m_c1.norm() * c2_m.norm()));

  // normiliza to -pi ~ pi
  while (radian < -M_PI) {
    radian = radian + 2 * M_PI;
  }

  while (radian > M_PI) {
    radian = radian - 2 * M_PI;
  }

  theta_pred_ = radian;
}

}  // namespace my_aruco
