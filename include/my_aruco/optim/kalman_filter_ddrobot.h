#ifndef MY_ARUCO_KALMAN_FILTER_DDROBOT_H
#define MY_ARUCO_KALMAN_FILTER_DDROBOT_H

#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

// #include "my_aruco/types/velocity.hpp"

namespace my_aruco
{

class KalmanFilterDDRobot {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  KalmanFilterDDRobot() = delete;
  KalmanFilterDDRobot(ros::NodeHandle& nh, cv::FileNode& node);

  void LeftVelCallback(const std_msgs::Float64::ConstPtr& msg);
  void RightVelCallback(const std_msgs::Float64::ConstPtr& msg);
  void MeasurenmentCallback(const std_msgs::Float64::ConstPtr& msg);

  // bool ReadVel();

  bool Update();

  void Run();

  void Publish();

  void Predict();
private:

  bool debug_ = false;

  cv::FileNode node_;

  ros::NodeHandle nh_;
  ros::Subscriber leftVelSub_;
  ros::Subscriber rightVelSub_;
  ros::Subscriber measurementSub_;
  ros::Publisher predictPub_;
  ros::Publisher estimationPub_;

  /**
   * @brief velocity
   * v1: front left
   * v2: front right
   * v3: middle
   * v4: back left
   * v5=: back right
   */
  double radius_;
  double vl_ = 0.0;
  double vr_ = 0.0;
  Eigen::Vector3d  vm_, vc1_, vc2_;
  double measurement_;
  bool measurementIsNew_ = false;
  bool vlIsNew_ = false;
  bool vrIsNew_ = false;

  std_msgs::Float64 prediction_;
  std_msgs::Float64 estimation_;

  // slamdog modell parameters
  double b1_, b2_, l1_, l2_;

  bool initialized_;

  double A_, B_, C_, D_;

  double P0_, P_, Q_, R_;

  double K_;

  double theta_last_, theta_curr_;
  double theta_pred_, theta_measure_;

  ros::Time t_last_, t_curr_;
};

} // namespace my_aruco


#endif // MY_ARUCO_KALMAN_FILTER_DDROBOT_H