#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "my_aruco_msg/AngleStamped.h"


class PIDController {
public:
  PIDController(const ros::NodeHandle& nh) : nh_(nh) {
    cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    angleSub_ = nh_.subscribe("measurenment", 100, &PIDController::MsgCallback, this);

    last_time_ = ros::Time(0);

    controlMsg_.linear.x = 0.2;
  }

  void SetPID(double p, double i, double d) {
    kp_ = p;
    ki_ = i;
    kd_ = d;
  }

  void MsgCallback(const my_aruco_msg::AngleStamped::Ptr& msg) {
    if (last_time_ == ros::Time(0)) {
      last_time_ = msg->header.stamp;
      return;
    }

    ros::Duration dstamp = msg->header.stamp - last_time_;
    double dt = dstamp.toSec();
    last_time_ = msg->header.stamp;

    currAngle_ = msg->radian;

    double error = goalAngle_ - currAngle_;

    // P term
    pTerm = kp_ * error;

    // I term
    iTerm = iTerm + ki_ * error * dt;

    // D term
    if (dt != 0) {
      dTerm = kd_ * (error - lastError_) / dt;
      lastError_ = error;
    }

    // final output
    double w = pTerm + iTerm + dTerm;

    controlMsg_.angular.z = w;

    cmdVelPub_.publish(controlMsg_);

  }

private:
  double kp_, ki_, kd_;
  double defaultVel_ = 0.2;

  double pTerm = 0.0;
  double iTerm = 0.0;
  double dTerm = 0.0;

  ros::NodeHandle nh_;
  ros::Publisher cmdVelPub_;
  ros::Subscriber angleSub_;

  ros::Time last_time_;
  geometry_msgs::Twist controlMsg_;

  double lastError_ = 0.0;

  double goalAngle_ = 0.0;
  double currAngle_ = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pid_controller");
  ros::NodeHandle nh;

  PIDController pid(nh);

  ros::spin();
}