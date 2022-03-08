#ifndef MY_ARUCO_CAMERA_INFO_SUBSCRIBER_H
#define MY_ARUCO_CAMERA_INFO_SUBSCRIBER_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>

namespace my_aruco {
class CameraInfoSubscriber {
public:
  CameraInfoSubscriber() = delete;
  CameraInfoSubscriber(ros::NodeHandle& nh, std::string topic_name);

  void MsgCallback(const sensor_msgs::CameraInfoConstPtr& msg);

  void ParseData(cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

private:
  ros::NodeHandle nh_;
  std::shared_ptr<ros::Subscriber> pCameraInfoSub_;
  sensor_msgs::CameraInfo cinfo_;
};
}

#endif // MY_ARUCO_CAMERA_INFO_SUBSCRIBER_H