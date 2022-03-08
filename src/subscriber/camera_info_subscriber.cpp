#include <subscriber/camera_info_subscriber.h>

namespace my_aruco {
CameraInfoSubscriber::CameraInfoSubscriber(ros::NodeHandle& nh, std::string topic_name)
  : nh_(nh) {
    pCameraInfoSub_ = std::make_shared<ros::Subscriber>(nh_.subscribe(topic_name, 10, &CameraInfoSubscriber::MsgCallback, this));
}

void CameraInfoSubscriber::MsgCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
  cinfo_ = *msg;
}

void CameraInfoSubscriber::ParseData(cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
  // todo
}
}