#ifndef MY_ARUCO_IMAGE_SUBSCRIBER_H
#define MY_ARUCO_IMAGE_SUBSCRIBER_H

#include <deque>
#include <string>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

namespace my_aruco
{
class ImageSubscriber {
public:
  ImageSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buffer_size);

  void MsgCallback(const sensor_msgs::ImageConstPtr& msg);

  void ParseData(std::deque<std::shared_ptr<cv::Mat>>& dq_buffer);

private:
  ros::NodeHandle nh_;
  std::shared_ptr<ros::Subscriber> p_Image_sub_;

  std::deque<std::shared_ptr<cv::Mat>> dq_image_buffer_;
  size_t buffer_size_;
  std::mutex m_;

};

} // namespace my_aruco



#endif // MY_ARUCO_IMAGE_SUBSCRIBER_H