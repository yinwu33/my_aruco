#ifndef MY_ARUCO_IMAGE_SUBSCRIBER_H
#define MY_ARUCO_IMAGE_SUBSCRIBER_H

#include <deque>
#include <string>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include "my_aruco/types/image_stamped.hpp"

namespace my_aruco
{
class ImageSubscriber {
public:
  typedef std::shared_ptr<ImageSubscriber> Ptr;
public:
  ImageSubscriber() = delete;
  ImageSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buffer_size);

  void MsgCallback(const sensor_msgs::ImageConstPtr& msg);

  void ParseData(std::deque<ImageStamped::Ptr>& dq_buffer);

private:
  ros::NodeHandle nh_;
  ros::Subscriber imageSub_;

  std::deque<ImageStamped::Ptr> imageBuffer_;
  size_t buffer_size_;
  std::mutex m_;

};

} // namespace my_aruco
 


#endif // MY_ARUCO_IMAGE_SUBSCRIBER_H