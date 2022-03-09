#ifndef MY_ARUCO_IMAGE_STAMPED_HPP
#define MY_ARUCO_IMAGE_STAMPED_HPP

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

namespace my_aruco
{
struct ImageStamped {
  typedef std::shared_ptr<ImageStamped> Ptr;
  ImageStamped();
  ImageStamped(ros::Time timestamp, std::shared_ptr<cv::Mat> pImage) : timestamp(timestamp), pImage(pImage) {}


  double time; // ! to be deleted
  ros::Time timestamp;
  std::shared_ptr<cv::Mat> pImage;
};

} // namespace my_aruco

#endif // MY_ARUCO_IMAGE_STAMPED_HPP