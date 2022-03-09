#ifndef MY_ARUCO_IMAGE_STAMPED_HPP
#define MY_ARUCO_IMAGE_STAMPED_HPP

#include <opencv2/opencv.hpp>

namespace my_aruco
{
struct ImageStamped {
  typedef std::shared_ptr<ImageStamped> Ptr;
  ImageStamped();
  ImageStamped(double timestamp, std::shared_ptr<cv::Mat> pImage) : timestamp(timestamp), pImage(pImage) {}


  double timestamp;
  std::shared_ptr<cv::Mat> pImage;
};

} // namespace my_aruco

#endif // MY_ARUCO_IMAGE_STAMPED_HPP