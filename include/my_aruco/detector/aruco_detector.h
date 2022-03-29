#pragma once

#include "my_aruco/utils/parameters.h"
#include "my_aruco/types/markers.h"
#include "my_aruco/types/image_stamped.hpp"
#include "my_aruco_msg/AngleStamped.h"

#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

namespace my_aruco::detector
{

class ArucoDetector {
public:

  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Ptr = std::unique_ptr<ArucoDetector>;

  ArucoDetector() = delete;
  ArucoDetector(const my_aruco::Parameters& p, const ros::NodeHandle&);

  virtual void Run();

  virtual bool Detect() = 0;
  virtual bool PoseEstimate() = 0;

  void GetYaw(double& angle);
  void GetResult(Eigen::Vector3d&) {}; // todo
  void GetResult(Eigen::Matrix4d&) {}; // todo

protected:
  Parameters p_;

  ros::NodeHandle nh_;
  ros::Publisher measurementPub_;
  ros::Subscriber imageSub_;

  ImageStamped currImage_;
  Markers markers_;


private:
  void ImageCallback(const sensor_msgs::Image::Ptr&);
};

} // namespace my_aruco::detector
