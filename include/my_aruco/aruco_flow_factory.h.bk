#pragma once

#include "my_aruco/utils/parameters.h"
#include "my_aruco/detector/aruco_detector_factory.hpp"
#include "my_aruco/optim/aruco_optimizer_factory.hpp"
#include "my_aruco/types/markers.h"
#include "my_aruco_msg/AngleStamped.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

namespace my_aruco
{

class ArucoFlowFactory {
public:

  using Ptr = std::shared_ptr<ArucoFlowFactory>;

  static Ptr Create(const std::string& configFile);

  void Run();

private:
  ArucoFlowFactory() = delete;
  ArucoFlowFactory(const std::string& configFile);

  void ImageCallback(const sensor_msgs::ImagePtr& msg);

  void PublishImage();

  bool Detect();

  static std::shared_ptr<ArucoFlowFactory> pInstance_;

private:
  my_aruco::Parameters p_;

  ros::NodeHandle nh_;
  ros::Subscriber imageSub_;
  ros::Publisher arucoImagePub_;
  ros::Publisher measurementPub_;
  ros::Publisher estimationPub_;

  sensor_msgs::Image::Ptr pArucoImage_;

  my_aruco::AngleStamped measurementMsg_;
  my_aruco::AngleStamped estimationMsg_;

  my_aruco::detector::ArucoDetector::Ptr pDetector_;
  my_aruco::optim::ArucoOptimizer::Ptr pOptimizer_;

  my_aruco::Markers markers_;

  size_t count_ = 0.0;

  double interval_ = 0.0;

  bool doPubImage = false;
};


} // namespace my_aruco
