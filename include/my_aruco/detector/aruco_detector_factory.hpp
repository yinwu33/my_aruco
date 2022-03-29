#pragma once

#include "my_aruco/detector/aruco_detector.h"
#include "my_aruco/detector/aruco_detector_opencv.h"

namespace my_aruco::detector
{
std::unique_ptr<ArucoDetector> create(const Parameters& p, const ros::NodeHandle& nh) {
  switch (p.detector)
  {
  case my_aruco::Detector::OPENCV:
    return std::make_unique<ArucoDetectorOpenCV>(p, nh);

  default:
    return nullptr;
  }
}

} // namespace my_aruco::detector
