#pragma once

#include "my_aruco/utils/parameters.h"
#include "my_aruco/types/markers.h"

namespace my_aruco::optim
{

class ArucoCalibrator {
public:
  ArucoCalibrator();
  ArucoCalibrator(const Parameters&);

  bool Valid();

  void Calib();

private:

  Parameters p_;

  cv::Ptr<cv::aruco::Board> pBoard_;
  cv::Size imageSize_;

  std::vector<std::vector<cv::Point2f>> allCornersConcatenated_;
  std::vector<int> allIdsConcateneated_;
  std::vector<int> markerCounterPerFrame_;

  // int calibrationFLags_ = cv::calibrateCamera::FL
};
  
} // namespace my_aruco:optim
