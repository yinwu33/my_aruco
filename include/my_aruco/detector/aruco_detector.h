#pragma once

#include "my_aruco/utils/parameters.h"
#include "my_aruco/types/markers.h"
#include "my_aruco/types/image_stamped.hpp"

namespace my_aruco::detector
{

class ArucoDetector {
public:
  ArucoDetector() = delete;
  ArucoDetector(const my_aruco::Parameters& p);

  virtual bool Detect(const my_aruco::ImageStamped) = 0;
  virtual bool PoseEstimate() = 0;

  void GetAvgResult();

  virtual void GetResult(double&);
  virtual bool GetResult(Eigen::Vector3d&);
  virtual bool GetResult(Eigen::Matrix4d&);


protected:
  Parameters p_;

  Markers markers_;
};
  
} // namespace my_aruco::detector
