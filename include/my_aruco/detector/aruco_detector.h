#pragma once

#include "my_aruco/utils/parameters.h"
#include "my_aruco/types/markers.h"
#include "my_aruco/types/image_stamped.hpp"

namespace my_aruco::detector
{

class ArucoDetector {
public:

  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Ptr = std::unique_ptr<ArucoDetector>; 

  ArucoDetector() = delete;
  ArucoDetector(const my_aruco::Parameters& p);

  virtual bool Detect(my_aruco::Markers& markers) = 0;
  virtual bool PoseEstimate(my_aruco::Markers& markers) = 0;

  void GetAvgResult();

  virtual void GetYaw(my_aruco::Markers& markers);
  virtual void GetResult(Eigen::Vector3d&) {}; // todo
  virtual void GetResult(Eigen::Matrix4d&) {}; // todo


protected:
  Parameters p_;

};
  
} // namespace my_aruco::detector
