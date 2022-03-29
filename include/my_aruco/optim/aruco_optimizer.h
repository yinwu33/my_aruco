#pragma once

#include "my_aruco/utils/parameters.h"
#include "my_aruco/types/markers.h"
#include "my_aruco_msg/AngleStamped.h"

namespace my_aruco::optim
{

class ArucoOptimizer {
public:

  using Ptr = std::unique_ptr<ArucoOptimizer>;

  ArucoOptimizer() = delete;
  ArucoOptimizer(const Parameters&, const ros::NodeHandle&);

  virtual void Run() = 0;

  virtual void Update1D() {};
  virtual void Update3D() {};
  virtual void Update6D() {};

protected:
  Parameters p_;

  ros::NodeHandle nh_;
  ros::Publisher estimationPub_;
  ros::Subscriber measurementSub_;

  my_aruco_msg::AngleStamped angle_;

  bool newMeasurement_;

private:
  void MsgCallback1D(const my_aruco_msg::AngleStamped::Ptr& msg);
};

} // namespace my_aruco
