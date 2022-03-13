#ifndef MY_ARUCO_VELOCITY_HPP
#define MY_ARUCO_VELOCITY_HPP

#include <ros/ros.h>

namespace my_aruco
{
struct Velocity {
  double vx = 0.0;
  double vy = 0.0;

  inline double abs() { return sqrt(vx * vx + vy * vy); }
};

} // namespace my_aruco


#endif // MY_ARUCO_VELOCITY_HPP