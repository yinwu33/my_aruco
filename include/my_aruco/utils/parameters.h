#pragma once

#include "my_aruco/common_include.hpp"

namespace my_aruco
{

enum class Detector {
  OPENCV = 0
};

enum class Optimizer {
  NONE = 0,
  MOVING_AVG,
  EKF
};

struct Parameters {

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // ? todo

  Parameters() = default;
  Parameters(const cv::FileStorage& node);
  Parameters(const std::string& configFile);

  cv::FileStorage fs_;

  Detector detector{Detector::OPENCV};

  Optimizer optimizer{Optimizer::NONE};

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

  double markerSize;

  // calculate yaw method
  Eigen::Vector3d rotateVector;
  std::pair<int, int> projectPlane;
  size_t windowSize = 5;

  std::string topicImageRaw;
  std::string topicImageAruco;

  std::string topicRobotState;

  size_t fps;

  // kalman filter parameters
  double P0, Q, R;
  // robot model parameters
  double l1;
  double l2;

public:
  void Logging();
};
  
} // namespace my_aruco
