#pragma once

#include "my_aruco/common_include.hpp"

namespace my_aruco
{

enum class Detector {
  OPENCV = 0
};

enum class Optimizer {
  MOVING_AVG = 0
};

enum class Mode {
  ARUCO_1D = 0
};

struct Parameters {
  Parameters() = default;
  Parameters(const cv::FileNode& node);

  Mode mode{Mode::ARUCO_1D};

  Detector detector{Detector::OPENCV};

  Optimizer optimizer{Optimizer::MOVING_AVG};

};
  
} // namespace my_aruco
