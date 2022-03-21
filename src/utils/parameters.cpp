#include "my_aruco/utils/parameters.h"


namespace my_aruco
{
Parameters::Parameters(const cv::FileNode& node) {
  if ((int)node["mode"] == 0)
    mode = Mode::ARUCO_1D;

  if ((int)node["detector"] == 0)
    detector = Detector::OPENCV;

  if ((int)node["optimizer"] == 0)
    optimizer = Optimizer::MOVING_AVG;

  node["K"] >> cameraMatrix;
  node["D"] >> distCoeffs;

  markerSize = (double)node["marker_size"];

  // the parameters to calculate yaw angle
  cv::Mat rotateVector_temp;
  node["rotation_vector"] >> rotateVector_temp;
  rotateVector << rotateVector_temp.at<double>(0, 0), rotateVector_temp.at<double>(0, 1), rotateVector_temp.at<double>(0, 2);

  projectPlane.first = (int)node["plane_vector_1"];
  projectPlane.second = (int)node["plane_vector_2"];

}

Parameters::Parameters(const std::string& configFile) {
  // todo
}

Parameters::Logging() {
  // todo
}

} // namespace my_aruco
