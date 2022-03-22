#include "my_aruco/utils/parameters.h"

#include <unordered_map>


namespace my_aruco
{
Parameters::Parameters(const cv::FileStorage& node) {
  
  if ((int)node["mode"] == 0)
    mode = Mode::ARUCO_1D;

  if ((int)node["detector"] == 0)
    detector = Detector::OPENCV;

  if ((int)node["optimizer"] == 0)
    optimizer = Optimizer::MOVING_AVG;

  node["K"] >> cameraMatrix;
  node["D"] >> distCoeffs;

  topicImageRaw = (std::string)node["topic_image_raw"];
  topicImageAruco = (std::string)node["topic_image_aruco"];

  markerSize = (double)node["marker_size"];

  // the parameters to calculate yaw angle
  cv::Mat rotateVector_temp;
  node["rotation_vector"] >> rotateVector_temp;
  rotateVector << rotateVector_temp.at<float>(0, 0), rotateVector_temp.at<float>(0, 1), rotateVector_temp.at<float>(0, 2);

  projectPlane.first = (int)node["plane_vector_1"];
  projectPlane.second = (int)node["plane_vector_2"];

}

Parameters::Parameters(const std::string& configFile) {
  cv::FileStorage fs(configFile, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    throw std::invalid_argument(" config file not exists");
  }

  // todo: may be wrong, due to local value
  Parameters(fs.getFirstTopLevelNode());
}

void Parameters::Logging() {
  std::cout << "==================== Parameter setting ====================\n" << std::endl;

  switch (mode)
  {
  case Mode::ARUCO_1D:
    std::cout << "Mode: 1D angle" << std::endl;
    break;

  default:
    break;
  }

  switch (detector)
  {
  case Detector::OPENCV:
    std::cout << "Detector: Aruco detector from OpenCV" << std::endl;
    break;

  default:
    break;
  }

  switch (optimizer)
  {
  case Optimizer::MOVING_AVG:
    std::cout << "Optimizer: Moving Average" << std::endl;
    break;

  default:
    break;
  }

  std::cout << "Camera Matrix: \n" << cameraMatrix << std::endl;
  std::cout << "Distortion Coeffience: " << distCoeffs << std::endl;
  std::cout << "Marker size: " << markerSize << std::endl;
  std::cout << "For 1D Mode: " << std::endl;
  std::cout << "Vector will be rotated: " << rotateVector.transpose() << std::endl;

  std::unordered_map<int, char> numberToAxis{ std::pair<int, char>{1, 'x'},
    std::pair<int, char>{2, 'y'}, std::pair<int, char>{3, 'z'} };

  std::cout << "Angle will be measured in plane: " << numberToAxis[projectPlane.first]
    << " and " << numberToAxis[projectPlane.second] << std::endl;

  std::cout << "image raw topic: " << topicImageRaw << std::endl;
  std::cout << "image aruco topic: " << topicImageAruco << std::endl;
  std::cout << "==================== Logging Finished ====================\n" << std::endl;

}

} // namespace my_aruco
