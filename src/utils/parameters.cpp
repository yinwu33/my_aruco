#include "my_aruco/utils/parameters.h"

#include <unordered_map>


namespace my_aruco
{
Parameters::Parameters(const cv::FileStorage& node) {

  if ((int)node["detector"] == 0)
    detector = Detector::OPENCV;

  if ((int)node["optimizer"] == 0)
    optimizer = Optimizer::NONE;
  else if ((int)node["optimizer"] == 1)
    optimizer = Optimizer::MOVING_AVG;
  else if ((int)node["optimizer"] == 2)
    optimizer = Optimizer::EKF;

  node["K"] >> cameraMatrix;
  node["D"] >> distCoeffs;

  topicImageRaw = (std::string)node["topic_image_raw"];
  // topicImageAruco = (std::string)node["topic_image_aruco"];
  topicRobotState = (std::string)node["topic_robot_state"];

  markerSize = (double)node["marker_size"];

  windowSize = (int)node["window_size"];
  // the parameters to calculate yaw angle
  cv::Mat rotateVector_temp;
  node["rotation_vector"] >> rotateVector_temp;
  rotateVector << rotateVector_temp.at<float>(0, 0), rotateVector_temp.at<float>(0, 1), rotateVector_temp.at<float>(0, 2);

  projectPlane.first = (int)node["plane_vector_1"];
  projectPlane.second = (int)node["plane_vector_2"];

  fps = (int)node["fps"];

  // * kalman filter parameters
  l1 = (double)node["l1"];
  l2 = (double)node["l2"];
  P0 = (double)node["P0"];
  Q = (double)node["Q"];
  R = (double)node["R"];
}

Parameters::Parameters(const std::string& configFile) {
  if (!fs_.open(configFile, cv::FileStorage::READ)) {
    throw std::invalid_argument(" config file not exists");
  }

  // if (!pFs->isOpened()) {
  // // if (!fs.isOpened()) {
  // }

  // todo: may be wrong, due to local value
  Parameters(fs_);
}

void Parameters::Logging() {
  std::cout << "==================== Parameter setting ====================\n" << std::endl;



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
  case Optimizer::EKF:
    std::cout << "Optimizer: Extend Kalman Filter" << std::endl;

  default:
    break;
  }

  std::cout << "Camera Matrix: \n" << cameraMatrix << std::endl;
  std::cout << "Distortion Coeffience: " << distCoeffs << std::endl;
  std::cout << "Marker size: " << markerSize << std::endl;
  std::cout << "window size: " << windowSize << std::endl;
  std::cout << "Vector will be rotated: " << rotateVector.transpose() << std::endl;

  std::unordered_map<int, char> numberToAxis{ std::pair<int, char>{1, 'x'},
    std::pair<int, char>{2, 'y'}, std::pair<int, char>{3, 'z'} };

  std::cout << "Angle will be measured in plane: " << numberToAxis[projectPlane.first]
    << " and " << numberToAxis[projectPlane.second] << std::endl;

  std::cout << "image raw topic: " << topicImageRaw << std::endl;
  // std::cout << "image aruco topic: " << topicImageAruco << std::endl;
  std::cout << "robot state topic: " << topicRobotState << std::endl;
  

  std::cout << "fps: " << fps << std::endl;
  
  // kalman filter parameters
  std::cout << "l1: " << l1 << std::endl;
  std::cout << "l2: " << l2 << std::endl;
  std::cout << "P0: " << P0 << std::endl;
  std::cout << "Q: " << Q << std::endl;
  std::cout << "R: " << R << std::endl;
  std::cout << "==================== Logging Finished ====================\n" << std::endl;

}

} // namespace my_aruco
