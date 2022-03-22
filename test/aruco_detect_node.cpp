#include "my_aruco/utils/parameters.h"
#include "my_aruco/detector/aruco_detector_factory.hpp"
#include "my_aruco/optim/aruco_optimizer_factory.hpp"

#include <sensor_msgs/Image.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_detect_node");

  ros::NodeHandle nh, nh_private("~");

  std::string configFile = nh_private.param<std::string>("configFile", "");

  if (configFile == "") {
    std::cerr << "Please give a config file path";
    return EXIT_FAILURE;
  }

  cv::FileStorage fs(configFile, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Please check the config file path";
    return EXIT_FAILURE;
  }

  cv::FileNode node = fs.getFirstTopLevelNode();

  // create a image subscriber
  // todo: use my image sub

  // load parameters
  my_aruco::Parameters param(node);

  // create a detector
  auto detector = my_aruco::detector::create(param);

  // create an optimizer
  auto optimzer = my_aruco::optim::create(param);

  ros::Rate rate((int)node["fps"]);

  while (ros::ok()) {
    ros::spinOnce();

    rate.sleep();
  }
}