#include <iostream>

#include <opencv2/core.hpp>

#include "my_aruco/image_subscriber.h"
#include "my_aruco/aruco_detector.h"

using namespace my_aruco;

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_aruco_node");

  ros::NodeHandle nh, nh_private("~");

  // parse config file
  std::string config_file = nh_private.param<std::string>("config_file", "");

  if (config_file.size() == 0) {
    std::cerr << "Config File shouldn't be empty!" << std::endl;
    return EXIT_FAILURE;
  }

  std::shared_ptr<cv::FileStorage> fs = std::make_shared<cv::FileStorage>();

  if (!fs->open(config_file, cv::FileStorage::READ)) {
    std::cerr << "Config File " << config_file << " doesn't exist, Please check your file path" << std::endl;
    return EXIT_FAILURE;
  }

  // run
  ImageSubscriber::Ptr pImageSub = std::make_shared<ImageSubscriber>(nh, "camera_image", 100);
  ArucoDetector arucoDetector(fs, pImageSub, nh);

  std::deque<std::shared_ptr<cv::Mat>> dq_buffer;

  while (ros::ok()) {
    ros::spinOnce();

    arucoDetector.Run();
  }
}