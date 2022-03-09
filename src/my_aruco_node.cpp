#include <iostream>

#include <opencv2/core.hpp>

#include "my_aruco/subscriber/image_subscriber.h"
#include "my_aruco/aruco_detector.h"
#include "my_aruco/optim/kalman_filter.h"

using namespace my_aruco;

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_aruco_node");

  ros::NodeHandle nh, nh_private("~");

  // load parameters
  bool doDebug = nh_private.param<bool>("doDebug", false); // todo
  std::string image_topic = nh_private.param<std::string>("image_topic", "camera_image");
  std::string config_file = nh_private.param<std::string>("config_file", "");

  // load configuration file
  if (config_file.size() == 0) {
    std::cerr << "Config File shouldn't be empty!" << std::endl;
    return EXIT_FAILURE;
  }

  std::shared_ptr<cv::FileStorage> fs = std::make_shared<cv::FileStorage>();

  if (!fs->open(config_file, cv::FileStorage::READ)) {
    std::cerr << "Config File " << config_file << " doesn't exist, Please check your file path" << std::endl;
    return EXIT_FAILURE;
  }

  // create instances
  ImageSubscriber::Ptr pImageSub = std::make_shared<ImageSubscriber>(nh, image_topic, 100);
  ArucoDetector arucoDetector(fs, pImageSub, nh);
  KalmanFilter filter(1, 1, 1); // todo use config file with cv::File

  std::deque<std::shared_ptr<cv::Mat>> dq_buffer;  // ! to be deleted

  // run
  while (ros::ok()) {
    ros::spinOnce();

    if (!arucoDetector.Run())
      continue;

    double measurement = arucoDetector.getYaw();

    filter.Update(measurement, arucoDetector.getTime());

    double estimate = filter.getState();

    std::cout << "measurement: " << measurement << "\nfiltered: " << estimate << "\ndiffer:" << (estimate - measurement) << std::endl << std::endl;
  }
}