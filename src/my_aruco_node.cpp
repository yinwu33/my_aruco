#include <iostream>

#include <opencv2/core.hpp>

#include "my_aruco/subscriber/image_subscriber.h"
#include "my_aruco/model/aruco_detector.h"
#include "my_aruco/optim/kalman_filter.h"

#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>

using namespace my_aruco;

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_aruco_node");

  ros::NodeHandle nh, nh_private("~");

  // load parameters
  bool doDebug = nh_private.param<bool>("doDebug", false); // todo
  std::string image_topic = nh_private.param<std::string>("image_topic", "image_raw");
  std::string config_file = nh_private.param<std::string>("config_file", "");
  int fps = nh_private.param<int>("fps", 30);
  // bool use_degree = nh.param<bool>("use_degree", false); // ! to be deleted


  // todo
  ros::Publisher measurement_pub = nh.advertise<std_msgs::Float64>("measurement", 100);
  ros::Publisher estimation_pub = nh.advertise<std_msgs::Float64>("estimation", 100);
  std_msgs::Float64 measurement;
  std_msgs::Float64 estimation;
  // geometry_msgs::PointStamped estimation;

  // load configuration file
  if (config_file.size() == 0) {
    std::cerr << "Config File shouldn't be empty!" << std::endl;
    return EXIT_FAILURE;
  }

  cv::FileStorage fs = cv::FileStorage();

  if (!fs.open(config_file, cv::FileStorage::READ)) {
    std::cerr << "Config File " << config_file << " doesn't exist, Please check your file path" << std::endl;
    return EXIT_FAILURE;
  }

  // create instances
  ImageSubscriber::Ptr pImageSub = std::make_shared<ImageSubscriber>(nh, image_topic, 100);
  ArucoDetector arucoDetector(fs, pImageSub, nh);
  KalmanFilter filter(1, 1, 1); // todo use config file with cv::File


  ros::Rate rate(fps); // to be deleted

  // run
  while (ros::ok()) {
    ros::spinOnce();

    if (!arucoDetector.Run())
      continue;

    double yaw_mea = arucoDetector.getYaw();

    // ! constant velocity KF
    ros::Time time = arucoDetector.getTime();

    filter.Update(yaw_mea, time);

    double yaw_est = filter.getState();

    // if (use_degree) { // ! to be deleted
    //   yaw_mea = yaw_mea * 180 / M_PI;
    //   yaw_est = yaw_est * 180 / M_PI;
    // }

    measurement.data = yaw_mea;
    estimation.data = yaw_est;

    measurement_pub.publish(measurement);
    estimation_pub.publish(estimation);
  }
}