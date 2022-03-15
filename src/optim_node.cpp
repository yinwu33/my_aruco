#include <string>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "my_aruco/optim/kalman_filter_ddrobot.h"

using namespace my_aruco;


int main(int argc, char** argv) {
  ros::init(argc, argv, "optim_node");

  ros::NodeHandle nh, nh_private("~");

  std::string configFile = nh_private.param<std::string>("config_file", "/home/ubuntu/Workspace/KIT/slamdog/my_aruco_ws/src/my_aruco/config/sim_slamdog.yaml");
  if (configFile == "") {
    std::cerr << "must speicifca config file path";
    return EXIT_FAILURE;
  }

  cv::FileStorage fs(configFile, cv::FileStorage::READ);

  cv::FileNode node = fs["kalman_filter_ddrobot"];

  ros::Rate rate(30); // ! debug

  KalmanFilterDDRobot optim(nh, node);

  while (ros::ok()) {
    ros::spinOnce();
    optim.Run();
    rate.sleep();
  }
}