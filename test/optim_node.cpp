#include "my_aruco/optim/aruco_optimizer_factory.hpp"


int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_optim_node");

  ros::NodeHandle nh;

  std::string configFile = nh.param<std::string>("config_file", "");

  if (configFile == "") {
    std::cerr << "Please give a config file path";
    return EXIT_FAILURE;
  }

  cv::FileStorage fs(configFile, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Please check the config file path";
    return EXIT_FAILURE;
  }

  my_aruco::Parameters p(fs);

  my_aruco::optim::ArucoOptimizer::Ptr pOptimizer = my_aruco::optim::create(p, nh);

  ros::Rate rate(p.fps);

  while (ros::ok()) {
    ros::spinOnce();
    pOptimizer->Run();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}