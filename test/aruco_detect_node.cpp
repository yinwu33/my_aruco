#include "my_aruco/detector/aruco_detector_opencv.h"
#include "my_aruco/optim/aruco_optimizer_moving_avg.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_detect_node");

  ros::NodeHandle nh, nh_private("~");

  std::string configFile = nh_private.param<std::string>("config_file", "");

  if (configFile == "") {
    std::cerr << "Please give a config file path";
    return EXIT_FAILURE;
  }

  cv::FileStorage fs(configFile, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Please check the config file path";
    return EXIT_FAILURE;
  }

  std::cout << "==============" << std::endl;
  my_aruco::Parameters p(fs);

  my_aruco::detector::ArucoDetectorOpenCV detector(p, nh);

  my_aruco::optim::ArucoOptimizerMovingAvg optimizer(p, nh);


  ros::Rate rate(p.fps);

  while (ros::ok()) {
    detector.Run();
    optimizer.Run();

    rate.sleep();
  }

  return EXIT_SUCCESS;
}