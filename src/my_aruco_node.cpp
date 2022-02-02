#include <iostream>

#include <my_aruco/image_subscriber.h>
#include <my_aruco/aruco_detector.h>

using namespace my_aruco;

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_aruco_node");

  ros::NodeHandle nh;

  ImageSubscriber image_sub_(nh, "camera_image", 100);
  ArucoDetector aruco_detector(nh);


  std::deque<std::shared_ptr<cv::Mat>> dq_buffer;

  while (ros::ok()) {
    ros::spinOnce();

    image_sub_.ParseData(dq_buffer);

    if (!aruco_detector.Detect(dq_buffer))
      continue;
    if (!aruco_detector.PoseEstimate())
      continue;

  }
}