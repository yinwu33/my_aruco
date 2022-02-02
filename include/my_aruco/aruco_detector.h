#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include <deque>
#include <vector>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/aruco.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

namespace my_aruco {
class ArucoDetector {
public:
  ArucoDetector();

  ArucoDetector(ros::NodeHandle& nh);

  bool Detect(std::deque<std::shared_ptr<cv::Mat>>& dq_buffer);

  bool PoseEstimate();

  void Publish();

  void Optim();

private:
  bool do_publish_ = false;

  std::vector<int> markerIds_;
  std::vector<std::vector<cv::Point2f>> markerCorners_, rejectedCandidates_;

  cv::Ptr <cv::aruco::DetectorParameters> parameters_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  // * instinsic parameters
  cv::Mat cameraMatrix_, distCoeffs_;

  //
  cv::Mat image_;

  // * for ros publisher
  ros::Publisher pose_pub_;
  ros::Publisher aruco_image_pub_;


  geometry_msgs::PoseArray poseArray_;
  sensor_msgs::ImagePtr aruco_image_;
  std::string frame_id_;

  double markerSideLength_;
};

}

#endif // ARUCO_DETECTOR_H