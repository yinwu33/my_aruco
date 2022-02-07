#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include <deque>
#include <vector>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/aruco.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "my_aruco/image_subscriber.h"

namespace my_aruco {
class ArucoDetector {
public:
  ArucoDetector(std::shared_ptr<cv::FileStorage> fs, ImageSubscriber::Ptr pImageSub);

  ArucoDetector(std::shared_ptr<cv::FileStorage> fs, ImageSubscriber::Ptr pImageSub, ros::NodeHandle& nh);

  void Initialize();

  bool Detect();

  bool PoseEstimate();

  void Publish();

  void Optim();

  void Run();

private:

  std::vector<int> markerIds_;
  std::vector<std::vector<cv::Point2f>> markerCorners_, rejectedCandidates_;
  std::vector<cv::Vec3d> rvecs_, tvecs_;

  cv::Ptr <cv::aruco::DetectorParameters> parameters_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  // * instinsic parameters
  cv::Mat cameraMatrix_, distCoeffs_;

  //
  cv::Mat image_;

  double markerSize_;

  std::deque<std::shared_ptr<cv::Mat>> dqBuffer_;

  // image subscriber
  // std::shared_ptr<ImageSubscriber> pImageSub_;
  ImageSubscriber::Ptr pImageSub_;

  // config file
  std::shared_ptr<cv::FileStorage> fs_;

  // ROS
  bool bDoPublish_ = false;

  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;
  ros::Publisher aruco_image_pub_;

  geometry_msgs::PoseArray poseArray_;
  sensor_msgs::ImagePtr aruco_image_;
  std::string frameId;
};

}

#endif // ARUCO_DETECTOR_H