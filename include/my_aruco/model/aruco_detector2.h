#ifndef ARUCO_DETECTOR2_H
#define ARUCO_DETECTOR2_H

#include <deque>
#include <vector>
#include <string>

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
#include <tf/transform_datatypes.h>

#include "my_aruco/subscriber/image_subscriber.h"
#include "my_aruco/types/image_stamped.hpp"

#include "aruco/aruco.h"

namespace my_aruco {
class ArucoDetector2 {
public:
  ArucoDetector2(cv::FileStorage& fs, ImageSubscriber::Ptr pImageSub, ros::NodeHandle& nh);

  bool Detect();

  bool PoseEstimate();

  void PosePublish();

  void FilterOutlier();

  void ImagePublish();

  bool Run();

  double getYaw() { return yaw_; }

  ros::Time getTime() { return timestamp_; }

private:
  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;
  ros::Publisher aruco_image_pub_;

  ImageSubscriber::Ptr pImageSub_;

  geometry_msgs::PoseArray poseArray_;

  sensor_msgs::ImagePtr aruco_image_;
  
  cv::FileStorage fs_;

  std::deque<ImageStamped::Ptr> dqBuffer_;

  // std::vector<int> markerIds_;
  // std::vector<std::vector<cv::Point2f>> markerCorners_, rejectedCandidates_;
  std::vector<cv::Vec3d> rvecs_, tvecs_;

  // cv::Ptr <cv::aruco::DetectorParameters> parameters_;
  // cv::Ptr<cv::aruco::Dictionary> dictionary_;

  aruco::CameraParameters camParam_;
  aruco::MarkerDetector detector_;
  std::vector<aruco::Marker> markers_;

  // * instinsic parameters
  cv::Mat cameraMatrix_, distCoeffs_;

  cv::Mat image_;

  double markerSize_;

  std::vector<Eigen::Quaterniond> qList_;
  
  double yaw_;
  std::vector<double> yawList_;

  ros::Time timestamp_;
  bool doPosePublish_ = false;
};

}

#endif // ARUCO_DETECTOR2_H