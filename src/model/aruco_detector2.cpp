#include "my_aruco/model/aruco_detector2.h"

namespace my_aruco
{

// static Eigen::Quaterniond AddOffset(const Eigen::Quaterniond& input) {
//   return input;
// }

static double calculateYaw(const Eigen::Quaterniond& q) {
  Eigen::Vector3d vector = -q.matrix() * Eigen::Vector3d(0, 0, 1);
  return atan2(vector[0], vector[2]);
}

/**
 * @brief Get the Quaternion object in x, y, z, w
 *
 * @param aa
 * @param q
 * @return Eigen::Quaterniond
 */
static Eigen::Quaterniond GetQuaternion(cv::Vec3d aa) {
  double angle = cv::norm(aa);
  double angle_2 = angle / 2;
  return Eigen::Quaterniond(cos(angle_2), aa(0) / angle * sin(angle_2), aa(1) / angle * sin(angle_2), aa(2) / angle * sin(angle_2));
}


ArucoDetector2::ArucoDetector2(cv::FileStorage& fs, ImageSubscriber::Ptr pImageSub, ros::NodeHandle& nh)
  : fs_(fs), pImageSub_(pImageSub), nh_(nh) {
  // ROS Publisher Initialization
  aruco_image_pub_ = nh_.advertise<sensor_msgs::Image>((std::string)fs_["topic_name_aruco_image"], 100);

  // camera parameters initialize
  fs_["K"] >> cameraMatrix_;
  fs_["D"] >> distCoeffs_;

  // aruco markers initialize
  markerSize_ = (double)fs_["marker_size"]; // todo use right marker size
  ROS_INFO("marker size: %f", markerSize_);

  // opencv aruco initialize
  // parameters_ = cv::aruco::DetectorParameters::create();
  // dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  camParam_.setParams(cameraMatrix_, distCoeffs_, cv::Size(640, 480));
  // camParam_.CameraMatrix = cameraMatrix_;
  // camParam_.Distorsion = distCoeffs_;
  // camParam_.CamSize.width = 640;
  // camParam_.CamSize.height = 480;

  detector_.setDetectionMode(aruco::DM_NORMAL, 0.02);

  if (doPosePublish_) {
    ROS_INFO("PoseArray will be published");
    pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>((std::string)fs_["topic_name_marker_poses"], 100);
    poseArray_.header.frame_id = (std::string)fs_["camera_frame_id"];
  }

}


bool ArucoDetector2::Detect() {
  if (dqBuffer_.size() == 0) return false;  // ! return if buffer is clear

  for (auto it = dqBuffer_.begin(); it != dqBuffer_.end(); ++it) {
    // todo make it a list
    timestamp_ = (**it).timestamp;
    image_ = (**it).image.clone();
    // cv::aruco::detectMarkers(image_, dictionary_, markerCorners_, markerIds_, parameters_, rejectedCandidates_);
    markers_.clear();
    detector_.detect(image_, markers_, camParam_, markerSize_, false, false);
  }
  dqBuffer_.clear();
  return true;
}

bool ArucoDetector2::PoseEstimate() {

  if (markers_.size() == 0)
    return false;

  // cv::aruco::estimatePoseSingleMarkers(markerCorners_, markerSize_, cameraMatrix_, distCoeffs_, rvecs_, tvecs_);

  yawList_.clear();
  qList_.clear();
  rvecs_.clear();
  tvecs_.clear();
  for (size_t i = 0; i < markers_.size(); ++i) {
    rvecs_.emplace_back(markers_[i].Rvec);
    tvecs_.emplace_back(markers_[i].Tvec);
    Eigen::Quaterniond q = GetQuaternion(markers_[i].Rvec);

    qList_.emplace_back(q);

    yawList_.emplace_back(calculateYaw(q));
  }

  FilterOutlier();

  return true;
}

void ArucoDetector2::FilterOutlier() {
  if (yawList_.size() >= 4) {
    // remove the largest and the smallest measurement
    // todo: maybe more reasonalbe technology
    std::sort(yawList_.begin(), yawList_.end());
    double sum = 0.0;
    for (size_t i = 0; i < yawList_.size(); ++i) {
      if (i == 0 or i == yawList_.size() - 1)
        continue;

      sum += yawList_[i];
    }
    yaw_ = sum / (yawList_.size() - 2);
  }
  else {
    double sum = 0.0;
    for (const auto& y : yawList_) {
      sum += y;
    }
    yaw_ = sum / yawList_.size();
  }
}

void ArucoDetector2::PosePublish() {
  // Publish poses
  poseArray_.poses.clear();
  for (size_t i = 0; i < qList_.size(); ++i) {
    geometry_msgs::Pose pose;
    poseArray_.header.stamp = timestamp_;

    // GetQuaternion(rvecs_[i], q);
    Eigen::Quaterniond q = qList_[i];
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose.position.x = tvecs_[i][0];
    pose.position.y = tvecs_[i][1];
    pose.position.z = tvecs_[i][2];

    poseArray_.poses.emplace_back(pose);
  }

  pose_pub_.publish(poseArray_);

}

void ArucoDetector2::ImagePublish() {
  // Publish Image
  for (size_t i = 0; i < markers_.size(); ++i) {
    cv::aruco::drawAxis(image_, cameraMatrix_, distCoeffs_, rvecs_[i], tvecs_[i], 0.1);
  }
  aruco_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
  aruco_image_pub_.publish(*aruco_image_);
}


bool ArucoDetector2::Run() {
  pImageSub_->ParseData(dqBuffer_);

  if (!Detect()) {
    ImagePublish();
    return false;
  }

  if (!PoseEstimate())
    return false;

  ImagePublish();

  if (doPosePublish_)
    PosePublish();

  return true;
}


} // namespace my_aruco
