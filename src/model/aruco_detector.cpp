#include "my_aruco/model/aruco_detector.h"

namespace my_aruco
{

static Eigen::Quaterniond AddOffset(const Eigen::Quaterniond& input) {
  // Eigen::Matrix4d offsetMatrix = Eigen::Matrix4d::Identity();
  return input;
  // Eigen::Matrix3d offsetMatrix = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
  // return Eigen::Quaterniond(input.matrix() * offsetMatrix);
}

static double calculateYaw(const Eigen::Quaterniond& q) {
  Eigen::Vector3d vector = - q.matrix() * Eigen::Vector3d(0, 0, 1);
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

ArucoDetector::ArucoDetector(std::shared_ptr<cv::FileStorage> fs, ImageSubscriber::Ptr pImageSub)
  : fs_(fs), pImageSub_(pImageSub) {
  Initialize();
}

ArucoDetector::ArucoDetector(std::shared_ptr<cv::FileStorage> fs, ImageSubscriber::Ptr pImageSub, ros::NodeHandle& nh)
  : fs_(fs), pImageSub_(pImageSub), nh_(nh), bDoPublish_(true) {
  Initialize();

  // ROS Initialization
  pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>((std::string)(*fs_)["topic_name_marker_poses"], 100);
  aruco_image_pub_ = nh_.advertise<sensor_msgs::Image>((std::string)(*fs_)["topic_name_aruco_image"], 100);
  poseArray_.header.frame_id = (std::string)(*fs_)["camera_frame_id"];
}

void ArucoDetector::Initialize() {
  // camera parameters initialize
  (*fs_)["K"] >> cameraMatrix_;
  (*fs_)["D"] >> distCoeffs_;

  // aruco markers initialize
  markerSize_ = (double)(*fs_)["marker_length"];

  // opencv aruco initialize
  parameters_ = cv::aruco::DetectorParameters::create();
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
}


bool ArucoDetector::Detect() {
  if (dqBuffer_.size() == 0) return false;  // ! return if buffer is clear

  for (auto it = dqBuffer_.begin(); it != dqBuffer_.end(); ++it) {
    // todo make it a list
    timestamp_ = (**it).timestamp;
    image_ = (**it).pImage->clone();
    cv::aruco::detectMarkers(image_, dictionary_, markerCorners_, markerIds_, parameters_, rejectedCandidates_);
  }
  dqBuffer_.clear();
  return true;
}

bool ArucoDetector::PoseEstimate() {

  if (markerIds_.size() == 0)
    return false;

  cv::aruco::estimatePoseSingleMarkers(markerCorners_, markerSize_, cameraMatrix_, distCoeffs_, rvecs_, tvecs_);

  double yawSum = 0.0;
  qList_.clear();
  for (size_t i = 0; i < rvecs_.size(); ++i) {
    Eigen::Quaterniond q = AddOffset(GetQuaternion(rvecs_[i]));

    qList_.emplace_back(q);

    yawSum += calculateYaw(q);
  }

  yaw_ = yawSum / rvecs_.size();

  return true;
}

void ArucoDetector::PosePublish() {
  // Publish poses
  poseArray_.poses.clear();
  for (size_t i = 0; i < qList_.size(); ++i) {
    geometry_msgs::Pose pose;
    poseArray_.header.stamp = ros::Time::now(); // todo, use same timestamp as image

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

void ArucoDetector::ImagePublish() {
  // Publish Image
  for (size_t i = 0; i < markerIds_.size(); ++i) {
    cv::aruco::drawAxis(image_, cameraMatrix_, distCoeffs_, rvecs_[i], tvecs_[i], 0.1);
  }
  aruco_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
  aruco_image_pub_.publish(*aruco_image_);

}


bool ArucoDetector::Run() {
  pImageSub_->ParseData(dqBuffer_);

  if (!Detect()) {
    ImagePublish();
    return false;
  }

  if (!PoseEstimate())
    return false;

  ImagePublish();

  if (bDoPublish_)
    PosePublish();

  return true;
}


} // namespace my_aruco
