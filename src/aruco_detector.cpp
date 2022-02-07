#include "my_aruco/aruco_detector.h"

namespace my_aruco
{
static void GetQuaternion(cv::Vec3d aa, double q[]) {
  double angle = cv::norm(aa);
  cv::Matx31d axis(aa(0) / angle, aa(1) / angle, aa(2) / angle);
  double angle_2 = angle / 2;
  q[0] = axis(0) * sin(angle_2);
  q[1] = axis(1) * sin(angle_2);
  q[2] = axis(2) * sin(angle_2);
  q[3] = cos(angle_2);
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
  if (dqBuffer_.size() == 0) return false;

  for (auto it = dqBuffer_.begin(); it != dqBuffer_.end(); ++it) {
    // todo make it a list
    image_ = (**it).clone();
    cv::aruco::detectMarkers(**it, dictionary_, markerCorners_, markerIds_, parameters_, rejectedCandidates_);
  }
  dqBuffer_.clear();
  return true;
}

bool ArucoDetector::PoseEstimate() {

  if (markerIds_.size() == 0)
    return false;

  cv::aruco::estimatePoseSingleMarkers(markerCorners_, markerSize_, cameraMatrix_, distCoeffs_, rvecs_, tvecs_);



  return true;
}

void ArucoDetector::Publish() {
  // Publish poses
  for (size_t i = 0; i < rvecs_.size(); ++i) {
    geometry_msgs::Pose pose;
    poseArray_.header.stamp = ros::Time::now(); // todo, use same timestamp as image
    double q[4];

    GetQuaternion(rvecs_[i], q);
    pose.orientation.x = q[0];
    pose.orientation.y = q[1];
    pose.orientation.z = q[2];
    pose.orientation.w = q[3];
    pose.position.x = tvecs_[i][0];
    pose.position.y = tvecs_[i][1];
    pose.position.z = tvecs_[i][2];

    poseArray_.poses.emplace_back(pose);

  }

  // if (poseArray_.poses.size() == 2) {
  //   double dx = abs(poseArray_.poses[0].position.x - poseArray_.poses[1].position.x);
  //   double dy = abs(poseArray_.poses[0].position.y - poseArray_.poses[1].position.y);
  //   double dz = abs(poseArray_.poses[0].position.z - poseArray_.poses[1].position.z);
  //   double dnorm = std::sqrt(dx * dx + dy * dy + dz * dz);
  //   std::cout << " distance: " << dx << " " << dy << " " << dz << " " << dnorm << std::endl;
  //   std::cout << "abs angle 0: " << 180 - cv::norm(rvecs_[0]) * 180 / M_PI << std::endl;
  //   std::cout << "abs angle 1: " << 180 - cv::norm(rvecs_[1]) * 180 / M_PI << std::endl;
  //   std::cout << " aa0 " << rvecs_[0] << std::endl;
  //   std::cout << " aa1 " << rvecs_[1] << std::endl;
  //   std::cout << " t0 " << tvecs_[0] << std::endl;
  //   std::cout << " t1 " << tvecs_[1] << std::endl;

  //   // test first pose
  //   Eigen::Quaterniond q0(
  //     poseArray_.poses[0].orientation.w,
  //     poseArray_.poses[0].orientation.x,
  //     poseArray_.poses[0].orientation.y,
  //     poseArray_.poses[0].orientation.z);
  //   double angle_y = abs(q0.toRotationMatrix().eulerAngles(2, 1, 3).y() * 180 / M_PI);

  //   std::cout << "angle y: " << (180 - angle_y) << std::endl;;
  // }


  pose_pub_.publish(poseArray_);
  poseArray_.poses.clear();
  // std::cout << "+++++++++++++++++" << std::endl;
  // Publish Image
  for (size_t i = 0; i < markerIds_.size(); ++i) {
    cv::aruco::drawAxis(image_, cameraMatrix_, distCoeffs_, rvecs_[i], tvecs_[i], 0.1);
  }
  aruco_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
  aruco_image_pub_.publish(*aruco_image_);
}

void ArucoDetector::Optim() {

}

/**
 * @brief Get the Quaternion object
 *
 * @param R
 * @param Q : x, y, z, w
 */


void ArucoDetector::Run() {
  pImageSub_->ParseData(dqBuffer_);

  Detect();

  PoseEstimate();

  if (bDoPublish_)
    Publish();
}

} // namespace my_aruco
