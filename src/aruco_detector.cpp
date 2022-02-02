#include "my_aruco/aruco_detector.h"

namespace my_aruco
{
static void GetQuaternion(cv::Vec3d aa, double q[]);

ArucoDetector::ArucoDetector()
  : parameters_(cv::aruco::DetectorParameters::create()), dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)) {

  cameraMatrix_ = (cv::Mat_<float>(3, 3) <<
    660.158566, 0.000000, 319.013077,
    0.000000, 662.195692, 246.567240,
    0.000000, 0.000000, 1.000000);

  distCoeffs_ = (cv::Mat_<float>(1, 4) << 0.039830, -0.139520, -0.001142, 0.000870, 0.000000);

  frame_id_ = "camera";

  markerSideLength_ = 0.079;

}

ArucoDetector::ArucoDetector(ros::NodeHandle& nh) : ArucoDetector() {
  ROS_INFO("Do Publish");
  pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("marker_poses", 100);
  aruco_image_pub_ = nh.advertise<sensor_msgs::Image>("aruco_image", 100);
  // std::cout << "debug" << std::endl;
  do_publish_ = true;

  poseArray_.header.frame_id = "camera";
}

bool ArucoDetector::Detect(std::deque<std::shared_ptr<cv::Mat>>& dq_buffer) {
  if (dq_buffer.size() == 0)
    return false;

  // std::cout << " buffer size: " << dq_buffer.size() << std::endl; // ! debug

  for (auto it = dq_buffer.begin(); it != dq_buffer.end(); ++it) {
    // todo make it a list
    image_ = (**it).clone();
    cv::aruco::detectMarkers(**it, dictionary_, markerCorners_, markerIds_, parameters_, rejectedCandidates_);
  }
  dq_buffer.clear();
  return true;
}

bool ArucoDetector::PoseEstimate() {

  if (markerCorners_.size() == 0)
    return false;

  // for (const auto id : markerIds_)
  //   std::cout << id << std::endl; // ! debug

  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(markerCorners_, markerSideLength_, cameraMatrix_, distCoeffs_, rvecs, tvecs);

  // drawing

  for (size_t i = 0; i < markerIds_.size(); ++i) {
    cv::aruco::drawAxis(image_, cameraMatrix_, distCoeffs_, rvecs[i], tvecs[i], 0.1);
  }

  aruco_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();

  if (do_publish_) {
    for (size_t i = 0; i < rvecs.size(); ++i) {
      geometry_msgs::Pose pose;
      poseArray_.header.stamp = ros::Time::now(); // todo, use same timestamp as image
      double q[4];

      GetQuaternion(rvecs[i], q);
      pose.orientation.x = q[0];
      pose.orientation.y = q[1];
      pose.orientation.z = q[2];
      pose.orientation.w = q[3];
      pose.position.x = tvecs[i][0];
      pose.position.y = tvecs[i][1];
      pose.position.z = tvecs[i][2];
      
      poseArray_.poses.emplace_back(pose);

    }

    if (poseArray_.poses.size() == 2) {
      double dx = abs(poseArray_.poses[0].position.x - poseArray_.poses[1].position.x);
      double dy = abs(poseArray_.poses[0].position.y - poseArray_.poses[1].position.y);
      double dz = abs(poseArray_.poses[0].position.z - poseArray_.poses[1].position.z);
      double dnorm = std::sqrt(dx*dx+dy*dy+dz*dz);
      std::cout << " distance: " << dx << " " << dy << " " << dz << " " << dnorm << std::endl;
      std::cout << "abs angle 0: " << 180 - cv::norm(rvecs[0])*180/M_PI << std::endl;
      std::cout << "abs angle 1: " << 180 - cv::norm(rvecs[1])*180/M_PI << std::endl;
      std::cout << " aa0 " << rvecs[0] << std::endl;
      std::cout << " aa1 " << rvecs[1] << std::endl;
      std::cout << " t0 " << tvecs[0] << std::endl;
      std::cout << " t1 " << tvecs[1] << std::endl;
    }

    Publish();
    poseArray_.poses.clear();
  }
  return true;
}

void ArucoDetector::Publish() {
  pose_pub_.publish(poseArray_);
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
static void GetQuaternion(cv::Vec3d aa, double q[]) {
  double angle = cv::norm(aa);
  cv::Matx31d axis(aa(0) / angle, aa(1) / angle, aa(2) / angle);
  double angle_2 = angle / 2;
  q[0] = axis(0) * sin(angle_2);
  q[1] = axis(1) * sin(angle_2);
  q[2] = axis(2) * sin(angle_2);
  q[3] = cos(angle_2);
}
} // namespace my_aruco
