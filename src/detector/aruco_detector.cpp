#include "my_aruco/detector/aruco_detector.h"


namespace my_aruco::detector
{
ArucoDetector::ArucoDetector(const Parameters& p, const ros::NodeHandle& nh) : p_(p), nh_(nh) {

  imageSub_ = nh_.subscribe(p_.topicImageRaw, 100, &ArucoDetector::ImageCallback, this);

  if (p_.mode == Mode::ARUCO_1D) {
    measurementPub_ = nh_.advertise<my_aruco_msg::AngleStamped>("measurement", 100);
  }
  else {
    throw;
  }
  // todo: other mode 3D, 6D
}

/**
 * @brief Pipeline of the detector
 *
 */
void ArucoDetector::Run() {
  if (!Detect())
    return;
  PoseEstimate();

  // 1D mode: detect yaw angle
  if (p_.mode == Mode::ARUCO_1D) {
    my_aruco_msg::AngleStamped msg;
    msg.header.stamp = markers_.pImageStamped->timestamp;
    GetYaw(msg.radian);
    msg.degree = msg.radian * 180 / M_PI;

    measurementPub_.publish(msg);
  }
}

void ArucoDetector::GetYaw(double& angle) {
  std::vector<double> yawList;

  for (size_t i = 0; i < markers_.rvecs.size(); ++i) {
    Eigen::Quaterniond q;
    markers_.GetQuaternion(markers_.rvecs[i], q);

    Eigen::Vector3d vector = -q.matrix() * p_.rotateVector;
    yawList.emplace_back(-atan2(vector[p_.projectPlane.first - 1], vector[p_.projectPlane.second - 1]));
  }

  // filter outlier when detect more than 4 arucos
  if (yawList.size() >= 4) {
    std::sort(yawList.begin(), yawList.end());
    double sum = 0.0;

    for (size_t i = 0; i < yawList.size(); ++i) {
      if (i == 0 or i == yawList.size() - 1)
        continue;

      angle = sum += yawList[i];
    }
    angle = sum / (yawList.size() - 2);
  }
  else {
    double sum = 0.0;
    for (const auto& y : yawList) {
      sum += y;
    }
    angle = sum / yawList.size();
  }
  return;
}

void ArucoDetector::ImageCallback(const sensor_msgs::Image::Ptr& msg) {
  // save the image into markers_
  markers_.SetImageStamped(std::make_shared<ImageStamped>(msg->header.stamp,
    cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image));
}



} // namespace my_aruco::detector
