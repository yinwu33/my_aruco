#include "my_aruco/types/markers.h"

namespace my_aruco {

void Markers::SetImageStamped(const ImageStamped::Ptr& _pImageStamped) {
  pImageStamped = _pImageStamped;

  hasImage = true;
}

void Markers::Draw(Parameters& p, cv::Mat& image) {
  for (size_t i = 0; i < GetNum(); ++i) {
    cv::aruco::drawAxis(image, p.cameraMatrix, p.distCoeffs, rvecs[i], tvecs[i], 0.1);
  }
}

void Markers::GetQuaternion(const cv::Vec3d& aa, Eigen::Quaterniond& q) {
  double angle = cv::norm(aa);
  double angle_2 = angle / 2;
  q = Eigen::Quaterniond(cos(angle_2), aa(0) / angle * sin(angle_2), aa(1) / angle * sin(angle_2), aa(2) / angle * sin(angle_2));
  return;
}

int Markers::GetNum() {
  return markerIds.size();
}

bool Markers::IsEmpty() {
  return GetNum() == 0;
}

void Markers::Clear() {
  pImageStamped = nullptr;
  markerIds.clear();
  markerCorners.clear();
  rejectedCandidates.clear();
  rvecs.clear();
  tvecs.clear();
  yaw = 0.0;
  optimizedYaw = 0.0;
  pose2D = Eigen::Vector3d::Zero();
  pose3D = Eigen::Matrix4d::Identity();

  hasImage = false;
}

}