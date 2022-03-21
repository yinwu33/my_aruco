#include "my_aruco/types/markers.h"

namespace my_aruco {

void Markers::AddImageStamped(const ImageStamped& _imageStamped) {
  imageStamped.timestamp = _imageStamped.timestamp;
  imageStamped.image = _imageStamped.image.clone();
}

void Markers::GetQuaternion(const cv::Vec3d& aa, Eigen::Quaterniond& q) {
  double angle = cv::norm(aa);
  double angle_2 = angle / 2;
  q = Eigen::Quaterniond(cos(angle_2), aa(0) / angle * sin(angle_2), aa(1) / angle * sin(angle_2), aa(2) / angle * sin(angle_2));
  return;
}

}