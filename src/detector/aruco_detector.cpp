#include "my_aruco/detector/aruco_detector.h"


namespace my_aruco::detector
{
  ArucoDetector::ArucoDetector(const Parameters& p) : p_(p) {
    
  }

  void ArucoDetector::GetYaw() {
    std::vector<double> yawList;

    for (size_t i = 0; i < markers_.rvecs.size(); ++i) {
      Eigen::Quaterniond q;
      markers_.GetQuaternion(markers_.rvecs[i], q);

      Eigen::Vector3d vector = -q.matrix() * p_.rotateVector;
      yawList.emplace_back(atan2(vector[p_.projectPlane.first], vector[p_.projectPlane.second]));
    }

    // filter outlier when detect more than 4 arucos
    if (yawList.size() >=4) {
      std::sort(yawList.begin(), yawList.end());
      double sum = 0.0;

      for (size_t i = 0; i < yawList.size(); ++i) {
        if (i == 0 or i < yawList.size() - 1)
          continue;

        sum += yawList[i];
      }
      markers_.yaw = sum / (yawList.size() - 2);
    }
    else {
      double sum = 0.0;
      for (const auto& y : yawList) {
        sum += y;
      }
      markers_.yaw = sum / yawList.size();
    }
    return;
  }
  
} // namespace my_aruco::detector
