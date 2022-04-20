#include "my_aruco/detector/aruco_detector_opencv.h"


namespace my_aruco::detector {

ArucoDetectorOpenCV::ArucoDetectorOpenCV(const Parameters& p, const ros::NodeHandle& nh)
  : ArucoDetector(p, nh) {
  dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  param_ = cv::aruco::DetectorParameters::create();
}

bool ArucoDetectorOpenCV::Detect() {
  if (!markers_.hasImage)
    return false;

  cv::aruco::detectMarkers(markers_.pImageStamped->image, dict_, markers_.markerCorners,
    markers_.markerIds, param_, markers_.rejectedCandidates);

  return !markers_.IsEmpty();
}

bool ArucoDetectorOpenCV::PoseEstimate() {
  cv::aruco::estimatePoseSingleMarkers(markers_.markerCorners, p_.markerSize,
    p_.cameraMatrix, p_.distCoeffs, markers_.rvecs, markers_.tvecs);

  return true;
}


}