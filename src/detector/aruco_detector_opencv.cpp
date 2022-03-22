#include "my_aruco/detector/aruco_detector_opencv.h"


namespace my_aruco::detector {

ArucoDetectorOpenCV::ArucoDetectorOpenCV(const Parameters& p)
  : ArucoDetector(p) {
  dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  param_ = cv::aruco::DetectorParameters::create();
  // todo set parameters
}

bool ArucoDetectorOpenCV::Detect(my_aruco::Markers& markers) {

  cv::aruco::detectMarkers(markers.pImageStamped->image, dict_, markers.markerCorners,
    markers.markerIds, param_, markers.rejectedCandidates, p_.cameraMatrix, p_.distCoeffs);

  return !markers.IsEmpty();
}

bool ArucoDetectorOpenCV::PoseEstimate(my_aruco::Markers& markers) {
  cv::aruco::estimatePoseSingleMarkers(markers.markerCorners, p_.markerSize,
    p_.cameraMatrix, p_.distCoeffs, markers.rvecs, markers.tvecs);

  return true;
}


}