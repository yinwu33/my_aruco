#include "my_aruco/detector/aruco_detector_opencv.h"


namespace my_aruco::detector {

ArucoDetectorOpenCV::ArucoDetectorOpenCV(const Parameters& p)
  : ArucoDetector(p) {
  dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  param_ = cv::aruco::DetectorParameters::create();
  // todo set parameters
}

bool ArucoDetectorOpenCV::Detect(const ImageStamped& image) {
  markers_.AddImageStamped(image);

  cv::aruco::detectMarkers(markers_.imageStamped.image, dict_, markers_.markerCorners,
    markers_.markerIds, param_, markers_.rejectedCandidates, p_.cameraMatrix, p_.distCoeffs);

  return true;
}

bool ArucoDetectorOpenCV::PoseEstimate() {
  cv::aruco::estimatePoseSingleMarkers(markers_.markerCorners, p_.markerSize,
    p_.cameraMatrix, p_.distCoeffs, markers_.rvecs, markers_.tvecs);
}

}