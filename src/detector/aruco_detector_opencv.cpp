#include "my_aruco/detector/aruco_detector_opencv.h"


namespace my_aruco::detector {

  ArucoDetectorOpenCV::ArucoDetectorOpenCV(const Parameters& p)
   : ArucoDetector(p) {
    dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    param_ = cv::aruco::DetectorParameters::create();
    // todo set parameters
  }

  bool ArucoDetectorOpenCV::Detect(const ImageStamped& image) {


  }

}