#include <opencv2/aruco.hpp>

#include "my_aruco/detector/aruco_detector.h"
#include "my_aruco/utils/parameters.h"
#include "my_aruco/types/markers.h"


namespace my_aruco::detector {

class ArucoDetectorOpenCV : public ArucoDetector {
public:
  ArucoDetectorOpenCV() = delete;
  ArucoDetectorOpenCV(const Parameters&, const ros::NodeHandle&);

  bool Detect();

  bool PoseEstimate();

private:
  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::DetectorParameters> param_;
};

}