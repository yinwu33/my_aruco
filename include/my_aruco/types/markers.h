#include "my_aruco/common_include.hpp"
#include "my_aruco/types/image_stamped.hpp"


namespace my_aruco
{

struct Markers {

  ImageStamped imageStamped;

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  std::vector<std::vector<cv::Point2f>> rejectedCandidates;
  std::vector<cv::Vec3d> rvecs;
  std::vector<cv::Vec3d> tvecs;

  void AddImageStamped(const ImageStamped&);

  static void GetQuaternion(const cv::Vec3d&, Eigen::Quaterniond&);

};
  
} // namespace my_aruco
