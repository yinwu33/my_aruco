#pragma once

#include "my_aruco/common_include.hpp"
#include "my_aruco/types/image_stamped.hpp"
#include "my_aruco/utils/parameters.h"


namespace my_aruco
{

struct Markers {

  using Ptr = std::shared_ptr<Markers>;

  bool hasImage = false;

  ImageStamped::Ptr pImageStamped = nullptr;

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  std::vector<std::vector<cv::Point2f>> rejectedCandidates;
  std::vector<cv::Vec3d> rvecs;
  std::vector<cv::Vec3d> tvecs;

  // result data
  double yaw, optimizedYaw;
  Eigen::Vector3d pose2D;
  Eigen::Matrix4d pose3D;

  void SetImageStamped(const ImageStamped::Ptr&);

  void Draw(Parameters& p, cv::Mat&);

  static void GetQuaternion(const cv::Vec3d&, Eigen::Quaterniond&);

  int GetNum();

  bool IsEmpty();

  void Clear();

};
  
} // namespace my_aruco
