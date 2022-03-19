#include "my_aruco/types/markers.h"

namespace my_aruco {

void Markers::addImageStamped(const ImageStamped& imageStamped) {
  image.time = imageStamped.time;
  image.timestamp = imageStamped.timestamp;
  image.pImage = std::make_shared<cv::Mat>(imageStamped.pImage->clone());
}
}