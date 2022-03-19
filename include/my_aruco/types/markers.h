#include "my_aruco/common_include.hpp"
#include "my_aruco/types/image_stamped.hpp"


namespace my_aruco
{

struct Markers {

  ImageStamped image;

  void addImageStamped(const ImageStamped& imageStamped);

};
  
} // namespace my_aruco
