#include "my_aruco/optim/aruco_optimizer_kalman_filter.h"


namespace my_aruco::optim
{

ArucoOptimizerKalmanFilter::ArucoOptimizerKalmanFilter() : p_(p) {
  mode_ = Mode::StateModel;
}

void ArucoOptimizerKalmanFilter::Update(Markers& markers) {

}
  
} // namespace my_aruco:optim
