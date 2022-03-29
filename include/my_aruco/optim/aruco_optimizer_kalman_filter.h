#include "my_aruco/optim/aruco_optimizer.h"


namespace my_aruco::optim
{
class ArucoOptimizerKalmanFilter : public ArucoOptimizer {
public:

  enum Mode {
    DDModel = 0, // differential drive, with vl, vr
    StateModel = 1 // 2D velocity and 1D angular velocity model
  };

  using Ptr = std::unique_ptr<ArucoOptimizerKalmanFilter>;

  ArucoOptimizerKalmanFilter() = delete;
  ArucoOptimizerKalmanFilter(const Parameters& p);

  void Update(Markers& markers);

private:
  Mode mode_;

};
} // namespace my_aruco

