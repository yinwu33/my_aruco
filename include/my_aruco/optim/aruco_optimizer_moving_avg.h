#include "my_aruco/optim/aruco_optimizer.h"


namespace my_aruco::optim
{

class ArucoOptimzerMovingAvg : public ArucoOptimizer {
public:
  ArucoOptimzerMovingAvg(const Parameters* p);

  void Update(Markers& markers);

private:
  int windowSize_;
  std::vector<double> buffer_;

};
  
} // namespace my_aruco:optim
