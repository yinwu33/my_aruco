#include "my_aruco/optim/aruco_optimizer.h"


namespace my_aruco::optim
{

class ArucoOptimzerMovingAvg : public ArucoOptimizer {
public:
  ArucoOptimzerMovingAvg(const Parameters* p);
};
  
} // namespace my_aruco:optim
