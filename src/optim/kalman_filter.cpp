#include "my_aruco/optim/kalman_filter.h"


namespace my_aruco
{

KalmanFilter::KalmanFilter(const double Q, const double R, const double P
) : A_(1.0), C_(1.0), Q_(Q), R_(R), P0_(P), initialized_(false) {

}

void KalmanFilter::Init(double x0, const double t0) {
  x_last_ = x_curr_ = x0;
  t_last_ = t_curr_ = t0;

  P_ = P0_;

  initialized_ = true;
}


void KalmanFilter::Update(const double y, const double t) {
  if (!initialized_) {
    Init(y, t);
    return;
  }

  double v = getVelocity();

  // prediction
  x_pred_ = A_ * x_curr_ + v;
  P_ = A_ * P_ * A_ + Q_;

  // update
  K_ = P_ * C_ / (C_ * P_ * C_ + R_);
  P_ = (1 - K_ * C_) * P_;

  // a posterior state
  x_last_ = x_curr_;
  x_curr_ = x_pred_ + K_ * (y - C_ * x_pred_);
}


double KalmanFilter::getVelocity() {
  return (x_curr_ - x_last_) / (t_curr_ - t_last_);
}
} // namespace my_aruco
