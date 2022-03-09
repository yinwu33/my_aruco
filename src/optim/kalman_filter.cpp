#include "my_aruco/optim/kalman_filter.h"


namespace my_aruco
{

KalmanFilter::KalmanFilter(const double Q, const double R, const double P
) : A_(1.0), C_(1.0), Q_(Q), R_(R), P0_(P), initialized_(false) {

}

void KalmanFilter::Init(double x0, const ros::Time t0) {
  x_last_ = x_curr_ = x0;
  t_last_ = t_curr_ = t0;

  P_ = P0_;

  initialized_ = true;
}


void KalmanFilter::Update(const double y, const ros::Time t) {
  if (!initialized_) {
    Init(y, t);
    return;
  }
  // std::cout << "---------" << t << std::endl;

  double v = getVelocity();

  // t_curr_ = t;

  // prediction
  x_pred_ = A_ * x_curr_ + v * ros::Duration(t - t_curr_).toSec();
  P_ = A_ * P_ * A_ + Q_;

  // update
  K_ = P_ * C_ / (C_ * P_ * C_ + R_);
  P_ = (1 - K_ * C_) * P_;

  // a posterior state
  x_last_ = x_curr_;
  t_last_ = t_curr_;

  // update current states
  t_curr_ = t;
  x_curr_ = x_pred_ + K_ * (y - C_ * x_pred_);
}


double KalmanFilter::getVelocity() {
  double v = (x_curr_ - x_last_) / ros::Duration(t_curr_ - t_last_).toSec();
  // std::cout << "v: " << v << std::endl; // ! debug
  // std::cout << t_curr_ << " " << t_last_ << std::endl;
  if (t_curr_ == t_last_) {
    // std::cout << "init" << std::endl;
    return 0.0;
  }
  return v;
}
} // namespace my_aruco
