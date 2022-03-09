#ifndef MY_ARUCO_KALMAN_FILTER_H
#define MY_ARUCO_KALMAN_FILTER_H

#include <iostream>

namespace my_aruco
{
/**
 * @brief
 * model:
 * xk = x_k-1 + v_k + w_k
 * yk = x_k + n_k
 *
 * Covariance
 * x: P
 * v: Q
 * w: R
 *
 */
class KalmanFilter {
public:
  KalmanFilter() = default;
  KalmanFilter(
    const double Q,
    const double R,
    const double P
  );

  void Init(double x0, const double t0);

  void Update(const double y, const double t);


  double getState() { return x_hat_; }
  double getTime() { return t_; }

  /**
   * @brief Get angular velocity
   *
   * @return double
   */
  double getVelocity();


private:
  // modell
  double A_, B_, C_, D_;

  // covariance
  double P0_, P_, Q_, R_;

  // kalman gain
  double K_;

  double t_last_, t_curr_;
  double x_last_, x_curr_;

  double x_pred_;

  double t0_, t_;

  double dt_;


  bool initialized_;

  double x_hat_, x_hat_new_;
};

} // namespace my_aruco


#endif // MY_ARUCO_KALMAN_FILTER_H