#include "my_aruco/front_optim.h"

namespace my_aruco
{

struct CostFunction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CostFunction(
      const Eigen::Quaterniond& Q1,
      const Eigen::Vector3d& t1,
      const Eigen::Quaterniond& Q2,
      const Eigen::Vector3d& t2,
      const Eigen::Quaterniond& Q12,
      const Eigen::Vector3d& t12)
    : Q1_(Q1), t1_(t1), Q2_(Q2), t2_(t2), Q12_(Q12), t12_(t12) { }

  template <typename T>
  bool operator()(const T* const dQ1, const T* const dt1, const T* const dQ2, const T* const dt2, T* residual) const {
    T dQ1_w = dQ1[-]
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d measuremnt) {
    return (new ceres::AutoDiffCostFunction<CostFunction, 4, 4, 3, 4, 3>(new CostFunction()));
  }

public:
  Eigen::Quaterniond Q12_;
  Eigen::Quaterniond Q1_;
  Eigen::Quaterniond Q2_;
  Eigen::Vector3d t12_;
  Eigen::Vector3d t1_;
  Eigen::Vector3d t2_;

};

FrontOptim::FrontOptim() {
  Q12_ = Eigen::Quaterniond::Identity();
  t12_ << 0.1458, 0, 0;

  dQ1_ = Eigen::Quaterniond::Identity();
  dt1_ = Eigen::Vector3d::Zero();
}

void FrontOptim::DoOptim(std::vector<Eigen::Matrix4d>& poses) {
  if (poses.size() < 2)
    return;

  Q1_ = Eigen::Quaterniond(poses[0].block(0, 0, 3, 3));
  Q2_ = Eigen::Quaterniond(poses[1].block(0, 0, 3, 3));
  t1_ = poses[0].block(0, 3, 3, 1);
  t1_ = poses[1].block(0, 3, 3, 1);

  ceres::Problem problem;
  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunction, 4, 2>(new CostFunction);


}

} // namespace my_aruco
