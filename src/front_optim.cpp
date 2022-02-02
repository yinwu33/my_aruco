#include "my_aruco/front_optim.h"

namespace my_aruco
{

struct CostFunction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CostFunction(const Eigen::Quaterniond& Q12, const Eigen::Vector3d& t12)
    : Q12_(Q12), t12_(t12) { }

  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d measuremnt) {
    return (new ceres::AutoDiffCostFunction<CostFunction, 4, 1, 1>(new CostFunction()));
  }

public:
  Eigen::Quaterniond Q12_;
  Eigen::Vector3d t12_;

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
