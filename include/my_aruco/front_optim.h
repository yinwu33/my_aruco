#ifndef MY_ARUCO_FRONT_OPTIM_H
#define MY_ARUCO_FRONT_OPTIM_H

#include <vector>

#include <Eigen/Core>
#include <ceres/ceres.h>
namespace my_aruco {
class FrontOptim {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FrontOptim();

  void DoOptim(std::vector<Eigen::Matrix4d> &poses);

private:
  Eigen::Quaterniond Q1_, Q2_;
  Eigen::Quaterniond dQ1_, dQ2_;

  Eigen::Vector3d t1_, t2_;
  Eigen::Vector3d dt1_, dt2_;

  Eigen::Quaterniond Q12_;
  Eigen::Vector3d t12_;
};

}

#endif 