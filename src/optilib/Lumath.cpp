#include "Lumath.hpp"

namespace optilib {

Eigen::Matrix4d v2T(const Eigen::Vector6d &v) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  const double angle = v.template tail<3>().norm();
  const Eigen::Vector3d axis = v.template tail<3>().normalized();
  T.block<3, 3>(0, 0) = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
  T.block<3, 1>(0, 3) = v.template head<3>();
  return T;
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  return Eigen::Matrix3d(
      {{0, -v(2), v(1)}, {v(2), 0, -v(0)}, {-v(1), v(0), 0}});
}

Eigen::Matrix4d T_inverse(const Eigen::Matrix4d &T) {
  Eigen::Matrix4d T_inverse = Eigen::Matrix4d::Identity();
  T_inverse.block<3, 1>(0, 3) =
      -T.block<3, 3>(0, 0).transpose() * T.block<3, 1>(0, 3);
  T_inverse.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
  return T_inverse;
}

} // namespace optilib
