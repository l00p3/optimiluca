#include "Lumath.hpp"

namespace optilib {

Eigen::Matrix2d rotationDerivative(const Eigen::Rotation2Dd &R) {
  const double &angle = R.angle();
  return Eigen::Matrix2d{{-sin(angle), -cos(angle)}, {cos(angle), -sin(angle)}};
}

Eigen::Matrix4d v2T(const Eigen::Matrix<double, 6, 1> &v) {
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

Eigen::Matrix3d Rx(const double &angle) {
  return Eigen::Matrix3d(
      {{1, 0, 0}, {0, cos(angle), -sin(angle)}, {0, sin(angle), cos(angle)}});
}

Eigen::Matrix3d Ry(const double &angle) {
  return Eigen::Matrix3d(
      {{cos(angle), 0, sin(angle)}, {0, 1, 0}, {-sin(angle), 0, cos(angle)}});
};

Eigen::Matrix3d Rz(const double &angle) {
  return Eigen::Matrix3d(
      {{cos(angle), -sin(angle), 0}, {sin(angle), cos(angle), 0}, {0, 0, 1}});
};

Eigen::Matrix3d Rx_der(const double &angle) {
  return Eigen::Matrix3d(
      {{0, 0, 0}, {0, -sin(angle), -cos(angle)}, {0, cos(angle), -sin(angle)}});
}

Eigen::Matrix3d Ry_der(const double &angle) {
  return Eigen::Matrix3d(
      {{-sin(angle), 0, cos(angle)}, {0, 0, 0}, {-cos(angle), 0, -sin(angle)}});
};

Eigen::Matrix3d Rz_der(const double &angle) {
  return Eigen::Matrix3d(
      {{-sin(angle), -cos(angle), 0}, {cos(angle), -sin(angle), 0}, {0, 0, 0}});
};

Eigen::Matrix3d Rx_der_0() {
  return Eigen::Matrix3d({{0, 0, 0}, {0, 0, -1}, {0, 1, 0}});
}

Eigen::Matrix3d Ry_der_0() {
  return Eigen::Matrix3d({{0, 0, 1}, {0, 0, 0}, {-1, 0, 0}});
};

Eigen::Matrix3d Rz_der_0() {
  return Eigen::Matrix3d({{0, -1, 0}, {1, 0, 0}, {0, 0, 0}});
};

Eigen::Matrix4d T_inverse(const Eigen::Matrix4d &T) {
  Eigen::Matrix4d T_inverse = Eigen::Matrix4d::Identity();
  T_inverse.block<3, 1>(0, 3) =
      -T.block<3, 3>(0, 0).transpose() * T.block<3, 1>(0, 3);
  T_inverse.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
  return T_inverse;
}

} // namespace optilib
