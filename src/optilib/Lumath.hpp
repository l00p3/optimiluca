#pragma once

#include <eigen3/Eigen/Dense>

namespace optilib {

Eigen::Matrix2d rotationDerivative(const Eigen::Rotation2Dd &R);

inline Eigen::VectorXd flatten(const Eigen::Matrix4d &T) {
  return T.block<3, 4>(0, 0).reshaped();
}

Eigen::Matrix4d v2T(const Eigen::Matrix<double, 6, 1> &v);

Eigen::Matrix3d skew(const Eigen::Vector3d &v);

// Rotation matrices functions
Eigen::Matrix3d Rx(const double &angle);
Eigen::Matrix3d Ry(const double &angle);
Eigen::Matrix3d Rz(const double &angle);

Eigen::Matrix3d Rx_der(const double &angle);
Eigen::Matrix3d Ry_der(const double &angle);
Eigen::Matrix3d Rz_der(const double &angle);

Eigen::Matrix3d Rx_der_0();
Eigen::Matrix3d Ry_der_0();
Eigen::Matrix3d Rz_der_0();

Eigen::Matrix4d T_inverse(const Eigen::Matrix4d &T);

} // namespace optilib
