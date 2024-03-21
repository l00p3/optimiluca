#pragma once

#include <eigen3/Eigen/Dense>

Eigen::Matrix2d rotationDerivative(const Eigen::Rotation2Dd &R);

inline Eigen::Vector4d flatten(const Eigen::Matrix2d &M) {
  return M.reshaped();
};

inline Eigen::Vector4d flatten(const Eigen::Rotation2Dd &R) {
  return flatten(R.toRotationMatrix());
}

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
