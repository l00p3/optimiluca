#include "Lumath.hpp"

Eigen::Matrix2d rotationDerivative(const Eigen::Rotation2Dd &R) {
  const double &angle = R.angle();
  return Eigen::Matrix2d{{-sin(angle), -cos(angle)}, {cos(angle), -sin(angle)}};
}

Eigen::Vector4d flatten(const Eigen::Rotation2Dd &R) {
  const Eigen::Matrix2d &M = R.toRotationMatrix();
  return flatten(M);
}
