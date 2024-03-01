#include "Lumath.hpp"

Eigen::Matrix2d rotationDerivative(const Eigen::Rotation2Dd &R) {
  const double &angle = R.angle();
  return Eigen::Matrix2d{{-sin(angle), -cos(angle)}, {cos(angle), -sin(angle)}};
}

Eigen::Vector4d flatten(const Eigen::Matrix2d &M) {
  return Eigen::Vector4d{M(0, 0), M(1, 0), M(0, 1), M(1, 1)};
}
