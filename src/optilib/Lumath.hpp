#pragma once

#include <eigen3/Eigen/Dense>

Eigen::Matrix2d rotationDerivative(const Eigen::Rotation2Dd &R);

inline Eigen::Vector4d flatten(const Eigen::Matrix2d &M) {
  return M.reshaped();
};

Eigen::Vector4d flatten(const Eigen::Rotation2Dd &R);
