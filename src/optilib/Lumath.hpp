#pragma once

#include <eigen3/Eigen/Dense>

namespace Eigen {
using Vector6d = Vector<double, 6>;
using Vector12d = Vector<double, 12>;
} // namespace Eigen

namespace optilib {

inline Eigen::Vector12d flatten(const Eigen::Matrix4d &T) {
  return T.block<3, 4>(0, 0).reshaped();
}

Eigen::Matrix4d v2T(const Eigen::Vector6d &v);

Eigen::Matrix3d skew(const Eigen::Vector3d &v);

// Rotation matrices functions
Eigen::Matrix4d T_inverse(const Eigen::Matrix4d &T);

} // namespace optilib
