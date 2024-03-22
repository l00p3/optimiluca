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
