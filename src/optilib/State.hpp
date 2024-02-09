#pragma once

#include <eigen3/Eigen/Dense>

namespace optilib {

class State {
public:
  // Constructors
  State(const double &r1, const double &r2, const double &r3, const double &r4);

  // Operators
  State boxPlus(const double &dx) const;

private:
  std::vector<Eigen::Rotation2D<double>> _rotations;
};

} // namespace optilib
