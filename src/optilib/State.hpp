#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Rotation2D.h>
#include <ostream>

namespace optilib {

using Rot2D = Eigen::Rotation2Dd;

class State {
public:
  // Constructors
  State(const double &r1, const double &r2, const double &r3, const double &r4);

  // Methods
  size_t size() const;
  Rot2D get_rotation(const int &idx) const;

  // Operators
  State boxPlus(const double &dx) const;
  friend std::ostream &operator<<(std::ostream &os, const State &state);

private:
  std::vector<Eigen::Rotation2D<double>> _rotations;
};

} // namespace optilib
