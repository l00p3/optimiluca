#pragma once

#include <eigen3/Eigen/Dense>
#include <ostream>

namespace optilib {

class State {
public:
  // Constructors
  State(const double &r1, const double &r2, const double &r3, const double &r4);

  // Methods
  size_t size() const;

  // Operators
  State boxPlus(const double &dx) const;
  friend std::ostream &operator<<(std::ostream &os, const State &state);

private:
  std::vector<Eigen::Rotation2D<double>> _rotations;
};

} // namespace optilib
