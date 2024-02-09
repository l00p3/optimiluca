#pragma once

#include "State.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Rotation2D.h>

namespace optilib {

using Rot2D = Eigen::Rotation2Dd;

class Solver {
public:
  // Constructors
  Solver();

  std::tuple<State, std::vector<float>>
  solve(const State &state, const std::vector<double> &measurements,
        const int n_iters = 10);

private:
  // Utility functions
  std::tuple<double, Eigen::Matrix4d>
  computeErrorAndJacobian(const Rot2D &x_i, const Rot2D &x_j,
                          const Rot2D &z_i) const;

  // Here you put all the parameters (if any)
};

} // namespace optilib
