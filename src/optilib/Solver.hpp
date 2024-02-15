#pragma once

#include "State.hpp"

#include <eigen3/Eigen/Dense>

namespace optilib {

using RowVec4D = Eigen::Matrix<double, 1, 4>;

class Solver {
public:
  std::vector<double> solve(State &state,
                            const std::vector<double> &measurements,
                            const int n_iters = 10, const bool verbose = false);

private:
  // Utility functions
  std::tuple<double, RowVec4D>
  computeErrorAndJacobian(const State &state, const int &observer_id,
                          const int &observed_id,
                          const Eigen::Rotation2Dd &z_i) const;
};

} // namespace optilib
