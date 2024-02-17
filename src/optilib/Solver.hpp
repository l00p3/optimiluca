#pragma once

#include "State.hpp"

#include <eigen3/Eigen/Dense>

namespace optilib {

class Solver {
public:
  std::vector<double> solve(State &state,
                            const std::vector<double> &measurements,
                            const int n_iters = 10, const bool verbose = false);

private:
  // Utility functions
  std::tuple<double, Eigen::MatrixXd>
  computeErrorAndJacobian(const State &state, const int &observer_id,
                          const int &observed_id,
                          const Eigen::Rotation2Dd &z_i) const;
};

} // namespace optilib
