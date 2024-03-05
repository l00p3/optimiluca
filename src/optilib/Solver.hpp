#pragma once

#include "State.hpp"

#include <eigen3/Eigen/Dense>

namespace optilib {

class Solver {
public:
  std::vector<double> solve(State &state,
                            const std::vector<double> &measurements,
                            const double termination_th = 1e-5,
                            const int n_iters = 10, const bool verbose = false);

private:
  // Utility functions
  std::tuple<Eigen::Vector4d, Eigen::MatrixXd>
  computeErrorAndJacobian(const State &state, const int from_idx,
                          const int to_idx,
                          const Eigen::Rotation2Dd &z_i) const;
};

} // namespace optilib
