#pragma once

#include "State.hpp"

#include <eigen3/Eigen/Dense>

namespace optilib {

class Solver {

public:
  std::vector<double> solve(State &state,
                            const std::vector<Measurement> &measurements,
                            const double termination_th = 1e-5,
                            const int n_iters = 10,
                            const int verbose_level = 1);

private:
  // Utility functions
  std::tuple<Eigen::Vector4d, Eigen::MatrixXd>
  computeErrorAndJacobian(const State &state, const Measurement &meas) const;
};

} // namespace optilib
