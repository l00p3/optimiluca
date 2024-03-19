#pragma once

#include "State.hpp"

namespace optilib {

class GNSolver {
public:
  std::vector<double> solve(State &state,
                            const std::vector<Measurement> &measurements,
                            const int n_iters = 100,
                            const bool verbose = false);
};

class DLSolver {
public:
  DLSolver(const double &trust_region_radius = 1e4,
           const double epsilon = 1e-10)
      : _trust_region_radius(trust_region_radius), _epsilon(epsilon){};

  std::vector<double> solve(State &state,
                            const std::vector<Measurement> &measurements,
                            const int n_iters = 10, const bool verbose = false);

private:
  double _trust_region_radius;
  double _epsilon;
  Eigen::VectorXd _dx;
  double _dx_norm;
};

} // namespace optilib
