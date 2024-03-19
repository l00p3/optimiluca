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
           const double epsilon1 = 0.001, const double epsilon2 = 0.001)
      : _trust_region_radius(trust_region_radius), _epsilon1(epsilon1),
        _epsilon2(epsilon2){};

  std::vector<double> solve(State &state,
                            const std::vector<Measurement> &measurements,
                            const int n_iters = 10, const bool verbose = false);

private:
  double _trust_region_radius;
  double _epsilon1;
  double _epsilon2;
  Eigen::VectorXd _dx;
  double _dx_norm;
};

} // namespace optilib
