#pragma once

#include "State.hpp"

namespace optilib {

class Solver {

public:
  virtual std::vector<double>
  solve(State &state, const std::vector<Measurement> &measurements,
        const int n_iters, const int verbose_level) = 0;
};

class GSSolver : public Solver {
public:
  std::vector<double> solve(State &state,
                            const std::vector<Measurement> &measurements,
                            const int n_iters = 10,
                            const int verbose_level = 1);
};

class DLSolver : public Solver {
public:
  DLSolver(const double &trust_region_radius = 0.01)
      : _trust_region_radius(trust_region_radius){};

  std::vector<double> solve(State &state,
                            const std::vector<Measurement> &measurements,
                            const int n_iters = 10,
                            const int verbose_level = 1);

private:
  double _trust_region_radius;
  Eigen::VectorXd _dx;
  double _dx_norm;
};

} // namespace optilib
